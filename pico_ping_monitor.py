#!/usr/bin/env python3
"""Collect timestamped ICMP reachability samples without contacting TCP 9000."""

import argparse
import csv
import json
import platform
import re
import shutil
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path


LATENCY_PATTERN = re.compile(r"time[=<]([0-9.]+)\s*ms")


def ping_command(host, timeout_s, system=None, executable=None):
    system = system or platform.system()
    executable = executable or shutil.which("ping") or "/sbin/ping"
    if system == "Darwin":
        return [executable, "-n", "-c", "1", "-W", str(int(timeout_s * 1000)), host]
    return [executable, "-n", "-c", "1", "-W", str(max(1, int(timeout_s))), host]


def parse_latency(output):
    match = LATENCY_PATTERN.search(output)
    return float(match.group(1)) if match else None


def run_ping(host, timeout_s):
    try:
        completed = subprocess.run(
            ping_command(host, timeout_s),
            capture_output=True,
            text=True,
            timeout=timeout_s + 1.0,
            check=False,
        )
        output = completed.stdout + completed.stderr
        return completed.returncode == 0, parse_latency(output), completed.returncode
    except subprocess.TimeoutExpired:
        # A missing host can outlive ping's platform-specific wait option.
        # Treat it as one failed sample instead of aborting the whole run.
        return False, None, 124


def summarize(samples):
    successes = sum(1 for sample in samples if sample["success"])
    first = next((sample for sample in samples if sample["success"]), None)
    longest = current = 0
    for sample in samples:
        if sample["success"]:
            current = 0
        else:
            current += 1
            longest = max(longest, current)
    latencies = [sample["latency_ms"] for sample in samples if sample["latency_ms"] is not None]
    return {
        "attempts": len(samples),
        "successes": successes,
        "failures": len(samples) - successes,
        "response_rate_percent": round(100.0 * successes / len(samples), 2) if samples else 0.0,
        "first_response_seconds": first["elapsed_seconds"] if first else None,
        "longest_consecutive_failures": longest,
        "average_latency_ms": round(sum(latencies) / len(latencies), 3) if latencies else None,
        "max_latency_ms": max(latencies) if latencies else None,
    }


def collect(host, label, duration_s, interval_s, timeout_s, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    run_id = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    stem = output_dir / f"pico_ping_{label}_{run_id}"
    csv_path = stem.with_suffix(".csv")
    json_path = stem.with_suffix(".json")
    started_wall = datetime.now(timezone.utc)
    started_mono = time.monotonic()
    samples = []

    print(f"start label={label} host={host}; power on the Pico W now")
    try:
        while time.monotonic() - started_mono < duration_s:
            attempt_started = time.monotonic()
            success, latency_ms, return_code = run_ping(host, timeout_s)
            sample = {
                "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                "elapsed_seconds": round(time.monotonic() - started_mono, 3),
                "success": success,
                "latency_ms": latency_ms,
                "return_code": return_code,
            }
            samples.append(sample)
            state = "OK" if sample["success"] else "FAIL"
            latency = "" if sample["latency_ms"] is None else f" {sample['latency_ms']:.3f}ms"
            print(f"{sample['timestamp']} +{sample['elapsed_seconds']:.3f}s {state}{latency}", flush=True)
            remaining = interval_s - (time.monotonic() - attempt_started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("collection interrupted; saving collected samples")

    summary = summarize(samples)
    summary.update(
        {
            "label": label,
            "host": host,
            "started_at_utc": started_wall.isoformat(),
            "requested_duration_seconds": duration_s,
            "interval_seconds": interval_s,
            "ping_timeout_seconds": timeout_s,
            "csv_path": str(csv_path),
        }
    )
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=samples[0].keys() if samples else ["timestamp"])
        writer.writeheader()
        writer.writerows(samples)
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"saved: {csv_path}\nsaved: {json_path}")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.0.29")
    parser.add_argument("--label", required=True, help="Example: mac-usb or mobile-battery")
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/pico_ping"))
    args = parser.parse_args()
    if args.duration <= 0 or args.interval <= 0 or args.timeout <= 0:
        parser.error("duration, interval, and timeout must be positive")
    collect(args.host, args.label, args.duration, args.interval, args.timeout, args.output_dir)


if __name__ == "__main__":
    main()
