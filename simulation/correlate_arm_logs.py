#!/usr/bin/env python3
"""Correlate Pico event logs with locked ARM probe JSON by generation."""

import argparse
import json
import re
from collections import Counter, defaultdict
from pathlib import Path


EVENTS = {
    "TCP_ACCEPT",
    "READY_TX_BEGIN",
    "READY_TX_OK",
    "READY_TX_ERROR",
    "ARM_RX",
    "ARM_PENDING",
    "ACK_TX_BEGIN",
    "ACK_TX_PARTIAL",
    "ACK_TX_OK",
    "ACK_TX_ERROR",
    "ARM_COMMIT",
    "ARM_EXPIRED",
    "STOP_LATCHED",
    "TCP_CLOSE",
    "TCP_RECONNECT",
    "EXCEPTION",
}
FIELD = re.compile(r"([a-zA-Z_]+)=([^ ]+)")


def parse_value(value):
    try:
        return int(value)
    except ValueError:
        try:
            return float(value)
        except ValueError:
            return value


def parse_pico_events(text):
    events = []
    for line_number, line in enumerate(text.splitlines(), 1):
        event = line.split(" ", 1)[0]
        if event not in EVENTS:
            continue
        fields = {
            key: parse_value(value) for key, value in FIELD.findall(line)
        }
        fields.update(event=event, line=line_number)
        events.append(fields)
    return events


def classify(generation, pico_events, host_result):
    names = [item["event"] for item in pico_events]
    host_ack = bool(host_result and host_result.get("arm_rejected_lock"))
    if "ARM_RX" not in names:
        return "A_NO_ARM_RX"
    if "ACK_TX_BEGIN" not in names:
        return "A_STOPPED_BEFORE_ACK_GENERATION"
    if "ACK_TX_ERROR" in names:
        return "B_ACK_SEND_ERROR"
    if "ACK_TX_OK" in names and not host_ack:
        return "C_PICO_SEND_OK_HOST_DID_NOT_RECEIVE"
    if "ACK_TX_OK" in names and host_ack:
        return "PASS_ACK_CORRELATED"
    return "D_SESSION_STOPPED_WITHOUT_TERMINAL_ACK_EVENT"


def correlate(pico_text, host_report):
    pico_events = parse_pico_events(pico_text)
    by_generation = defaultdict(list)
    for event in pico_events:
        generation = event.get("generation")
        if generation is not None:
            by_generation[generation].append(event)
    host_by_generation = {
        item["generation"]: item for item in host_report.get("results", [])
    }
    generations = sorted(set(by_generation) | set(host_by_generation))
    rows = []
    for generation in generations:
        events = by_generation.get(generation, [])
        host = host_by_generation.get(generation)
        rows.append(
            {
                "generation": generation,
                "classification": classify(generation, events, host),
                "pico_connection_ids": sorted(
                    {
                        item["connection_id"]
                        for item in events
                        if isinstance(item.get("connection_id"), int)
                    }
                ),
                "pico_events": [item["event"] for item in events],
                "host_ready": None if host is None else host.get("ready"),
                "host_ack": None
                if host is None
                else host.get("arm_rejected_lock"),
                "host_reconnect": None
                if host is None
                else host.get("reconnect_ready"),
                "host_error": None if host is None else host.get("error"),
            }
        )
    return {
        "pico_event_counts": dict(Counter(item["event"] for item in pico_events)),
        "classifications": dict(Counter(row["classification"] for row in rows)),
        "rows": rows,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pico_log", type=Path)
    parser.add_argument("host_json", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = correlate(
        args.pico_log.read_text(encoding="utf-8", errors="replace"),
        json.loads(args.host_json.read_text(encoding="utf-8")),
    )
    rendered = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
