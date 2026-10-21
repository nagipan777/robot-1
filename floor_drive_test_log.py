#!/usr/bin/env python3
"""Hardware-independent evidence log for a bounded Roomba floor test."""

import argparse
import csv
import json
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path


RESULTS = {"PASS", "FAIL", "NOT_RUN"}
STAGES = {"stage0", "stage1", "stage2", "stage3", "stage4"}


def utc_now():
    return datetime.now(timezone.utc).isoformat()


@dataclass
class FloorDriveEvent:
    event_type: str
    occurred_at: str = field(default_factory=utc_now)
    monotonic_ms: int | None = None
    arm_generation: int | None = None
    drive_sequence: int | None = None
    left_mm_s: int | None = None
    right_mm_s: int | None = None
    uart_ack: str | None = None
    watchdog_state: str | None = None
    stop_reason: str | None = None
    encoder_left: int | None = None
    encoder_right: int | None = None
    encoder_delta_left: int | None = None
    encoder_delta_right: int | None = None
    measured_distance_m: float | None = None
    measured_yaw_rad: float | None = None
    physical_stop_observed: bool | None = None
    note: str = ""


@dataclass
class FloorDriveRecord:
    test_id: str
    stage: str
    result: str = "NOT_RUN"
    started_at: str = field(default_factory=utc_now)
    completed_at: str | None = None
    safety_flags: dict = field(
        default_factory=lambda: {
            "allow_nonzero_drive": False,
            "drive_enabled": False,
            "control_enabled": False,
            "calibration_ready": False,
        }
    )
    environment: dict = field(default_factory=dict)
    events: list[FloorDriveEvent] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    def __post_init__(self):
        if not self.test_id or any(character.isspace() for character in self.test_id):
            raise ValueError("test_id must be non-empty and contain no whitespace")
        if self.stage not in STAGES:
            raise ValueError("stage must be one of {}".format(sorted(STAGES)))
        if self.result not in RESULTS:
            raise ValueError("result must be PASS, FAIL, or NOT_RUN")

    def add_event(self, event_type, **values):
        if not event_type:
            raise ValueError("event_type is required")
        event = FloorDriveEvent(event_type=event_type, **values)
        self.events.append(event)
        return event

    def finalize(self, result, note=None):
        if result not in RESULTS:
            raise ValueError("result must be PASS, FAIL, or NOT_RUN")
        self.result = result
        self.completed_at = utc_now()
        if note:
            self.notes.append(str(note))

    def write(self, output_directory):
        output_directory = Path(output_directory)
        output_directory.mkdir(parents=True, exist_ok=True)
        json_path = output_directory / "{}.json".format(self.test_id)
        csv_path = output_directory / "{}-events.csv".format(self.test_id)
        payload = asdict(self)
        json_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        fieldnames = list(asdict(FloorDriveEvent("schema")).keys())
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for event in self.events:
                writer.writerow(asdict(event))
        return json_path, csv_path


def main():
    parser = argparse.ArgumentParser(
        description="Create a non-driving floor-test evidence template"
    )
    parser.add_argument("--test-id", required=True)
    parser.add_argument("--stage", required=True, choices=sorted(STAGES))
    parser.add_argument("--output", default="artifacts/floor_drive_tests")
    parser.add_argument("--note", default="Prepared only; no physical run performed")
    args = parser.parse_args()
    record = FloorDriveRecord(args.test_id, args.stage)
    record.add_event("test_prepared", note=args.note)
    record.finalize("NOT_RUN", args.note)
    json_path, csv_path = record.write(args.output)
    print(json_path)
    print(csv_path)


if __name__ == "__main__":
    main()
