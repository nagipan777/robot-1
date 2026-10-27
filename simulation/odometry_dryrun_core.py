"""ROS-independent encoder CSV schema, replay validation, and state handling."""

import csv
import math
from dataclasses import dataclass
from pathlib import Path

try:
    from localization_core import WheelOdometry
except ModuleNotFoundError:  # Package import in host-side unit tests.
    from .localization_core import WheelOdometry


CONNECTED = "connected"
DISCONNECTED_STATES = frozenset(("disconnected", "timeout"))
COMMUNICATION_STATES = frozenset((CONNECTED,)) | DISCONNECTED_STATES


@dataclass(frozen=True)
class EncoderSample:
    timestamp: float
    left_encoder: int | None
    right_encoder: int | None
    valid: bool
    communication_state: str
    sequence: int
    error: str = ""


@dataclass(frozen=True)
class ProcessedSample:
    sample: EncoderSample
    update: object | None
    integrated: bool
    reason: str


def parse_bool(value):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes"):
        return True
    if normalized in ("0", "false", "no"):
        return False
    raise ValueError("valid must be true or false")


def parse_encoder_sample(row):
    timestamp = float(row["timestamp"])
    if not math.isfinite(timestamp):
        raise ValueError("timestamp must be finite")
    valid = parse_bool(row.get("valid", True))
    state = str(row.get("communication_state", CONNECTED)).strip().lower()
    if state not in COMMUNICATION_STATES:
        raise ValueError("unsupported communication_state: {}".format(state))
    sequence = int(row.get("sequence", 0))
    left_raw = row.get("left_encoder", "")
    right_raw = row.get("right_encoder", "")
    left = None if left_raw in (None, "") else int(left_raw)
    right = None if right_raw in (None, "") else int(right_raw)
    if valid and state == CONNECTED and (left is None or right is None):
        raise ValueError("connected valid sample requires both encoders")
    return EncoderSample(
        timestamp,
        left,
        right,
        valid,
        state,
        sequence,
        str(row.get("error", "")),
    )


def load_encoder_csv(path):
    samples = []
    previous_timestamp = None
    previous_sequence = None
    with Path(path).open(newline="", encoding="utf-8") as source:
        for line_number, row in enumerate(csv.DictReader(source), start=2):
            try:
                sample = parse_encoder_sample(row)
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError("line {}: {}".format(line_number, error)) from error
            if previous_timestamp is not None and sample.timestamp <= previous_timestamp:
                raise ValueError("line {}: timestamp must increase".format(line_number))
            if previous_sequence is not None and sample.sequence <= previous_sequence:
                raise ValueError("line {}: sequence must increase".format(line_number))
            samples.append(sample)
            previous_timestamp = sample.timestamp
            previous_sequence = sample.sequence
    if not samples:
        raise ValueError("encoder CSV has no samples")
    return samples


class DryRunOdometry:
    """Stop integration on invalid input and re-baseline after communication loss."""

    def __init__(self, estimator, max_time_gap_s=1.0):
        if not all(
            hasattr(estimator, name)
            for name in ("update", "resynchronize", "pose", "last_timestamp")
        ):
            raise TypeError("estimator must provide the WheelOdometry interface")
        if not math.isfinite(max_time_gap_s) or max_time_gap_s <= 0:
            raise ValueError("max_time_gap_s must be positive")
        self.estimator = estimator
        self.max_time_gap_s = float(max_time_gap_s)
        self.connected = False

    def process(self, sample):
        if not isinstance(sample, EncoderSample):
            raise TypeError("sample must be EncoderSample")
        if not sample.valid or sample.communication_state != CONNECTED:
            self.estimator.resynchronize()
            self.connected = False
            return ProcessedSample(sample, None, False, sample.error or sample.communication_state)
        if self.estimator.last_timestamp is not None:
            gap = sample.timestamp - self.estimator.last_timestamp
            if gap > self.max_time_gap_s:
                self.estimator.resynchronize()
                self.connected = False
        update = self.estimator.update(
            sample.left_encoder, sample.right_encoder, sample.timestamp
        )
        was_connected = self.connected
        self.connected = True
        reason = "integrated" if update.initialized and was_connected else "baseline"
        return ProcessedSample(sample, update, update.initialized and was_connected, reason)
