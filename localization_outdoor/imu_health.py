"""Read-only IMU delivery diagnostics; never changes estimator readiness."""

from collections import deque
import threading
import time


class ImuHealth:
    """Keep bounded interruption history independently of browser polling."""

    def __init__(self, raw_timeout_s, attitude_timeout_s):
        self.limits = {"raw": raw_timeout_s, "attitude": attitude_timeout_s}
        self.events = deque(maxlen=20)
        self.gap_at = None
        self.recovered_at = None
        self.lock = threading.RLock()

    def update(self, now, stamps, wall_now=None):
        wall_now = time.time() if wall_now is None else wall_now
        with self.lock:
            missing = [at + self.limits[key] for key, at in stamps.items()
                       if at is not None and now - at >= self.limits[key]]
            if missing and self.gap_at is None:
                self.gap_at = min(missing)
                self.events.append({"kind": "interrupted", "at_unix_s": wall_now-(now-self.gap_at)})
            elif not missing and self.gap_at is not None and all(at is not None for at in stamps.values()):
                self.events.append({"kind": "recovered", "at_unix_s": wall_now,
                                    "duration_s": max(0., now-self.gap_at)})
                self.gap_at = None
                self.recovered_at = now

    def snapshot(self, now, stamps, serial_connected):
        with self.lock:
            ages = {key: max(0., now-at) if at is not None else None
                    for key, at in stamps.items()}
            missing = [key for key, age in ages.items()
                       if age is not None and age >= self.limits[key]]
            state = ("interrupted" if missing else "waiting" if None in ages.values()
                     else "recovered" if self.recovered_at is not None and now-self.recovered_at < 3.
                     else "normal")
            return {"state": state, "raw_age_s": ages["raw"],
                    "attitude_age_s": ages["attitude"], "missing": missing,
                    "serial_connected": serial_connected,
                    "interruption_s": max(0., now-self.gap_at) if self.gap_at is not None else 0.,
                    "events": [dict(event) for event in self.events]}
