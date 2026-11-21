"""Scans of the RPLidar taken while the robot stands still.

Used by the LiDAR calibrations and by tests/testOdometryAccuracy.py
(LidarTruth, the ground truth of the odometry test, is built on it).
"""
from ..lidar_reader import rotate_scan

LIDAR_SCANS_PER_STOP   = 3       # scans taken at each stop
LIDAR_WARMUP_SCANS     = 3       # discard this many scans at startup (partials)
LIDAR_MIN_SCAN_POINTS  = 100     # min points to consider a scan complete


class LidarScanner:
    """A few full scans at a time, for a robot that stands still.

    Blocking pattern: iter_scans is opened only when a measurement is
    needed, a few warm-up scans are discarded (they arrive partial as the
    motor accelerates and the generator syncs to rotation boundaries), then
    the scans asked for are collected, and the LiDAR is stopped so the
    serial buffer doesn't fill up while the robot moves.

    The scans are robot-oriented (0° = robot forward): yaw_deg, the
    profile's lidar_yaw_deg, is removed from every scan. The calibration
    that measures that very angle passes 0 (raw scans)."""

    def __init__(self, port: str, yaw_deg: float) -> None:
        from rplidar import RPLidar      # here: the analysis also runs on the Mac
        print(f"Starting LiDAR on {port}...")
        self.lidar = RPLidar(port)
        self.yaw_deg: float = yaw_deg
        # A previous LiDAR user (lidarTest2, PersonFollower) may have left the
        # sensor mid-scan with garbage in the serial buffer. Reset from a
        # known state before we start measuring.
        self._reset_lidar()

    def _reset_lidar(self) -> None:
        """Bring the LiDAR to a clean idle state: stop scanning, flush buffer,
        reset core. Called on init and after checksum/checkbit errors."""
        try: self.lidar.stop()
        except Exception: pass
        try: self.lidar.stop_motor()
        except Exception: pass
        try: self.lidar.clean_input()
        except Exception: pass
        try: self.lidar.reset()   # reset() ends with a 2s sleep + clean_input
        except Exception: pass

    def _collect_scans_once(self, n: int) -> list:
        """One attempt: open iter_scans, discard warm-ups, collect n scans,
        stop. Any exception propagates to the caller for retry."""
        it = self.lidar.iter_scans(scan_type='normal',
                                    max_buf_meas=6000,
                                    min_len=LIDAR_MIN_SCAN_POINTS)
        try:
            for _ in range(LIDAR_WARMUP_SCANS):
                next(it)
            return [rotate_scan(next(it), self.yaw_deg) for _ in range(n)]
        finally:
            try: self.lidar.stop()
            except Exception: pass

    def collect_scans(self, n: int = LIDAR_SCANS_PER_STOP, retries: int = 2) -> list:
        """Collect n scans with checksum-retry. Serial-sync errors ('Check bit
        not equal to 1', 'Invalid checksum') recover with a full reset."""
        for attempt in range(retries + 1):
            try:
                return self._collect_scans_once(n)
            except Exception as e:
                if attempt == retries:
                    raise
                print(f"  LiDAR read error ({e}) — resetting and retrying "
                      f"[{attempt+1}/{retries}]")
                self._reset_lidar()
        return []

    def close(self) -> None:
        try: self.lidar.stop()
        except Exception: pass
        try: self.lidar.stop_motor()
        except Exception: pass
        try: self.lidar.disconnect()
        except Exception: pass
