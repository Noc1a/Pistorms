"""The RPLidar, read in a process of its own.

rplidar decodes every measure in Python, 2000 per second. In the robot's
process, the camera, the SLAM and the web server hold the Python interpreter
(one thread at a time): measured 2026-11-07 (PersonFollower.py --record),
the reading fell ~1300 bytes behind on average, so the scans were stamped
~140 ms late, and the serial port buffer (~4 KB) overflowed: 27 corrupted
streams ("Check bit not equal to 1", "New scan flags mismatch") and gaps of
up to 3.7 s. A process has its own interpreter: nothing else slows it.

This module imports only rplidar: the process starts with 'spawn' (safe in
a program that already runs threads) and must not load the camera.
"""
import multiprocessing
import queue
import time

# Scans waiting for the robot's process; if it lags behind, the oldest go
SCANS_QUEUE_SIZE = 4
MAX_BACKOFF_SECS = 15.0


def read_lidar(port_name: str, scans: multiprocessing.Queue, stop: multiprocessing.Event) -> None:
    """Process: read full scans until stop is set, and put each one in
    scans as (time.monotonic() at its end, [(quality, angle_deg_cw, mm), ...],
    serial bytes still waiting). time.monotonic() is the same clock in every
    process of the Pi."""
    from rplidar import RPLidar

    lidar = RPLidar(port_name)
    backoff = 1.0
    try:
        while not stop.is_set():
            try:
                # max_buf_meas: a larger buffer tolerates brief stalls without warnings
                for scan in lidar.iter_scans(max_buf_meas=6000):
                    backoff = 1.0
                    if stop.is_set():
                        break
                    _put_dropping_oldest(scans, (time.monotonic(), scan, _bytes_waiting(lidar)))
            except Exception as e:
                if stop.is_set():
                    break
                print(f"Lidar error: {e!r} — reconnecting in {backoff:.1f}s")
                time.sleep(backoff)
                backoff = min(backoff * 2, MAX_BACKOFF_SECS)
                try:
                    lidar = _reconnect(lidar, port_name)
                except Exception as reconnect_error:
                    print(f"Lidar reconnect failed: {reconnect_error!r}")
    except KeyboardInterrupt:
        pass        # Ctrl+C reaches every process of the program: just stop
    finally:
        # stop() only ends the scan: stop_motor() stops the motor, which
        # otherwise keeps turning (the lidar-pause service keeps the port open)
        for cleanup in (lidar.stop, lidar.stop_motor, lidar.disconnect):
            try:
                cleanup()
            except Exception:
                pass


def _put_dropping_oldest(scans: multiprocessing.Queue, item: tuple) -> None:
    try:
        scans.put_nowait(item)
    except queue.Full:
        try:
            scans.get_nowait()
        except queue.Empty:
            pass
        scans.put_nowait(item)


def _bytes_waiting(lidar) -> int:
    """Bytes still in the serial port when a scan ends: over ~1500 (one
    LiDAR turn: ~2000 measures/s × 5 bytes / 6.4 turns/s), the reading is
    late, and so is the scan's time stamp."""
    try:
        return lidar._serial.in_waiting
    except Exception:
        return -1


def _reconnect(lidar, port_name: str):
    """Reset the LiDAR cleanly — used after a corrupted stream, or a stale
    session whose get_health() returns a 3-tuple the library can't unpack."""
    from rplidar import RPLidar

    for cleanup in (lidar.stop, lidar.disconnect):
        try:
            cleanup()
        except Exception:
            pass
    time.sleep(1.0)
    return RPLidar(port_name)
