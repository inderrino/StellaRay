"""
StellaRay — Mission Control
High-level autonomy layer. Ties together MAVLink interface, vision pipeline,
and object detection into a unified mission execution system.

Mission modes:
    SURVEY      : Fly a predefined waypoint grid, capture frames at each point
    SEARCH      : Loiter and scan for objects, hover on detection
    PATROL      : Repeat a closed waypoint loop continuously
    RTL         : Abort mission and return to launch

State machine:
    IDLE → PREFLIGHT → TAKEOFF → MISSION → LAND → IDLE

Author : Inderpal Singh
Project: StellaRay — Autonomous Quadcopter Platform
Status : Active Development
"""

import time
import threading
import logging
import os
from enum import Enum, auto
from datetime import datetime

from mavlink_interface import MAVLinkInterface
from vision_pipeline import VisionPipeline
from object_detection import build_detection_fn


# -----------------------------
# LOGGING SETUP
# -----------------------------
os.makedirs("logs", exist_ok=True)
log_file = f"logs/mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)
log = logging.getLogger("MissionControl")


# -----------------------------
# ENUMS
# -----------------------------
class MissionState(Enum):
    IDLE       = auto()
    PREFLIGHT  = auto()
    TAKEOFF    = auto()
    MISSION    = auto()
    HOLD       = auto()
    LAND       = auto()
    RTL        = auto()
    ABORT      = auto()


class MissionMode(Enum):
    SURVEY  = "survey"
    SEARCH  = "search"
    PATROL  = "patrol"


# -----------------------------
# WAYPOINT
# -----------------------------
class Waypoint:
    def __init__(self, lat, lon, alt, hold_time=2, label="WP"):
        """
        Args:
            lat, lon  : decimal degrees
            alt       : altitude in meters (relative to launch)
            hold_time : seconds to loiter at waypoint before moving on
            label     : human-readable name for logs
        """
        self.lat       = lat
        self.lon       = lon
        self.alt       = alt
        self.hold_time = hold_time
        self.label     = label

    def __repr__(self):
        return f"Waypoint({self.label} | {self.lat:.6f}, {self.lon:.6f}, {self.alt}m)"


# -----------------------------
# PREFLIGHT CHECKS
# -----------------------------
class PreflightChecker:
    def __init__(self, drone: MAVLinkInterface):
        self.drone = drone

    def run(self):
        log.info("Running preflight checks...")
        results = {}

        # GPS fix
        gps = self.drone.get_gps(timeout=5)
        results['gps'] = gps is not None and gps.get('fix_type', 0) >= 3
        log.info(f"  GPS fix      : {'OK' if results['gps'] else 'FAIL'} {gps}")

        # Battery
        battery = self.drone.get_battery(timeout=5)
        results['battery'] = battery is not None and battery.get('remaining_%', 0) >= 20
        log.info(f"  Battery      : {'OK' if results['battery'] else 'FAIL'} {battery}")

        # Heartbeat
        hb = self.drone.get_heartbeat(timeout=5)
        results['heartbeat'] = hb is not None
        log.info(f"  Heartbeat    : {'OK' if results['heartbeat'] else 'FAIL'}")

        # Armed state (should NOT be armed at preflight)
        results['not_armed'] = not self.drone.is_armed()
        log.info(f"  Disarmed     : {'OK' if results['not_armed'] else 'FAIL'}")

        passed = all(results.values())
        log.info(f"Preflight {'PASSED' if passed else 'FAILED'}: {results}")
        return passed


# -----------------------------
# MISSION CONTROLLER
# -----------------------------
class MissionController:
    def __init__(
        self,
        connection_string="/dev/ttyAMA0",
        baud=57600,
        takeoff_alt=10,
        camera_src=0,
        model_path="models/detect.tflite",
        label_path="models/labelmap.txt",
        conf_threshold=0.5,
        display=False
    ):
        """
        Args:
            connection_string : MAVLink serial/UDP connection
            baud              : serial baud rate
            takeoff_alt       : default takeoff altitude in meters
            camera_src        : camera index or device path
            model_path        : TFLite model path
            label_path        : label map path
            conf_threshold    : detection confidence threshold
            display           : show OpenCV window (False on headless RPi)
        """
        log.info("Initialising StellaRay Mission Controller...")

        self.takeoff_alt  = takeoff_alt
        self.state        = MissionState.IDLE
        self._abort_flag  = threading.Event()
        self._detections  = []
        self._det_lock    = threading.Lock()

        # MAVLink
        self.drone = MAVLinkInterface(connection_string, baud)

        # Vision + Detection
        self.detect_fn = build_detection_fn(
            model_path=model_path,
            label_path=label_path,
            conf_threshold=conf_threshold,
            mavlink=self.drone
        )
        self.vision = VisionPipeline(
            camera_src=camera_src,
            mavlink=self.drone,
            display=display
        )

        # Start vision in background thread
        self._vision_thread = threading.Thread(
            target=self.vision.run,
            kwargs={"detection_fn": self._detection_wrapper},
            daemon=True
        )
        self._vision_thread.start()
        log.info("Vision pipeline running in background.")

    def _detection_wrapper(self, frame):
        """Wraps detect_fn to cache latest detections for mission logic."""
        results = self.detect_fn(frame)
        with self._det_lock:
            self._detections = results
        return results

    def _get_latest_detections(self):
        with self._det_lock:
            return list(self._detections)

    # -----------------------------
    # STATE MACHINE
    # -----------------------------
    def _set_state(self, new_state: MissionState):
        log.info(f"State: {self.state.name} → {new_state.name}")
        self.state = new_state

    def _preflight(self):
        self._set_state(MissionState.PREFLIGHT)
        checker = PreflightChecker(self.drone)
        if not checker.run():
            log.error("Preflight failed. Aborting.")
            self._set_state(MissionState.ABORT)
            return False
        return True

    def _takeoff(self):
        self._set_state(MissionState.TAKEOFF)
        self.drone.takeoff(self.takeoff_alt)
        log.info(f"Climbing to {self.takeoff_alt}m...")
        time.sleep(8)  # allow time to reach altitude
        log.info("Takeoff complete.")

    def _land(self):
        self._set_state(MissionState.LAND)
        self.drone.land()
        log.info("Landing...")
        time.sleep(10)
        log.info("Landed.")
        self._set_state(MissionState.IDLE)

    def _rtl(self):
        self._set_state(MissionState.RTL)
        self.drone.return_to_launch()
        log.info("RTL initiated.")

    def abort(self):
        """Emergency abort — triggers RTL immediately."""
        log.warning("ABORT triggered.")
        self._abort_flag.set()
        self._rtl()
        self._set_state(MissionState.ABORT)

    def _check_abort(self):
        """Call this in any mission loop to catch abort signal."""
        if self._abort_flag.is_set():
            log.warning("Abort flag detected. Exiting mission.")
            self._rtl()
            return True
        return False

    # -----------------------------
    # WAYPOINT NAVIGATION
    # Send drone to a GPS waypoint via MAVLink SET_POSITION_TARGET_GLOBAL_INT
    # -----------------------------
    def _goto_waypoint(self, wp: Waypoint, arrival_radius=2.0):
        """
        Commands drone to fly to a GPS waypoint and hold.

        Args:
            wp             : Waypoint instance
            arrival_radius : distance in meters to consider waypoint reached
        """
        from pymavlink import mavutil

        log.info(f"Flying to {wp}")

        self.drone.master.mav.set_position_target_global_int_send(
            0,
            self.drone.master.target_system,
            self.drone.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # position only
            int(wp.lat * 1e7),
            int(wp.lon * 1e7),
            wp.alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

        # Poll GPS until within arrival radius
        deadline = time.time() + 60  # 60s max per waypoint
        while time.time() < deadline:
            if self._check_abort():
                return False

            gps = self.drone.get_gps(timeout=1)
            if gps:
                dist = self._haversine(gps['lat'], gps['lon'], wp.lat, wp.lon)
                log.info(f"  Distance to {wp.label}: {dist:.1f}m")
                if dist <= arrival_radius:
                    log.info(f"Reached {wp.label}. Holding for {wp.hold_time}s.")
                    time.sleep(wp.hold_time)
                    return True
            time.sleep(0.5)

        log.warning(f"Timeout reaching {wp.label}.")
        return False

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2):
        """Approximate ground distance in meters between two GPS coords."""
        import math
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # -----------------------------
    # MISSION MODES
    # -----------------------------
    def run_survey(self, waypoints: list):
        """
        Flies through a list of waypoints in order.
        Captures and logs detections at each point.

        Args:
            waypoints : list of Waypoint objects
        """
        log.info(f"Starting SURVEY mission — {len(waypoints)} waypoints.")
        self._set_state(MissionState.MISSION)

        for i, wp in enumerate(waypoints):
            log.info(f"Waypoint {i + 1}/{len(waypoints)}: {wp}")
            if self._check_abort():
                return

            reached = self._goto_waypoint(wp)
            if not reached:
                continue

            # Check for detections at this waypoint
            detections = self._get_latest_detections()
            if detections:
                log.info(f"Detections at {wp.label}: {[d['label'] for d in detections]}")

        log.info("Survey complete.")

    def run_search(self, center: Waypoint, radius_m=20, scan_duration=60):
        """
        Flies to center point, loiters, and monitors for object detections.
        Moves toward detected object if found.

        Args:
            center      : Waypoint to loiter around
            radius_m    : search radius in meters (unused for now, future expansion)
            scan_duration: seconds to scan before returning
        """
        log.info(f"Starting SEARCH mission around {center}.")
        self._set_state(MissionState.MISSION)

        self._goto_waypoint(center)
        self.drone.set_mode("LOITER")

        deadline = time.time() + scan_duration
        while time.time() < deadline:
            if self._check_abort():
                return

            detections = self._get_latest_detections()
            if detections:
                log.info(f"Object found: {[d['label'] for d in detections]}")
                self._set_state(MissionState.HOLD)
                log.info("Holding position over detected object.")
                time.sleep(10)  # hover and observe
                self._set_state(MissionState.MISSION)

            time.sleep(0.5)

        log.info("Search scan complete.")

    def run_patrol(self, waypoints: list, loops=3):
        """
        Repeatedly flies a closed waypoint loop.

        Args:
            waypoints : list of Waypoint objects forming the patrol route
            loops     : number of times to repeat the route (0 = infinite)
        """
        log.info(f"Starting PATROL mission — {len(waypoints)} points, {loops if loops else '∞'} loops.")
        self._set_state(MissionState.MISSION)

        count = 0
        while loops == 0 or count < loops:
            log.info(f"Patrol loop {count + 1}")
            for wp in waypoints:
                if self._check_abort():
                    return
                self._goto_waypoint(wp)
            count += 1

        log.info("Patrol complete.")

    # -----------------------------
    # FULL MISSION RUNNER
    # -----------------------------
    def execute(self, mode: MissionMode, waypoints: list, **kwargs):
        """
        Full autonomous mission with preflight, takeoff, mission, and landing.

        Args:
            mode      : MissionMode enum value
            waypoints : list of Waypoint objects
            **kwargs  : extra args forwarded to the mission mode function
        """
        try:
            if not self._preflight():
                return

            self._takeoff()

            if mode == MissionMode.SURVEY:
                self.run_survey(waypoints)
            elif mode == MissionMode.SEARCH:
                self.run_search(waypoints[0], **kwargs)
            elif mode == MissionMode.PATROL:
                self.run_patrol(waypoints, **kwargs)

            self._land()

        except KeyboardInterrupt:
            log.warning("KeyboardInterrupt — initiating RTL.")
            self._rtl()

        except Exception as e:
            log.error(f"Unhandled exception: {e}. Initiating RTL.")
            self._rtl()
            raise

        finally:
            self.vision.stop()
            log.info("Mission controller shut down.")


# -----------------------------
# TEST SCRIPT
# -----------------------------
if __name__ == "__main__":

    # --- Define waypoints (replace with actual GPS coords) ---
    waypoints = [
        Waypoint(lat=28.461200, lon=76.578400, alt=15, hold_time=3, label="WP1"),
        Waypoint(lat=28.461500, lon=76.578700, alt=15, hold_time=3, label="WP2"),
        Waypoint(lat=28.461800, lon=76.578400, alt=15, hold_time=3, label="WP3"),
        Waypoint(lat=28.461500, lon=76.578100, alt=15, hold_time=3, label="WP4"),
    ]

    mc = MissionController(
        connection_string="/dev/ttyACM0",
        baud=57600,
        takeoff_alt=15,
        camera_src=0,
        model_path="models/detect.tflite",
        label_path="models/labelmap.txt",
        conf_threshold=0.5,
        display=False
    )

    mc.execute(mode=MissionMode.SURVEY, waypoints=waypoints)
