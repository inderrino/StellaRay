"""
StellaRay — MAVLink Interface
Handles low-level communication between Raspberry Pi 4 and Pixhawk (ArduPilot).

Connection options:
    Serial : /dev/ttyAMA0 or /dev/serial0 (RPi GPIO UART)
    USB    : /dev/ttyACM0
    UDP    : udp:127.0.0.1:14550 (SITL / Mission Planner passthrough)

Author : Inderpal Singh
Project: StellaRay — Autonomous Quadcopter Platform
Status : Active Development
"""

from pymavlink import mavutil
import time
import threading


class MAVLinkInterface:
    def __init__(self, connection_string="/dev/ttyAMA0", baud=57600):
        print("Connecting to Pixhawk...")
        self.master = mavutil.mavlink_connection(connection_string, baud=baud)

        # Wait for first heartbeat
        self.master.wait_heartbeat()
        print(f"Connected — System: {self.master.target_system}, Component: {self.master.target_component}")

        self._velocity_thread = None
        self._velocity_active = False

    # -----------------------------
    # ARM / DISARM
    # -----------------------------
    def is_armed(self):
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return False

    def arm(self):
        if self.is_armed():
            print("Already armed.")
            return
        print("Arming...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("Armed!")

    def disarm(self):
        if not self.is_armed():
            print("Already disarmed.")
            return
        self.stop_velocity()
        print("Disarming...")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        print("Disarmed!")

    # -----------------------------
    # MODE
    # -----------------------------
    def set_mode(self, mode, timeout=5):
        if mode not in self.master.mode_mapping():
            print(f"Mode '{mode}' not supported.")
            return False

        mode_id = self.master.mode_mapping()[mode]
        self.master.set_mode(mode_id)

        # Confirm mode switch via HEARTBEAT
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                current_mode = mavutil.mode_string_v10(msg)
                if current_mode == mode:
                    print(f"Mode confirmed: {mode}")
                    return True
        print(f"Warning: Mode switch to '{mode}' not confirmed within {timeout}s.")
        return False

    # -----------------------------
    # TELEMETRY
    # -----------------------------
    def get_heartbeat(self, timeout=5):
        return self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)

    def get_gps(self, timeout=5):
        msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=timeout)
        if msg:
            return {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "alt": msg.alt / 1000,
                "fix_type": msg.fix_type,
                "satellites_visible": msg.satellites_visible
            }
        print("Warning: GPS message timed out.")
        return None

    def get_attitude(self, timeout=5):
        msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=timeout)
        if msg:
            return {
                "roll":  round(msg.roll, 4),
                "pitch": round(msg.pitch, 4),
                "yaw":   round(msg.yaw, 4)
            }
        print("Warning: ATTITUDE message timed out.")
        return None

    def get_battery(self, timeout=5):
        msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=timeout)
        if msg:
            return {
                "voltage_V":  msg.voltage_battery / 1000,
                "current_A":  msg.current_battery / 100,
                "remaining_%": msg.battery_remaining
            }
        print("Warning: SYS_STATUS message timed out.")
        return None

    # -----------------------------
    # TAKEOFF
    # -----------------------------
    def takeoff(self, altitude=5):
        assert altitude > 0, "Altitude must be positive."
        print(f"Initiating takeoff to {altitude}m...")

        if not self.set_mode("GUIDED"):
            print("Failed to enter GUIDED mode. Aborting takeoff.")
            return

        time.sleep(1)
        self.arm()

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0,
            altitude
        )
        print(f"Takeoff command sent. Target altitude: {altitude}m")

    # -----------------------------
    # LAND / RTL
    # -----------------------------
    def land(self):
        print("Landing...")
        self.stop_velocity()
        self.set_mode("LAND")

    def return_to_launch(self):
        print("Returning to launch...")
        self.stop_velocity()
        self.set_mode("RTL")

    # -----------------------------
    # VELOCITY CONTROL (LOCAL NED)
    # Velocity commands expire ~1s on Pixhawk.
    # Use send_velocity_sustained() for actual flight.
    # -----------------------------
    def _send_velocity_once(self, vx, vy, vz):
        """Single velocity command (NED frame, m/s)."""
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # velocity only
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )

    def send_velocity_sustained(self, vx, vy, vz, duration=None, interval=0.1):
        """
        Sends velocity commands in a background thread at `interval` seconds.
        Runs indefinitely if duration=None, until stop_velocity() is called.

        Args:
            vx, vy, vz : velocity in m/s (NED frame)
            duration   : seconds to run (None = run until stopped)
            interval   : command repeat interval in seconds (default 0.1s)
        """
        self.stop_velocity()  # stop any existing thread first

        self._velocity_active = True

        def _loop():
            start = time.time()
            while self._velocity_active:
                if duration and (time.time() - start) >= duration:
                    break
                self._send_velocity_once(vx, vy, vz)
                time.sleep(interval)

        self._velocity_thread = threading.Thread(target=_loop, daemon=True)
        self._velocity_thread.start()
        print(f"Velocity command started: vx={vx}, vy={vy}, vz={vz}")

    def stop_velocity(self):
        """Stops sustained velocity command."""
        self._velocity_active = False
        if self._velocity_thread and self._velocity_thread.is_alive():
            self._velocity_thread.join(timeout=1)
        self._send_velocity_once(0, 0, 0)
        print("Velocity stopped.")

    # -----------------------------
    # PARAM READ
    # -----------------------------
    def get_param(self, param_name):
        assert len(param_name) <= 16, "MAVLink param IDs are max 16 characters."

        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode('utf-8'),
            -1
        )

        deadline = time.time() + 5
        while time.time() < deadline:
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg and msg.param_id.decode('utf-8').strip('\x00') == param_name:
                return msg.param_value

        print(f"Warning: Param '{param_name}' not received.")
        return None


# -----------------------------
# TEST SCRIPT
# -----------------------------
if __name__ == "__main__":
    drone = MAVLinkInterface(connection_string="/dev/ttyACM0", baud=57600)

    print("Heartbeat:", drone.get_heartbeat())

    gps = drone.get_gps()
    print("GPS:", gps)

    attitude = drone.get_attitude()
    print("Attitude:", attitude)

    battery = drone.get_battery()
    print("Battery:", battery)

    # --- Uncomment for actual flight test ---
    # drone.set_mode("GUIDED")
    # drone.arm()
    # drone.takeoff(5)
    # time.sleep(5)
    # drone.send_velocity_sustained(vx=0.5, vy=0, vz=0, duration=3)  # move forward 3s
    # time.sleep(4)
    # drone.land()
