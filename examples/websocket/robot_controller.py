from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
import time
from typing import Dict, Union, List
import math

class RobotArmController:
    def __init__(self, port: str = "/dev/tty.usbmodem58750063001"):
        # Initialize with all 6 possible motors
        self.motor_bus = FeetechMotorsBus(
            port=port,
            motors={
                "motor1": (1, "sts3215"),
                "motor2": (2, "sts3215"),
                "motor3": (3, "sts3215"),
                "motor4": (4, "sts3215"),
                "motor5": (5, "sts3215"),
                "motor6": (6, "sts3215")
            }
        )
        self.is_connected = False

    def connect(self):
        """Connect to the motor bus"""
        try:
            self.motor_bus.connect()
            self.is_connected = True
            print(f"Connected on port {self.motor_bus.port}")
        except Exception as e:
            print(f"Connection failed: {e}")
            self.is_connected = False

    def disconnect(self):
        """Disconnect from the motor bus"""
        if self.is_connected:
            self.motor_bus.disconnect()
            self.is_connected = False
            print("Disconnected from motor bus")

    def degrees_to_steps(self, degrees: float) -> int:
        """Convert degrees to motor steps
        0 degrees = 2048 steps (center position)
        360 degrees = 4095 steps
        -360 degrees = 0 steps
        """
        # Convert degrees to steps
        steps_per_degree = 4095 / 360
        steps = int(2048 + (degrees * steps_per_degree))
        
        # Ensure steps stay within valid range
        return max(0, min(4095, steps))

    def steps_to_degrees(self, steps: int) -> float:
        """Convert motor steps to degrees"""
        degrees_per_step = 360 / 4095
        return (steps - 2048) * degrees_per_step

    def move_motors_radians(self, angles: Dict[int, float]) -> Dict[int, float]:
        """
        Move specified motors to given angles in radians

        Args:
            angles: Dictionary mapping motor IDs to target angles in radians

        Returns:
            Dictionary of actual angles achieved by each motor
        """
        # Convert radians to degrees
        angles_degrees = {motor_id: math.degrees(angle) for motor_id, angle in angles.items()}

        # Move motors with degrees
        return self.move_motors(angles_degrees)

    def move_motors(self, angles: Dict[int, float]) -> Dict[int, float]:
        """
        Move specified motors to given angles in degrees
        
        Args:
            angles: Dictionary mapping motor IDs to target angles in degrees
                   e.g., {1: 90, 3: -45} moves motor 1 to 90° and motor 3 to -45°
        
        Returns:
            Dictionary of actual angles achieved by each motor
        """
        if not self.is_connected:
            print("Not connected to motor bus")
            return {}

        results = {}
        try:
            # Unlock and move each motor
            for motor_id, angle in angles.items():

                if not 1 <= motor_id <= 6:
                    print(f"Invalid motor ID: {motor_id}")
                    continue

                # Convert angle to steps
                target_steps = self.degrees_to_steps(angle)

                # Unlock motor
                self.motor_bus.write_with_motor_ids(
                    self.motor_bus.motor_models,
                    motor_id,
                    "Lock",
                    0
                )

                # Set Maximum_Acceleration to 254 to speedup acceleration and deceleration of
                # the motors. Note: this configuration is not in the official STS3215 Memory Table
                self.motor_bus.write_with_motor_ids(
                    self.motor_bus.motor_models,
                    motor_id,
                    "Goal_Speed",
                    0
                )

                # Move to position
                self.motor_bus.write_with_motor_ids(
                    self.motor_bus.motor_models,
                    motor_id,
                    "Goal_Position",
                    target_steps
                )

            # # Wait for movements to complete
            # time.sleep(2)

            # Read final positions
            # for motor_id in angles.keys():
            #     actual_steps = self.motor_bus.read_with_motor_ids(
            #         self.motor_bus.motor_models,
            #         motor_id,
            #         "Present_Position",
            #         num_retry=2
            #     )
            #     actual_angle = self.steps_to_degrees(actual_steps)
            #     results[motor_id] = actual_angle

        except Exception as e:
            print(f"Error moving motors: {e}")

        return results

    def unlock_all_motors(self):
        """Unlock all 6 motors"""
        if not self.is_connected:
            print("Not connected to motor bus")
            return

        try:
            for motor_id in range(1, 7):  # Motors 1-6
                self.motor_bus.write_with_motor_ids(
                    self.motor_bus.motor_models,
                    motor_id,
                    "Lock",
                    1  # 1 means unlocked/disabled
                )
            print("All motors unlocked")
        except Exception as e:
            print(f"Error unlocking motors: {e}")