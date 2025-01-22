import enum
import logging
import math
import time
import traceback
from copy import deepcopy

import numpy as np
import tqdm
import websockets
import threading
import asyncio
import json

from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError
from lerobot.common.utils.utils import capture_timestamp_utc


class WebSocketController:
    """
    The WebSocketController is meant to act as a leader arm for a robot, but through a websocket - to be used with a
    digital twin on a website. Specifically made to work with a VR device with WebXR.
    """

    def __init__(
        self,
        port: str,
        motors: dict[str, tuple[int, str]],
        extra_model_control_table: dict[str, list[tuple]] | None = None,
        extra_model_resolution: dict[str, int] | None = None,
        mock=False,
    ):
        self.port = port
        self.motors = motors
        self.mock = mock

        self.port_handler = None
        self.packet_handler = None
        self.calibration = None
        self.is_connected = False
        self.group_readers = {}
        self.group_writers = {}
        self.logs = {}

        self.track_positions = {}
        self.joint_states = {
            1: 0,
            2:-180,
            3: 145,
            4: 90,
            5: 0,
            6: 0,
        }
        self.websocket_thread = None

    def run_websocket_server(self):
        async def start_server():
            try:
                async with websockets.serve(self.handle_joint_angles, "localhost", self.port):
                    print(f"WebSocket server started on ws://localhost:{self.port}")
                    self.loop = asyncio.get_event_loop()
                    await asyncio.Future()  # Run forever
            except Exception as e:
                print(f"WebSocket server error: {str(e)}")
            finally:
                print("WebSocket server stopped")

        # Create new event loop for this thread
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(start_server())


    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                f"WebSocketController({self.port}) is already running. Do not call `motors_bus.connect()` twice."
            )

        # Start the WebSocket server in a separate thread
        self.websocket_thread = threading.Thread(
            target=self.run_websocket_server,
            daemon=True  # Thread will exit when main program exits
        )
        self.websocket_thread.start()

        # Allow to read and write
        self.is_connected = True

    async def handle_joint_angles(self, websocket):
        try:
            async for message in websocket:
                try:
                    # Parse the incoming JSON message
                    data = json.loads(message)
                    
                    # Update joint angles if valid data is received
                    if isinstance(data, dict):
                        # for joint, angle in data.items():
                        #     if joint in current_joint_angles and isinstance(angle, (int, float)):
                        #         current_joint_angles[joint] = float(angle)


                        joint_angles = {}
                        joint_angles["Rotation"] = float(data["Rotation"])
                        joint_angles["Pitch"] = float(data["Pitch"])
                        joint_angles["Elbow"] = float(data["Elbow"])
                        joint_angles["Wrist_Pitch"] = float(data["Wrist_Pitch"])
                        joint_angles["Wrist_Roll"] = float(data["Wrist_Roll"])
                        joint_angles["Jaw"] = float(data["Jaw"])

                        if self.check_joint_angles(joint_angles):
                            # Send confirmation back to client

                            self.joint_states = {
                                1: self.radians_to_degrees(joint_angles["Rotation"]),
                                2: self.radians_to_degrees(joint_angles["Pitch"]),
                                3: self.radians_to_degrees(joint_angles["Elbow"]),
                                4: self.radians_to_degrees(joint_angles["Wrist_Pitch"]),
                                5: self.radians_to_degrees(joint_angles["Wrist_Roll"]),
                                6: self.radians_to_degrees(joint_angles["Jaw"])
                            }

                            # print(f"Received joint angles: {joint_angles}")

                            total_off_target = 0
                            # TODO: Implement force feedback somehow
                            # for result in results:
                            #     total_off_target += results[result]

                            response = {
                                "status": "success",
                                "current_angles": joint_angles,
                                "off_target": total_off_target
                            }
                            await websocket.send(json.dumps(response))

                        else:
                            error_response = {
                                "status": "error",
                                "message": "Invalid joint angles"
                            }
                            await websocket.send(json.dumps(error_response))
                    
                except json.JSONDecodeError:
                    error_response = {
                        "status": "error",
                        "message": "Invalid JSON format"
                    }
                    await websocket.send(json.dumps(error_response))

                except Exception as e:
                    print(f"Error processing message: {str(e)}")
                    error_response = {
                        "status": "error",
                        "message": str(e)
                    }
                    await websocket.send(json.dumps(error_response))
                    
        except websockets.exceptions.ConnectionClosed:
            print("Client connection closed")
            self.is_connected = False

        except Exception as e:
            print(f"WebSocket error: {str(e)}")
            traceback.print_exc()
            self.is_connected = False


    def check_joint_angles(self, joint_angles):
        # Check if any joint angle is outside the valid range
        
        # Check if Rotation is outside the valid range
        if joint_angles["Rotation"] < -1.4 or joint_angles["Rotation"] > 1.4:
            print("Rotation is outside the valid range")
            return False

        # Check if Pitch is outside the valid range
        # if joint_angles["Pitch"] < -0.25 or joint_angles["Pitch"] > 3.0:
        #     print("Pitch is outside the valid range")
        #     return False

        # Check if Elbow is outside the valid range
        # if joint_angles["Elbow"] < -1.58 or joint_angles["Elbow"] > 1.58:
        #     print("Elbow is outside the valid range")
        #     return False

        # Check if Wrist_Pitch is outside the valid range
        if joint_angles["Wrist_Pitch"] < -1.58 or joint_angles["Wrist_Pitch"] > 1.58:
            print("Wrist_Pitch is outside the valid range")
            return False

        # Check if Wrist_Roll is outside the valid range
        if joint_angles["Wrist_Roll"] < -3.0 or joint_angles["Wrist_Roll"] > 3.0:
            print("Wrist_Roll is outside the valid range")
            return False

        # Check if Jaw is outside the valid range
        if joint_angles["Jaw"] < -0.25 or joint_angles["Jaw"] > 1.0:
            print("Jaw is outside the valid range")
            return False


        return True

    def reconnect(self):
        self.is_connected = True

    def are_motors_configured(self):
        # Only check the motor indices and not baudrate, since if the motor baudrates are incorrect,
        # a ConnectionError will be raised anyway.

        return True
        # try:
        #     return (self.motor_indices == self.read("ID")).all()
        # except ConnectionError as e:
        #     print(e)
        #     return False

    def find_motor_indices(self, possible_ids=None, num_retry=2):
        # TODO: do we need this?
        return None

    @property
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [idx for idx, _ in self.motors.values()]

    def radians_to_steps(self, radians: float) -> int:
        """Convert radians to motor steps
        0 degrees = 2048 steps (center position)
        360 degrees = 4095 steps
        -360 degrees = 0 steps
        """

        degrees = math.degrees(radians)
        # Convert degrees to steps
        steps_per_degree = 4095 / 360
        steps = int(2048 + (degrees * steps_per_degree))
        
        # Ensure steps stay within valid range
        return max(0, min(4095, steps))

    def steps_to_degrees(self, steps: int) -> float:
        """Convert motor steps to degrees"""
        degrees_per_step = 360 / 4095
        return (steps - 2048) * degrees_per_step

    def radians_to_degrees(self, radians: float) -> float:
        """Convert radians to degrees"""
        return math.degrees(radians)

    def set_calibration(self, calibration: dict[str, list]):
        self.calibration = calibration


    def apply_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):

        # Convert from unsigned int32 original range [0, 2**32] to signed float32 range
        values = values.astype(np.float32)

        return values

    def revert_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        """Inverse of `apply_calibration`."""
        if motor_names is None:
            motor_names = self.motor_names

        # not implemented

        values = np.round(values).astype(np.int32)
        return values


    def read(self, data_name, motor_names: str | list[str] | None = None):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"WebSocketController({self.port}) is not connected. You need to run `motors_bus.connect()`."
            )

        if motor_names is None:
            motor_names = self.motor_names

        if isinstance(motor_names, str):
            motor_names = [motor_names]

        motor_ids = []
        models = []
        for name in motor_names:
            motor_idx, model = self.motors[name]
            motor_ids.append(motor_idx)
            models.append(model)

        values = []
        for idx in motor_ids:
            # value = self.radians_to_steps(self.joint_states[idx])
            value = self.joint_states[idx]
            values.append(value)

        values = np.array(values)

        return values

    def write_with_motor_ids(self, motor_models, motor_ids, data_name, values, num_retry=0):

        # Not writing anything because there's no control

        return

    def write(self, data_name, values: int | float | np.ndarray, motor_names: str | list[str] | None = None):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"WebSocketController({self.port}) is not connected. You need to run `motors_bus.connect()`."
            )

        
        # Not writing anything because there's no control

        return

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"WebSocketController({self.port}) is not connected. Try running `motors_bus.connect()` first."
            )

        if self.port_handler is not None:
            self.port_handler.closePort()
            self.port_handler = None

        self.packet_handler = None
        self.group_readers = {}
        self.group_writers = {}

        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.websocket_thread:
            self.websocket_thread.join()

        self.is_connected = False

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()
