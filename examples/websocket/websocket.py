import asyncio
import websockets
import json
from robot_controller import RobotArmController

# Store the latest joint angles
# current_joint_angles = {
#     "Rotation": 0.04,
#     "Pitch": -0.22,
#     "Elbow": -0.17,
#     "Wrist_Pitch": -0.32,
#     "Wrist_Roll": 0.04
#     "Jaw": 0.8,
# }

current_joint_angles = {
    "Rotation": 0.04,
    "Pitch": -0.22,
    "Elbow": 1.54,
    "Wrist_Pitch": -1.0,
    "Wrist_Roll": 0.04,
    "Jaw": 0.8,
}

robot_arm = RobotArmController()

robot_arm.connect()

robot_arm.unlock_all_motors()

async def handle_joint_angles(websocket):  # Removed 'path' parameter
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

                    current_joint_angles["Rotation"] = float(data["Rotation"])
                    current_joint_angles["Pitch"] = float(data["Pitch"])
                    current_joint_angles["Elbow"] = float(data["Elbow"])
                    current_joint_angles["Wrist_Pitch"] = float(data["Wrist_Pitch"])
                    current_joint_angles["Wrist_Roll"] = float(data["Wrist_Roll"])
                    current_joint_angles["Jaw"] = float(data["Jaw"])

                    if check_joint_angles():
                        # Send confirmation back to client
                        response = {
                            "status": "success",
                            "current_angles": current_joint_angles
                        }
                        await websocket.send(json.dumps(response))

                        robot_arm.move_motors_radians({
                            1: current_joint_angles["Rotation"],
                            2: current_joint_angles["Pitch"],
                            3: current_joint_angles["Elbow"],
                            4: current_joint_angles["Wrist_Pitch"],
                            5: current_joint_angles["Wrist_Roll"],
                            6: current_joint_angles["Jaw"]
                        })

                        print(f"Received joint angles: {current_joint_angles}")

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
                
    except websockets.exceptions.ConnectionClosed:
        print("Client connection closed")


def check_joint_angles():
    # Check if any joint angle is outside the valid range
    
    # Check if Rotation is outside the valid range
    if current_joint_angles["Rotation"] < -1.4 or current_joint_angles["Rotation"] > 1.4:
        print("Rotation is outside the valid range")
        return False

    # Check if Pitch is outside the valid range
    if current_joint_angles["Pitch"] < -0.25 or current_joint_angles["Pitch"] > 3.0:
        print("Pitch is outside the valid range")
        return False

    # Check if Elbow is outside the valid range
    if current_joint_angles["Elbow"] < -1.58 or current_joint_angles["Elbow"] > 1.58:
        print("Elbow is outside the valid range")
        print("Elbow Angle: " + str(current_joint_angles["Elbow"]))
        return False

    # Check if Wrist_Pitch is outside the valid range
    if current_joint_angles["Wrist_Pitch"] < -1.58 or current_joint_angles["Wrist_Pitch"] > 1.58:
        print("Wrist_Pitch is outside the valid range")
        return False

    # Check if Wrist_Roll is outside the valid range
    if current_joint_angles["Wrist_Roll"] < -3.0 or current_joint_angles["Wrist_Roll"] > 3.0:
        print("Wrist_Roll is outside the valid range")
        return False

    # Check if Jaw is outside the valid range
    if current_joint_angles["Jaw"] < -0.25 or current_joint_angles["Jaw"] > 1.0:
        print("Jaw is outside the valid range")
        return False


    return True

async def start_server():
    async with websockets.serve(handle_joint_angles, "localhost", 8765):
        print("WebSocket server started on ws://localhost:8765")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(start_server())
