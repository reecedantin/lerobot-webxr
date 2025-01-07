WebSocket LeRobot Control - SO-ARM100

This is meant to be run with the WebXR demo. Moving the robot in the virtual world will match the movements to the real world.

Make sure you are in the lerobot conda env and you set up with pip install -e ".[feetech]"

Then install websockets - pip install websockets.

Make sure your robot is plugged in and the usb is configured in robot_controller.py

Port forward on chrome://inspect/#devices for port 8765.

Then run with python websockets.py