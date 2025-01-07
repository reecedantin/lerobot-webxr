WebXR LeRobot Control - SO-ARM100

URDFLoader from - https://github.com/gkjohnson/urdf-loaders

SO-ARM100 URDF Model from - https://github.com/TheRobotStudio/SO-ARM100

To start the web server, first npm install and then npm run dev

# For Wireless:
Open the browser on your headset and navigate to your computer's IP over https and port 8081. Use https://IP_ADDRESS:8081

# For Wired:
Plug your Meta Quest headset into your computer with a long usb cable, can be USB 2.0 speeds. Make sure developer mode is enabled and paired to computer.

Navigate to chrome://inspect/#devices and port forward 8081.

On the headset, navigate to https://localhost:8081. Click on "START XR" and you should see the robot 3D model.


# Controls:

Press the X button to enable movement of the robot, then use the thumbsticks to position the virtual robot to the actual robot. Left controller will move up and down in Z axis and rotate around Z, right controller will move in X and Y.

Squeeze the side button on the right controller to move the dot. The robot will position the tip at the dot.

Press the A button on the controller to enable remote control, the robot will now mimic the movement of the virtual robot