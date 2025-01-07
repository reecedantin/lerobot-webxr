/**
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

import * as THREE from 'three';
import { XR_BUTTONS, XR_AXES } from 'gamepad-wrapper';
import { init } from './init.js';
import { CCDIKSolver } from 'three/addons/animation/CCDIKSolver.js';

import URDFLoader from './urdf/URDFLoader.js';

let robot

function createSkeletonFromURDF(robot) {
    const bones = [];
    let rootBone = new THREE.Bone();
    rootBone.position.set(0, 1, -1)
    bones.push(rootBone);

    // Helper function to log the URDF structure
    function logURDFStructure(node, depth = 0) {
        const prefix = '  '.repeat(depth);
        console.log(`${prefix}${node.name || 'unnamed'} (${node.type || 'no type'})`);
        node.children.forEach(child => logURDFStructure(child, depth + 1));
    }

    // Log the URDF structure to understand the hierarchy
    console.log("URDF Structure:");
    logURDFStructure(robot);

    function processJoint(parent, joint) {
        console.log("Processing joint:", joint.name, joint.type);
        const bone = new THREE.Bone();
        bone.name = joint.name || `bone_${bones.length}`;
        parent.add(bone);

        // Get world position of the joint
        const worldPos = new THREE.Vector3();
        joint.getWorldPosition(worldPos);

        // Convert world position to local space of parent bone
        parent.updateMatrixWorld();
        bone.position.copy(parent.worldToLocal(worldPos));

        bones.push(bone);

        // Process child joints
        joint.children.forEach(child => {
            if (child.isURDFLink) {
                child.children.forEach(grandChild => {
                    if (grandChild.isURDFJoint) {
                        processJoint(bone, grandChild);
                    } else if (child.name == "Moving Jaw") {
                        const jawBone = new THREE.Bone();
                        jawBone.name = child.name || `bone_${bones.length}`;
                        bone.add(jawBone);
                        bones.push(jawBone);

                        // Get the mesh's geometry center
                        let center = new THREE.Vector3();
                        if (child.children[0] && child.children[0].geometry) {
                            child.children[0].geometry.computeBoundingBox();
                            child.children[0].geometry.boundingBox.getCenter(center);
                            console.log("Jaw mesh center:", center);
                        }

                        // Manually set the position relative to the parent bone
                        // Adjust these values based on your URDF model
                        jawBone.position.set(0.03, -0.08, 0); // Example values - adjust these

                        // Log the final position for debugging
                        const worldPos = new THREE.Vector3();
                        jawBone.getWorldPosition(worldPos);
                        console.log("Jaw bone world position:", worldPos);
                    }
                });
            }
        });
    }

    // Find all joints and create bones
    let jointCount = 0;
    robot.traverse(child => {
        if (child.isURDFJoint) {
            jointCount++;
            console.log("Found joint:", child.name, child.type, child.parent.name);
        }
    });
    console.log("Total joints found:", jointCount);

    // Start processing from the robot's joints
    robot.traverse(child => {
        if (child.isURDFJoint && child.parent === robot) {
            processJoint(rootBone, child);
        }
    });

    console.log("Created bones:", bones.length);
    bones.forEach(bone => console.log("Bone:", bone.name));

    // Create the skeleton
    const skeleton = new THREE.Skeleton(bones);

    // Create a helper to visualize the skeleton
    const skeletonHelper = new THREE.SkeletonHelper(rootBone);

    return { skeleton, skeletonHelper };
}


async function setupScene({ scene, camera, renderer, player, controllers }) {
    // Add directional light (like sunlight)
    const directionalLight = new THREE.DirectionalLight(0xFFFFFF, 1.0);
    directionalLight.position.set(-200, 200, 150);
    directionalLight.castShadow = true; // Enable shadow casting
    scene.add(directionalLight);

    const targetGeometry = new THREE.SphereGeometry(0.005);
    const targetMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    const targetMesh = new THREE.Mesh(targetGeometry, targetMaterial);
    // targetMesh.position.set(0, 1.1, -0.75);
    scene.add(targetMesh);

    let ikSolver;

    // Load robot
    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.load('./assets/lerobot/urdf/lerobot.URDF', result => {
        robot = result;
    });

    manager.onLoad = () => {
        robot.rotation.x = Math.PI * 1.5;
        robot.traverse(c => {
            c.castShadow = true;
        });

        robot.updateMatrixWorld(true);

        robot.position.x = 0;
        robot.position.y = 1;
        robot.position.z = -1;
        scene.add(robot);

        // Create skeleton from URDF
        const { skeleton, skeletonHelper } = createSkeletonFromURDF(robot);
        // skeletonHelper.visible = true;
        // scene.add(skeletonHelper);

        const geometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
        const material = new THREE.MeshBasicMaterial({
            color: 0x00ff00,
            wireframe: true,
            visible: false
        });

        // Create the skinned mesh
        const skinnedMesh = new THREE.SkinnedMesh(geometry, material);

        // Bind the skeleton to the mesh
        skinnedMesh.add(skeleton.bones[0]); // Add root bone
        skinnedMesh.bind(skeleton);

        // Position the skinned mesh to match your robot
        skinnedMesh.position.copy(robot.position);
        skinnedMesh.rotation.copy(robot.rotation);
        scene.add(skeleton.bones[0]);

        console.log("Skeleton bones:", skeleton.bones);

        const targetBone = new THREE.Bone();
        skeleton.bones[0].add(targetBone); // Keep it in the skeleton hierarchy
        skeleton.bones.push(targetBone);

        // Convert the desired world position to local position
        const desiredWorldPosition = new THREE.Vector3(0, 1.1, -0.75);
        const rootWorldMatrix = skeleton.bones[0].matrixWorld;
        const rootWorldMatrixInverse = new THREE.Matrix4().copy(rootWorldMatrix).invert();
        const localPosition = desiredWorldPosition.clone()
            .applyMatrix4(rootWorldMatrixInverse);

        // Set the local position
        targetBone.position.copy(localPosition);

        // Update the target mesh to match the world position
        targetMesh.position.copy(desiredWorldPosition);
        targetBone.updateMatrixWorld(true);

        // const skeletonHelper2 = new THREE.SkeletonHelper(skeleton.bones[0]);
        // skeletonHelper2.visible = true;
        // scene.add(skeletonHelper2);

        // Setup IK
        const iks = [{
            target: 8, // Index of the end effector bone
            effector: 7, // Same as target for end effector
            links: [
                {
                    index: 6,
                    rotationMin: new THREE.Vector3(0, 0, 0),
                    rotationMax: new THREE.Vector3(0, 0, 0)
                },
                {
                    index: 5,
                    rotationMin: new THREE.Vector3(0, 0, 0),
                    rotationMax: new THREE.Vector3(0, 0, 0)
                },
                {
                    index: 4,
                    rotationMin: new THREE.Vector3(-Math.PI, 0, 0),
                    rotationMax: new THREE.Vector3(0, 0, 0)
                },
                {
                    index: 3,
                    rotationMin: new THREE.Vector3(-Math.PI / 2, 0, 0),
                    rotationMax: new THREE.Vector3(Math.PI / 2, 0, 0)
                },
                {
                    index: 2,
                    rotationMin: new THREE.Vector3(-Math.PI / 2, 0, 0),
                    rotationMax: new THREE.Vector3(Math.PI / 2, 0, 0)
                },
                {
                    index: 1,
                    rotationMin: new THREE.Vector3(0, -Math.PI, 0),
                    rotationMax: new THREE.Vector3(0, Math.PI, 0)
                },
            ],
            // iteration: 10,
            // minAngle: 0.0,
            // maxAngle: 1.0
        }];

        // Create IK Solver with the skinned mesh
        ikSolver = new CCDIKSolver(skinnedMesh, iks);

        // Store references for animation
        scene.userData.ikSolver = ikSolver;
        scene.userData.target = targetMesh;
        scene.userData.targetBone = targetBone;
        scene.userData.skinnedMesh = skinnedMesh;
    };

    setupWebSocket();
}

// Add these variables at the top of the file, outside any function
let lastControllerPosition = new THREE.Vector3();
let isSqueezing = false;
let movingModel = false;
let controllingRobot = false;

function onFrame(
    delta,
    time,
    { scene, camera, renderer, player, controllers },
) {
    if (controllers.right) {
        const { gamepad, raySpace, mesh } = controllers.right;

        if (gamepad.getButtonClick(XR_BUTTONS.BUTTON_1)) {
            exitXRSession(renderer);
        }

        if (gamepad.getButtonClick(XR_BUTTONS.BUTTON_2)) {
            if (controllingRobot) {
                controllingRobot = false
                // Set target mesh to red
                scene.userData.target.material.color.setHex(0xff0000);
            } else {
                controllingRobot = true
                // Set target mesh to green
                scene.userData.target.material.color.setHex(0x00ff00);
            }
        }

        // Get current controller position
        const currentControllerPosition = new THREE.Vector3();
        mesh.getWorldPosition(currentControllerPosition);

        // Handle squeeze button state
        if (gamepad.getButton(XR_BUTTONS.SQUEEZE)) {
            if (!isSqueezing) {
                // Just started squeezing, store initial position
                isSqueezing = true;
                lastControllerPosition.copy(currentControllerPosition);
            }

            // Calculate the movement delta in world space
            const movement = new THREE.Vector3();
            movement.subVectors(currentControllerPosition, lastControllerPosition);
            // movement.multiplyScalar(scaleFactor);

            // Get the current world position of the target
            const currentWorldPos = new THREE.Vector3();
            scene.userData.target.getWorldPosition(currentWorldPos);

            // Add the movement in world space
            currentWorldPos.add(movement);

            const skeleton = scene.userData.skinnedMesh.skeleton;

            // Convert the new world position to local space relative to the root bone
            const rootWorldMatrix = skeleton.bones[0].matrixWorld;
            const rootWorldMatrixInverse = new THREE.Matrix4().copy(rootWorldMatrix).invert();
            const localPosition = currentWorldPos.clone()
                .applyMatrix4(rootWorldMatrixInverse);

            // Update the target bone's local position
            scene.userData.targetBone.position.copy(localPosition);

            // Update the visual target mesh in world space
            scene.userData.target.position.copy(currentWorldPos);

            // Update matrices
            scene.userData.targetBone.updateMatrixWorld(true);

            lastControllerPosition.copy(currentControllerPosition);

            let triggerValue = gamepad.getButtonValue(XR_BUTTONS.TRIGGER)
            // set the Jaw rotation based on the trigger value
            if (robot) {
                robot.joints.Jaw.setJointValue((1 - triggerValue.toFixed(2)) - 0.25);
            }

            // Update IK
            if (scene.userData.ikSolver) {
                scene.userData.skinnedMesh.updateMatrixWorld(true);
                scene.userData.ikSolver.update();

                // Get joint angles after IK update


                // Create euler to convert quaternions to euler angles
                const euler = new THREE.Euler();

                euler.setFromQuaternion(skeleton.bones[1].quaternion);
                robot.joints.Rotation.setJointValue(euler.y.toFixed(2));
                euler.setFromQuaternion(skeleton.bones[2].quaternion);
                robot.joints.Pitch.setJointValue(euler.x.toFixed(2));
                euler.setFromQuaternion(skeleton.bones[3].quaternion);
                robot.joints.Elbow.setJointValue(euler.x.toFixed(2));

                // Set wrist pitch based on bone
                euler.setFromQuaternion(skeleton.bones[4].quaternion);
                robot.joints.Wrist_Pitch.setJointValue(euler.x.toFixed(2));
                // robot.joints.Wrist_Roll.setJointValue(Math.PI / 2);

                // Set wrist pitch based on rotation of controller
                // euler.setFromRotationMatrix(mesh.matrixWorld);
                // robot.joints.Wrist_Pitch.setJointValue(euler.x.toFixed(2));

                // Set wrist roll based on rotation of controller
                euler.setFromRotationMatrix(mesh.matrixWorld);
                robot.joints.Wrist_Roll.setJointValue(euler.z.toFixed(2));

                if (controllingRobot) {
                    // Send joint angles to the robot
                    sendJointAnglesToRobot();
                }
            }
        } else {
            // Reset squeezing state when button is released
            isSqueezing = false;
        }

        if (movingModel) {
            if (robot) {

                const skeleton = scene.userData.skinnedMesh.skeleton;

                // Controlling the position of the robot with the joysticks
                if (gamepad.getAxis(XR_AXES.THUMBSTICK_X) !== 0) {
                    robot.position.x += gamepad.getAxis(XR_AXES.THUMBSTICK_X) * 0.002;
                    skeleton.bones[0].position.x += gamepad.getAxis(XR_AXES.THUMBSTICK_X) * 0.002;
                    scene.userData.target.position.x += gamepad.getAxis(XR_AXES.THUMBSTICK_X) * 0.002;
                }

                if (gamepad.getAxis(XR_AXES.THUMBSTICK_Y) !== 0) {
                    robot.position.z += gamepad.getAxis(XR_AXES.THUMBSTICK_Y) * 0.002;
                    skeleton.bones[0].position.z += gamepad.getAxis(XR_AXES.THUMBSTICK_Y) * 0.002;
                    scene.userData.target.position.z += gamepad.getAxis(XR_AXES.THUMBSTICK_Y) * 0.002;
                }
            }
        }
    }

    if (controllers.left) {
        const { gamepad, raySpace, mesh } = controllers.left;

        if (gamepad.getButtonClick(XR_BUTTONS.BUTTON_1)) {
            movingModel = !movingModel;
        }

        if (movingModel) {
            if (robot) {
                const skeleton = scene.userData.skinnedMesh.skeleton;

                //control the rotation of the robot
                if (gamepad.getAxis(XR_AXES.THUMBSTICK_X) !== 0) {
                    robot.rotation.z += gamepad.getAxis(XR_AXES.THUMBSTICK_X) * -0.02;
                    skeleton.bones[0].rotation.y += gamepad.getAxis(XR_AXES.THUMBSTICK_X) * -0.02;
                    scene.userData.target.rotation.y += gamepad.getAxis(XR_AXES.THUMBSTICK_X) * -0.02;
                }

                // change the height of the robot
                if (gamepad.getAxis(XR_AXES.THUMBSTICK_Y) !== 0) {
                    robot.position.y += gamepad.getAxis(XR_AXES.THUMBSTICK_Y) * -0.002;
                    skeleton.bones[0].position.y += gamepad.getAxis(XR_AXES.THUMBSTICK_Y) * -0.002;
                    scene.userData.target.rotation.y += gamepad.getAxis(XR_AXES.THUMBSTICK_X) * -0.02;
                }
            }
        }

    }
}

let ws

function setupWebSocket() {
    // Connect to the websocket
    // Get the current base URL
    const baseUrl = window.location.hostname;

    // Create the WebSocket URL using the base URL and port 8765
    const wsUrl = `ws://${baseUrl}:8765`;

    // Establish the WebSocket connection
    ws = new WebSocket(wsUrl)

    ws.onopen = function () {
        console.log('WebSocket connection established');
    };

    ws.onmessage = function (event) {
        console.log('Message from server:', event.data);
    };

    ws.onerror = function (error) {
        console.error('WebSocket error:', error);
    };

    ws.onclose = function (event) {
        console.log('WebSocket connection closed:', event.code, event.reason);
    };
}

function sendJointAnglesToRobot() {
    if (robot) {
        // console.log(robot.joints.Rotation)
        // Get joint angles from sliders
        const jointAngles = {
            Rotation: robot.joints.Rotation.jointValue[0] * -1,
            Pitch: robot.joints.Pitch.jointValue[0] + Math.PI / 2,
            Elbow: robot.joints.Elbow.jointValue[0],
            Wrist_Pitch: robot.joints.Wrist_Pitch.jointValue[0] + Math.PI / 2,
            Wrist_Roll: robot.joints.Wrist_Roll.jointValue[0] - Math.PI / 2,
            Jaw: robot.joints.Jaw.jointValue[0],
        };
        console.log(jointAngles);
        // send the joint angles to the websocket
        ws.send(JSON.stringify(jointAngles));

    }
}

function exitXRSession(renderer) {
    if (renderer.xr.isPresenting) {
        const session = renderer.xr.getSession();
        if (session) {
            session.end();
            console.log("Exiting XR session");
        }
    }
}


init(setupScene, onFrame);
