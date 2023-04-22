import asyncio
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from mavsdk import System
from mavsdk import (OffboardError, PositionNEDYaw)

class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')

        # Connect to the drone
        self.drone = System()
        self.drone.connect(system_address="udp://:14540")

        # Set up ROS2 subscriptions and publishers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_pose', 10)

        # Define the ArUco marker parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Define the marker size in meters
        self.marker_size = 0.1

        # Set up the offboard mode
        self.offboard_task = asyncio.ensure_future(self.offboard_control())

    def image_callback(self, msg):
        # Convert the ROS2 Image message to a NumPy array
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect ArUco marker in the image
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and len(ids) > 0:
            # Get the first detected marker (assuming only one marker is present)
            marker_corners = corners[0]
            marker_id = ids[0][0]

            # Estimate the pose of the marker in 3D space
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners, self.marker_size, self.drone.camera.get_intrinsics().as_opencv_matrix(), None)

            # Convert the rotation vector to a rotation matrix
            rot_matrix, _ = cv2.Rodrigues(rvec)

            # Compute the position and orientation of the marker in 3D space
            position = tvec[0]
            orientation = cv2.decomposeProjectionMatrix(np.hstack((rot_matrix, np.zeros((3, 1)))))[6]

            # Publish the marker pose as a ROS2 PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'aruco_marker'
            pose_msg.pose.position.x = position[0]
            pose_msg.pose.position.y = position[1]
            pose_msg.pose.position.z = -position[2]
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = orientation[2]
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)

    async def offboard_control(self):
        # Wait for the drone to connect
        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is
