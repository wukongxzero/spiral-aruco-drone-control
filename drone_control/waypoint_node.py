import asyncio
import rclpy
from rclpy.node import Node
from px4_msgs.msg import PositionSetpoint
from mav_msgs.msg import PositionTarget
from mavsdk import System, OffboardError
from mavsdk.offboard import Attitude
from mavsdk.mission import MissionPlan, MissionItem, MissionItemWaypoint


class GoToWaypointNode(Node):
    def __init__(self):
        super().__init__('go_to_waypoint_node')

        self.create_subscription(PositionSetpoint, 'mavros/setpoint_position/local', self.callback, 10)
        self.pub_setpoint_raw = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/local', 10)

        asyncio.get_event_loop().run_until_complete(self.run())

    async def run(self):
        # Connect to drone using mavsdk
        drone = System()
        await drone.connect(system_address="udp://:14540")

        # Wait for drone to be ready
        async for state in drone.core.connection_state():
            if state.is_connected:
                break

        # Set offboard mode
        try:
            await drone.offboard.set_mode(Attitude())
        except OffboardError as error:
            self.get_logger().error(f"Could not set offboard mode: {error}")

        # Create mission plan with two waypoints
        mission_items = []
        mission_items.append(MissionItem(mission_item=MissionItemWaypoint(0, 0, 0, 0, 0, 0, 0)))
        mission_items.append(MissionItem(mission_item=MissionItemWaypoint(0, 0, 5, 0, 0, 0, 0)))
        mission_plan = MissionPlan(mission_items)

        # Upload mission to drone
        try:
            await drone.mission.upload_mission(mission)
