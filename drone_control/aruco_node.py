#!/usr/bin/env python3

import asyncio
from mavsdk import System, aruco


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(1)
    # To fly drone 20m above the ground plane
    flying_alt = absolute_altitude + 20.0
    # goto_location() takes Absolute MSL altitude
    await drone.action.goto_location(47.397606, 8.543060, flying_alt, 0)

    # Start the ArUco detection
    await drone.camera.start_video_streaming()
    print("Starting ArUco detection...")

    async for detection in drone.camera.video_stream():
        # Check if an ArUco marker was detected
        if detection.aruco_detection:
            print(f"ArUco marker detected with ID {detection.aruco_detection.id}")
            # Land on the detected ArUco marker
            await drone.action.land_on_aruco(detection.aruco_detection.id, 0.2)
            break

    print("Stopping ArUco detection...")
    await drone.camera.stop_video_streaming()

    print("Landing...")
    await drone.action.land()


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
