#!/usr/bin/env python3

import asyncio
from px4py import *
from mavsdk import System

async def run():
    # Connect to the PX4 flight stack
    px4 = PX4()
    px4.start()

    # Connect to the drone via MAVSDK
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for vehicle to connect...")
    async for state in px4.vehicle_state():
        if state.connected:
            print("-- Connected to vehicle!")
            break

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

    print("Fetching AMSL altitude at home location....")
    async for home in px4.home_position():
        absolute_altitude = home.altitude
        break

    print("-- Arming")
    await px4.commander.arm()

    print("-- Taking off")
    await px4.commander.takeoff()

    await asyncio.sleep(1)
    # To fly drone 20m above the ground plane
    flying_alt = absolute_altitude + 20.0
    # goto_location() takes AMSL altitude
    await px4.commander.goto_location(47.397606, 8.543060, flying_alt, 0)

    while True:
        print("Staying connected, press Ctrl-C to exit")
        await asyncio.sleep(1)

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
