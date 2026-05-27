# Copyright (C) 2026 icarus

import asyncio
import logging
import signal
import ulog
import olympe

from olympe.messages.ardrone3.Piloting import moveBy, TakeOff
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

# This is limited to 15 charecters
PROCESS_NAME = b"move"

def move_square(drone):
    drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=10)).wait()
    drone(moveBy(2.0, 0, 0, 0) ).wait()
    drone(moveBy(0, 2.0, 0, 0) ).wait()
    drone(moveBy(-2.0, 0, 0, 0) ).wait()
    drone(moveBy(0, -2.0, 0, 0) ).wait()
    return True


async def service_main():
    # Initialisation code
    #
    # The service is automatically started by the drone when the mission is
    # loaded.
    ulog.setup_logging(PROCESS_NAME)
    logger = logging.getLogger('main')
    logger.info("Hello from move")
    run = True
    def sig_handler(*_):
        nonlocal run
        run = False

    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, sig_handler)

    # Create LocalController instance
    with olympe.LocalController() as drone:
        # Connect to drone
        res = drone.connect()
        if not res:
            logger.error("Failed to connect to drone")
            return 1
        logger.info("Connected to drone")

        # make square movement
        res = move_square(drone)
        if not res:
            logger.error("Failed to move in square")
            return 1
        logger.info("Square movement successful")

        # Disconnect from drone
        drone.disconnect()
        logger.info("Disconnected from drone")
    
    while run:
        await asyncio.sleep(1)

    logger.info("Exiting move")
    return 0
        


def main():
    asyncio.run(service_main())