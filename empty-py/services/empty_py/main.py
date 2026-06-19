# Copyright (C) 2026 icarus

import asyncio
import logging
import signal
import ulog

# This is limited to 15 characters
PROCESS_NAME = b"empty-py"


async def service_main():
    # Initialisation code
    #
    # The service is automatically started by the drone when the mission is
    # loaded.
    ulog.setup_logging(PROCESS_NAME)
    logger = logging.getLogger('main')
    logger.info("Hello from empty-py")
    run = True
    def sig_handler(*_):
        nonlocal run
        run = False

    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, sig_handler)

    # Loop code
    #
    # The service is assumed to run an infinite loop, and termination
    # requests are handled via a SIGTERM signal.
    # If your service exits before this SIGTERM is sent, it will be
    # considered as a crash, and the system will relaunch the service.
    # If this happens too many times, the system will no longer start the
    # service.
    while run:
        logger.info("Running ...")
        await asyncio.sleep(1)

    # Cleanup code
    #
    # When stopped by a SIGTERM, a service can use a short amount of time
    # for cleanup (typically closing opened files and ensuring that the
    # written data is coherent).
    logger.info("Cleaning up from empty-py")
    return 0


def main():
    asyncio.run(service_main())