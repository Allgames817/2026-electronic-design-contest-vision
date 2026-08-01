"""K230 automatic startup entry for firmware that launches main.py."""

import gc
import sys
import time


STARTUP_DELAY_MS = 1500
RETRY_DELAY_MS = 3000


def run():
    print("main.py: auto-start entry")
    # Let the camera and Wi-Fi hardware finish powering up first.
    time.sleep_ms(STARTUP_DELAY_MS)

    while True:
        try:
            import mjpeg_steel_ball_server
            mjpeg_steel_ball_server.main()
        except KeyboardInterrupt:
            print("Auto-start stopped by user")
            raise
        except Exception as error:
            print("Wi-Fi stream stopped; retrying in %d ms" % RETRY_DELAY_MS)
            sys.print_exception(error)

        gc.collect()
        time.sleep_ms(RETRY_DELAY_MS)


run()
