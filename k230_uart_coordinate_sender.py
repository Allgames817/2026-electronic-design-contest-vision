"""Send K230 ball coordinates to a TI MSPM0 through UART2.

Wiring:
    K230 Pin17 / IO5 / UART2_TXD  ->  MSPM0G3507 PA11 / UART RX
    K230 Pin37 / GND              ->  MSPM0G3507 GND

UART2_RXD is also mapped to K230 IO6 because CanMV requires both UART
directions to be configured before UART2 can be created. IO6 does not need
to be connected for this one-way transmission test.

Serial protocol:
    115200 baud, 8 data bits, no parity, 1 stop bit (8N1)
    One ASCII coordinate frame per line: x,y\n
Example frame:
    160,120\n
Run this file directly in CanMV IDE to transmit the fixed test coordinate
(160, 120) every 500 ms. To use real detection results, call
send_coordinate(ball_x, ball_y) from the detection loop instead.
"""

from machine import FPIOA, UART
import time


# K230 Pin17 is SoC IO5. Pin37 is GND and needs no software setup.
UART_TX_IO = 5
UART_RX_IO = 6
UART_BAUDRATE = 115200
SEND_INTERVAL_MS = 500


def init_uart2():
    """Configure UART2 pins and return the UART2 object."""
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_IO, FPIOA.UART2_TXD)
    # CanMV v1.8 asserts unless UART2 RX is mapped as well. The RX pin may
    # remain physically unconnected when communication is TX-only.
    fpioa.set_function(UART_RX_IO, FPIOA.UART2_RXD)

    uart = UART(
        UART.UART2,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE
    )
    return fpioa, uart


def send_coordinate(uart, x, y):
    """Send one coordinate frame and return the transmitted text."""
    frame = "%d,%d\n" % (int(x), int(y))
    uart.write(frame.encode("ascii"))
    return frame


def main():
    # Keep fpioa referenced while UART is in use.
    fpioa, uart = init_uart2()

    print(
        "UART2 ready: Pin17/IO5 TX, IO6 RX (unconnected), %d baud, 8N1" %
        UART_BAUDRATE
    )
    print("Press Ctrl+C to stop")

    # Fixed coordinates for the first wiring/communication test.
    x = 160
    y = 120

    try:
        while True:
            frame = send_coordinate(uart, x, y)
            # end="" avoids printing a second newline because frame has \n.
            print("send:", frame, end="")
            time.sleep_ms(SEND_INTERVAL_MS)
    except KeyboardInterrupt:
        print("\nCoordinate transmission stopped")
    finally:
        uart.deinit()
        uart = None
        fpioa = None


if __name__ == "__main__":
    main()
