#!/usr/bin/env python3
import sys, time, signal

CHIP = "gpiochip0"
LINE = 125

try:
    import gpiod
except ImportError:
    print("Install: sudo apt install -y python3-libgpiod", file=sys.stderr)
    sys.exit(1)

stop = False
def _stop(*_):
    global stop
    stop = True

signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)

def main():
    chip = gpiod.Chip(CHIP)

    # Try libgpiod v2 style
    try:
        req = chip.request_lines(
            consumer="pump_hold",
            config={
                LINE: gpiod.LineSettings(
                    direction=gpiod.line.Direction.OUTPUT,
                    output_value=gpiod.line.Value.ACTIVE,
                )
            },
        )
        while not stop:
            time.sleep(1)

        try:
            req.set_values({LINE: gpiod.line.Value.INACTIVE})
        except Exception:
            pass
        req.release()
        chip.close()
        return
    except Exception:
        pass

    # Fallback: libgpiod v1 style
    line = chip.get_line(LINE)
    line.request(consumer="pump_hold", type=gpiod.LINE_REQ_DIR_OUT, default_val=1)
    while not stop:
        time.sleep(1)
    try:
        line.set_value(0)
    except Exception:
        pass
    line.release()
    chip.close()

if __name__ == "__main__":
    main()