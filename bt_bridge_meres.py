import serial
import sys
import time
import random


ROBOT_PORT = "/dev/ttyACM0"
BT_PORT    = "/dev/rfcomm0"
BAUD       = 115200

PRINT_DEBUG = False

ENABLE_DISTURBANCES = True

LOSS_PROB = 0.5

#masodperc
BASE_DELAY_SEC = 0.0

#maximum masodperc
JITTER_MAX_SEC = 0.0


def apply_disturbances(tag: str) -> bool:
    
    if not ENABLE_DISTURBANCES:
        return False

    # 1) Packet loss
    if random.random() < LOSS_PROB:
        if PRINT_DEBUG:
            print(f"{tag} DROP")
        return True

    # 2) Base delay + jitter
    delay = BASE_DELAY_SEC
    if JITTER_MAX_SEC > 0:
        delay += random.uniform(0.0, JITTER_MAX_SEC)

    if delay > 0:
        time.sleep(delay)

    return False


def open_serial(port):
    return serial.Serial(
        port,
        BAUD,
        timeout=0.001,         
        write_timeout=0.001,
        rtscts=False,
        dsrdtr=False,
        xonxoff=False
    )


robot = open_serial(ROBOT_PORT)
bt    = open_serial(BT_PORT)

# Buffers per direction
buf_RtoC = bytearray()
buf_CtoR = bytearray()


def pump(src, dst, buf: bytearray, tag: str):
    """
    Read any available bytes from src, append to buffer,
    extract full lines ending in '\n', and forward (or drop) them.
    """
    # Read all currently available bytes
    try:
        waiting = src.in_waiting
    except OSError:
        waiting = 0

    if waiting:
        chunk = src.read(waiting)
        if chunk:
            buf.extend(chunk)

    # Process complete lines
    while True:
        newline_index = buf.find(b'\n')
        if newline_index == -1:
            break  # no complete line yet

        # Extract one full line INCLUDING '\n'
        line = buf[:newline_index + 1]
        del buf[:newline_index + 1]

        # Apply disturbances on a per-line basis
        if apply_disturbances(tag):
            # drop the line
            continue

        try:
            dst.write(line)
        except Exception as e:
            if PRINT_DEBUG:
                print(f"{tag} write error: {e}", file=sys.stderr)

        if PRINT_DEBUG:
            try:
                sys.stdout.write(f"{tag}{line.decode(errors='ignore')}")
                sys.stdout.flush()
            except Exception:
                pass


print("Transmitter running (USB <-> RFCOMM) with line-based loss + delay + jitter.")

while True:
    pump(robot, bt, buf_RtoC, "R→C ")
    pump(bt, robot, buf_CtoR, "C→R ")
    time.sleep(0.0005)
