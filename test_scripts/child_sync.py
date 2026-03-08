import gpiod
import time

PIN = 24

chip = gpiod.Chip("/dev/gpiochip0")

line = chip.request_lines(
    consumer="child",
    config={
        PIN: gpiod.LineSettings(
            direction=gpiod.line.Direction.INPUT,
            edge_detection=gpiod.line.Edge.RISING
        )
    }
)

print("Waiting for pulses...")

while True:

    event = line.read_edge_events()

    if event:
        t = time.monotonic_ns()
        print("Pulse received at", t)
