import gpiod
import time

PIN = 24   # GPIO24 = physical pin 18

chip = gpiod.Chip("/dev/gpiochip0")

line = chip.request_lines(
    consumer="parent",
    config={
        PIN: gpiod.LineSettings(
            direction=gpiod.line.Direction.OUTPUT
        )
    }
)

print("Sending pulses...")

for i in range(10):

    line.set_value(PIN, 1)
    print("pulse HIGH", i)
    time.sleep(0.02)

    line.set_value(PIN, 0)
    print("pulse LOW", i)
    time.sleep(0.5)

print("Done")
