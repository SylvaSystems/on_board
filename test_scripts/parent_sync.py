import gpiod
import time

PIN = 24  # GPIO24 = physical pin 18

PULSES = 20
PERIOD_S = 0.5
WIDTH_S = 0.02

chip = gpiod.Chip("/dev/gpiochip0")

req = chip.request_lines(
    consumer="parent",
    config={
        PIN: gpiod.LineSettings(
            direction=gpiod.line.Direction.OUTPUT
        )
    }
)

start_ns = time.monotonic_ns() + 2_000_000_000

print(f"START_NS={start_ns}")
print(f"PERIOD_NS={int(PERIOD_S * 1e9)}")
print("Sending pulses...")

for i in range(PULSES):
    rise_ns = start_ns + int(i * PERIOD_S * 1e9)
    fall_ns = rise_ns + int(WIDTH_S * 1e9)

    while time.monotonic_ns() < rise_ns:
        pass
    req.set_value(PIN, gpiod.line.Value.ACTIVE)

    while time.monotonic_ns() < fall_ns:
        pass
    req.set_value(PIN, gpiod.line.Value.INACTIVE)

    print(f"pulse {i} scheduled_rise_ns={rise_ns}")

print("Done")
