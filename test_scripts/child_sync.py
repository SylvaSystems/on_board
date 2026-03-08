import gpiod
import statistics

PIN = 24

# Copy these two values from the parent output for the test
START_NS = 0
PERIOD_NS = 500_000_000
PULSES = 20

chip = gpiod.Chip("/dev/gpiochip0")

req = chip.request_lines(
    consumer="child",
    config={
        PIN: gpiod.LineSettings(
            direction=gpiod.line.Direction.INPUT,
            edge_detection=gpiod.line.Edge.RISING,
            event_clock=gpiod.line.Clock.MONOTONIC,
        )
    }
)

print("Waiting for pulses...")

observed = []

while len(observed) < PULSES:
    events = req.read_edge_events()
    if events:
        for event in events:
            ts = event.timestamp_ns
            observed.append(ts)
            print(f"pulse {len(observed)-1} observed_ns={ts}")
            if len(observed) >= PULSES:
                break

errors = []
for i, t_obs in enumerate(observed):
    expected = START_NS + i * PERIOD_NS
    errors.append(t_obs - expected)

median_offset = int(statistics.median(errors))
mean_offset = int(statistics.mean(errors))
stdev_ns = statistics.pstdev(errors) if len(errors) > 1 else 0.0
max_abs_err = max(abs(e) for e in errors)

print("\nResults:")
print(f"median offset (child - parent) ns: {median_offset}")
print(f"mean offset   (child - parent) ns: {mean_offset}")
print(f"std dev ns: {stdev_ns:.1f}")
print(f"max abs error ns: {max_abs_err}")
print(f"median offset ms: {median_offset / 1e6:.6f}")
print(f"std dev us: {stdev_ns / 1e3:.3f}")
print(f"max abs error us: {max_abs_err / 1e3:.3f}")
