#!/usr/bin/env python3
"""
@author Cole Malinchock
@file child.py
"""

import os
import subprocess
import time

import gpiod

from lib.constants import (
    PI_PIN,
    CHIP_NAME,
    IMG_PATH,
    IMAGE_SIZE,
)


class Child:
    """Class for handling the child operations."""

    def __init__(self):
        self.chip = gpiod.Chip(CHIP_NAME)
        self.pi_req = self.chip.request_lines(
            consumer="child",
            config={
                PI_PIN: gpiod.LineSettings(
                    direction=gpiod.line.Direction.INPUT,
                    edge_detection=gpiod.line.Edge.RISING
                )
            }
        )

        self.idx = 0
        os.makedirs(IMG_PATH, exist_ok=True)

    def run(self):
        """Main loop."""
        while True:
            events = self.pi_req.read_edge_events()
            if events:
                for _event in events:
                    self.save_image()

    def save_image(self):
        """Captures and saves one image."""
        path = f"{IMG_PATH}/{self.idx:010d}.jpg"

        subprocess.run([
            "rpicam-still",
            "-t", "1000",
            "--width", str(IMAGE_SIZE[0]),
            "--height", str(IMAGE_SIZE[1]),
            "--autofocus-on-capture",
            "--nopreview",
            "-o", path,
        ], check=True)

        print(f"Saved image: {path}", flush=True)
        self.idx += 1


if __name__ == "__main__":
    child = Child()
    child.run()