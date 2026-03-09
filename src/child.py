"""
@author Cole Malinchock
@file child.py
"""

import os
import time

import cv2
import gpiod
from picamera2 import Picamera2

from lib.constants import (
    PI_PIN,
    CHIP_NAME,
    IMG_PATH,
    IMAGE_SIZE
)
from lib.structs import Img


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

        self.picam = Picamera2()
        camera_config = self.picam.create_still_configuration(
            main={"size": IMAGE_SIZE, "format": "RGB888"}
        )
        self.picam.configure(camera_config)
        self.picam.start()
        time.sleep(2)

    def run(self):
        """Main loop."""

        while True:
            events = self.pi_req.read_edge_events()

            if events:
                for _event in events:
                    self.save_image()

    def save_image(self):
        """Captures and saves one image."""
        img = self._get_image()
        cv2.imwrite(img.path_name, img.img)
        print(f"Saved image: {img.path_name}")
        self.idx += 1

    def _get_image(self) -> Img:
        """Gets the image from the camera and returns it."""
        frame = self.picam.capture_array()

        return Img(
            img=frame,
            path_name=f"{IMG_PATH}/{self.idx:010d}.jpg",
            timestamp=time.time(),
            id=self.idx
        )


if __name__ == "__main__":
    child = Child()
    child.run()
