#!/usr/bin/env python3
"""
@author Cole Malinchock
@file parent.py
"""

import csv
import os
import time
import subprocess

import cv2
import gpiod
from picamera2 import Picamera2
from libcamera import controls
from pymavlink import mavutil

from lib.constants import (
    PI_PIN,
    CHIP_NAME,
    FRAME_RATE,
    CSV_PATH,
    IMG_PATH,
    IMAGE_SIZE,
    TOGGLE_CHANNEL,
    TOGGLE_THRESHOLD,
)
from lib.structs import Pose, Img


class Parent:
    """Class for handling the parent operations."""

    def __init__(self):

        # Setup the gpio to the child
        self.chip = gpiod.Chip(CHIP_NAME)
        self.pi_req = self.chip.request_lines(
            consumer="parent",
            config={
                PI_PIN: gpiod.LineSettings(
                    direction=gpiod.line.Direction.OUTPUT
                )
            }
        )
        self.child_line_active = False
        self.pi_req.set_value(PI_PIN, gpiod.line.Value.INACTIVE)

        # Initialization sequence for variables
        self.idx = 0
        self.enabled = False
        self.last_capture_time = 0.0

        # Create the paths
        os.makedirs(IMG_PATH, exist_ok=True)
        os.makedirs(os.path.dirname(CSV_PATH), exist_ok=True)
        self._init_csv()

        # Initialization sequence for camera
        self.picam = Picamera2()
        camera_config = self.picam.create_still_configuration(
            main={"size": IMAGE_SIZE, "format": "RGB888"}
        )
        self.picam.configure(camera_config)
        self.picam.start()
        time.sleep(2)

        # Initialization sequence for pixhawk
        self.master = mavutil.mavlink_connection("udp:127.0.0.1:14550")

        # Gets the pixhawk heartbeat
        print("Waiting for heartbeat from Pixhawk...")
        self.master.wait_heartbeat()
        print("Heartbeat received.")

        # Cached MAVLink data
        self.latest_global_position = None
        self.latest_local_position = None
        self.latest_attitude = None
        self.latest_rc_channels = None


    def run(self):
        """Main loop."""
        while True:
            self._poll_mavlink()

            new_enabled = self.is_enabled()

            if new_enabled != self.enabled:
                self.enabled = new_enabled
                if self.enabled:
                    print("Toggle switched HIGH: capture enabled")
                else:
                    print("Toggle switched LOW: capture disabled")

            if self.enabled:

                if time.time() - self.last_capture_time >= (1.0 / FRAME_RATE):
                    self.last_capture_time = time.time()
                    self.save_image_and_pose()
                    self.idx += 1

            time.sleep(0.005)


    def is_enabled(self) -> bool:
        """
        Returns True when the chosen RC channel is above threshold.
        Reads from cached RC_CHANNELS message.
        """

        msg = self.latest_rc_channels
        if msg is None:
            return False

        field_name = f"chan{TOGGLE_CHANNEL}_raw"
        pwm = getattr(msg, field_name, None)
        if pwm is None:
            return False

        return pwm >= TOGGLE_THRESHOLD


    def save_image_and_pose(self):

        self._notify_child()

        print("Child notified")

        path = f"{IMG_PATH}/{self.idx:010d}.jpg"
        t_img = time.time()
        self._capture_image_file(path)
        print("Image captured")
        pose = self._get_pose()

        print("Pose received")

        with open(CSV_PATH, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                self.idx, t_img, pose.timestamp, path,
                pose.lat, pose.lon, pose.alt,
                pose.x, pose.y, pose.z,
                pose.roll, pose.pitch, pose.yaw,
                int(self.enabled)
            ])

        print("Image saved!")


    def _notify_child(self):
        """
        Pulse the child line. Child should watch for edge.
        """
        self.pi_req.set_value(PI_PIN, gpiod.line.Value.ACTIVE)
        time.sleep(0.002)  # 2 ms pulse
        self.pi_req.set_value(PI_PIN, gpiod.line.Value.INACTIVE)


    def _get_image(self) -> Img:
        frame = self.picam.capture_array()
        ts = time.time()

        return Img(
            img=frame,
            path_name=f"{IMG_PATH}/{self.idx:010d}.jpg",
            timestamp=ts,
            id=self.idx,
        )


    def _get_pose(self) -> Pose:
        """
        Build Pose from latest cached MAVLink data.
        GLOBAL_POSITION_INT gives global position.
        ATTITUDE gives roll/pitch/yaw.
        LOCAL_POSITION_NED gives x/y/z if streamed.
        """
        lat = lon = alt = float("nan")
        x = y = z = float("nan")
        roll = pitch = yaw = float("nan")

        if self.latest_global_position is not None:
            gp = self.latest_global_position
            lat = gp.lat / 1e7
            lon = gp.lon / 1e7
            alt = gp.relative_alt / 1000.0  # mm -> m

        if self.latest_local_position is not None:
            lp = self.latest_local_position
            x = lp.x
            y = lp.y
            z = lp.z

        if self.latest_attitude is not None:
            att = self.latest_attitude
            roll = att.roll
            pitch = att.pitch
            yaw = att.yaw

        return Pose(
            lat=lat, lon=lon, alt=alt,
            x=x, y=y, z=z,
            roll=roll, pitch=pitch, yaw=yaw,
            timestamp=time.time(), id=self.idx
        )


    def _poll_mavlink(self):
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()
            # print(msg_type)

            if msg_type == "GLOBAL_POSITION_INT":
                self.latest_global_position = msg
            elif msg_type == "LOCAL_POSITION_NED":
                self.latest_local_position = msg
            elif msg_type == "ATTITUDE":
                self.latest_attitude = msg
            elif msg_type == "RC_CHANNELS":
                self.latest_rc_channels = msg


    def _init_csv(self):
        """ Initializes the csv """

        # Checks that the file exists
        file_exists = os.path.exists(CSV_PATH)

        # Writes to the csv if it doesn't exist yet
        with open(CSV_PATH, "a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    "id", "img_timestamp", "pose_timestamp", "img_path",
                    "lat_deg", "lon_deg", "alt_m",
                    "x_m", "y_m", "z_m",
                    "roll_rad", "pitch_rad", "yaw_rad",
                    "toggle_enabled",
                ])


    def _capture_image_file(self, path: str):
        subprocess.run([
            "rpicam-still",
            "-t", "1000",
            "--width", str(IMAGE_SIZE[0]),
            "--height", str(IMAGE_SIZE[1]),
            "--autofocus", "manual",
            "--nopreview",
            "-o", path,
        ], check=True)


if __name__ == "__main__":
    parent = Parent()
    parent.run()
