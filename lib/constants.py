# Gets the values from the .env
from dotenv import load_dotenv
import os
lib_dir = os.path.dirname(os.path.abspath(__file__))
env_path = os.path.join(lib_dir, ".env")
load_dotenv(env_path)


PI_PIN = 24 # Communication pin between the two pis
CHIP_NAME = "/dev/gpiochip0" # The name of the chip on both raspberry pi
FRAME_RATE = 1 # The rate to take photos

CSV_PATH = f"{os.getenv('LOG_PATH')}/log.csv" # The path to save the csv log for pose
IMG_PATH = f"{os.getenv('LOG_PATH')}/images" # The path to the directory to log the images

IMAGE_SIZE = (4656,3496)

MAVLINK_DEVICE = "/dev/ttyAMA0" # The device to connect to the pixhawk
MAVLINK_BAUD = 57600 # The baud rate for the pixhawk

TOGGLE_CHANNEL = 6 # The channel that controls the toggle
TOGGLE_THRESHOLD = 1500 # The threshold for the toggle
