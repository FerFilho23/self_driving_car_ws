import cv2
from .detection.lanes.lane_detection import detect_lanes

class Car():

    def drive_car(self, frame):
        # Resizing to minimize computation time while still achieving results
        img = frame[0:640, 238:1042]
        img = cv2.resize(img, (320, 240))

        detect_lanes(img)