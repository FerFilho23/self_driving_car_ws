import cv2
from .color_segmentation import segment_lanes
from ...config import config

def detect_lanes(frame):
    # Cropping the img (keep only the street that is below the horizon)
    img_cropped = frame[config.CropHeight_resized:,:]

    mid_lane_mask,mid_lane_edge,outer_lane_edge,outerlane_side_sep,outerlane_points = segment_lanes(img_cropped,config.minArea_resized)
    cv2.imshow("mid_lane_mask",mid_lane_mask)
    cv2.imshow("mid_lane_edge",mid_lane_edge)
    cv2.imshow("outer_lane_edge",outer_lane_edge)
    cv2.imshow("outerlane_side_sep",outerlane_side_sep)
    cv2.waitKey(1)
