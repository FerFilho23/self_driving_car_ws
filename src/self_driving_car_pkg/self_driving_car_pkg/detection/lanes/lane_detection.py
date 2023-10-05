from .color_segmentation import segment_lanes
from ...config import config

def detect_lanes(frame):
    # Cropping the img (keep only the street that is below the horizon)
    img_cropped = frame[config.CropHeight_resized:,:]

    segment_lanes(img_cropped, config.minArea_resized)