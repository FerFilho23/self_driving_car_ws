import cv2
from .color_segmentation import segment_lanes
from .midlane_estimation import estimate_midlane
from ...config import config
from .cleaning import GetYellowInnerEdge, ExtendShortLane
from .data_extraction import FetchInfoAndDisplay

def detect_lanes(img):
    # cropping the roi (e.g keeping only below the horizon)
    img_cropped = img[config.CropHeight_resized:,:]

    midlane_mask,midlane_edge,outerlane_edge,outerlane_side_sep,outerlane_points = segment_lanes(img_cropped,config.minArea_resized)

    estimated_midlane = estimate_midlane(midlane_edge,config.MaxDist_resized)
    outerlane_oneSide,outer_cnts_oneSide,mid_cnts,offset_correction = GetYellowInnerEdge(outerlane_side_sep,estimated_midlane,outerlane_points)#3ms
    extended_midlane,extended_outerlane = ExtendShortLane(estimated_midlane,mid_cnts,outer_cnts_oneSide,outerlane_oneSide.copy())

    distance, angle = FetchInfoAndDisplay(midlane_edge,extended_midlane,extended_outerlane,img_cropped,offset_correction)

    # cv2.imshow("mid_lane_mask",mid_lane_mask)
    # cv2.imshow("mid_lane_edge",mid_lane_edge)
    # cv2.imshow("outer_lane_edge",outer_lane_edge)
    # cv2.imshow("outerlane_side_sep",outerlane_side_sep)
    # cv2.imshow("estimated_midlane",estimated_midlane)

    # cv2.imshow("OuterLane_OneSide",OuterLane_OneSide)
    cv2.imshow("extended_midlane",extended_midlane)
    cv2.imshow("extended_outerlane",extended_outerlane)
    
    cv2.waitKey(1)
    return distance, angle