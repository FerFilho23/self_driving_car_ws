import cv2
import numpy as np

from .Morph_op import BwareaOpen, Ret_LowestEdgePoints, RetLargestContour_OuterLane

# Initialize HLS values for white and yellow regions
hls = 0
src = 0

# Initialize HLS color range values for white and yellow regions
low_H_white, low_L_white, low_S_white = 0, 125, 0
low_H_yellow, low_L_yellow, low_S_yellow = 23, 0, 0

def color_segment(hls_frame, lower_range, upper_range):
    # Create a binary mask to isolate pixels within the specified color range
    mask_in_range = cv2.inRange(hls_frame, lower_range, upper_range)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask_dilated = cv2.morphologyEx(mask_in_range, cv2.MORPH_DILATE, kernel)
    return mask_dilated

def maskextract():

    mask   = color_segment(hls,(low_H_white,low_L_white,low_S_white),(255,255,255))
    mask_y = color_segment(hls,(low_H_yellow,low_L_yellow ,low_S_yellow),(255,255,255))

    mask_ = mask != 0
    dst = src * (mask_[:,:,None].astype(src.dtype))

    mask_y_ = mask_y != 0
    dst_Y = src * (mask_y_[:,:,None].astype(src.dtype))

    cv2.imshow('white_regions',dst)
    cv2.imshow('yellow_regions',dst_Y)

# Trackbar callback functions to adjust HLS values
def on_hue_low_change_white(val):
    global low_H_white
    low_H_white = val
    maskextract()

def on_lit_low_change_white(val):
    global low_L_white
    low_L_white = val
    maskextract()

def on_sat_low_change_white(val):
    global low_S_white
    low_S_white = val
    maskextract()

def on_hue_low_change_yellow(val):
    global low_H_yellow
    low_H_yellow = val
    maskextract()

def on_lit_low_change_yellow(val):
    global low_L_yellow
    low_L_yellow = val
    maskextract()

def on_sat_low_change_yellow(val):
    global low_S_yellow
    low_S_yellow = val
    maskextract()

cv2.namedWindow("white_regions")
cv2.namedWindow("yellow_regions")

# Create trackbars for adjusting HLS values
cv2.createTrackbar("Hue_Low", "white_regions", low_H_white, 255, on_hue_low_change_white)
cv2.createTrackbar("Lit_Low", "white_regions", low_L_white, 255, on_lit_low_change_white)
cv2.createTrackbar("Sat_Low", "white_regions", low_S_white, 255, on_sat_low_change_white)

cv2.createTrackbar("Hue_Low", "yellow_regions", low_H_yellow, 255, on_hue_low_change_yellow)
cv2.createTrackbar("Lit_Low", "yellow_regions", low_L_yellow, 255, on_lit_low_change_yellow)
cv2.createTrackbar("Sat_Low", "yellow_regions", low_S_yellow, 255, on_sat_low_change_yellow)

def get_mask_and_edge_of_larger_objects(frame,mask,min_area):
    # Keeping only objects larger then min_area
    frame_roi = cv2.bitwise_and(frame,frame,mask=mask)
    frame_roi_gray = cv2.cvtColor(frame_roi,cv2.COLOR_BGR2GRAY)
    mask_of_larger_objects = BwareaOpen(frame_roi_gray,min_area)
    frame_roi_gray = cv2.bitwise_and(frame_roi_gray,mask_of_larger_objects)
    # Extracting Edges of those larger objects
    frame_roi_smoothed = cv2.GaussianBlur(frame_roi_gray,(11,11),1)
    edges_of_larger_objects = cv2.Canny(frame_roi_smoothed,50,150, None, 3)

    return mask_of_larger_objects,edges_of_larger_objects

def segment_midlane(frame, white_regions, min_area):
    mid_lane_mask, mid_lane_edge = get_mask_and_edge_of_larger_objects(frame, white_regions, min_area)
    return mid_lane_mask, mid_lane_edge

def segment_outerlane(frame, yellow_regions, min_area):
    outer_points_list = []
    mask, edges = get_mask_and_edge_of_larger_objects(frame, yellow_regions, min_area)

    mask_largest, largest_found = RetLargestContour_OuterLane(mask, min_area)

    if largest_found:
        # Keep only edges of largest region
        edge_largest = cv2.bitwise_and(edges, mask_largest)

        # Return edge points for identifying closest edge later
        lanes_sides_sep, outer_points_list = Ret_LowestEdgePoints(mask_largest)
        edges = edge_largest
    else:
        lanes_sides_sep = np.zeros((frame.shape[0], frame.shape[1]), np.uint8)

    return edges, lanes_sides_sep, outer_points_list

# Function to segment lanes based on color
def segment_lanes(frame, min_area):
    global hls, src
    src = frame.copy()

    # Convert the frame to the HLS color space
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

    # Define HLS color range values for white and yellow regions
    lower_white = np.array([low_H_white, low_L_white, low_S_white])
    upper_white =  np.array([255, 255, 255])

    lower_yellow =  np.array([low_H_yellow, low_L_yellow, low_S_yellow])
    upper_yellow =  np.array([255, 255, 255])

    # Create masks for white and yellow regions
    white_regions = cv2.inRange(hls, lower_white, upper_white)
    yellow_regions = cv2.inRange(hls, lower_yellow, upper_yellow)

    # Display white and yellow regions for debugging
    cv2.imshow("white_regions", white_regions)
    cv2.imshow("yellow_regions", yellow_regions)
    cv2.waitKey(1)

    # Semgneting midlane from white regions
    mid_lane_mask,mid_lane_edge = segment_midlane(frame,white_regions,min_area)

    # Semgneting outerlane from yellow regions
    outer_lane_edge,outerlane_side_sep,outerlane_points = segment_outerlane(frame,yellow_regions,min_area+500)        

    return mid_lane_mask,mid_lane_edge,outer_lane_edge,outerlane_side_sep,outerlane_points
    
