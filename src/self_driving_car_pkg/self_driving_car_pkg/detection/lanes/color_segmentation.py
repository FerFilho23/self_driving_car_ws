import cv2
import numpy as np

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

# Function to segment lanes based on color
def segment_lanes(frame, min_area):
    global hls, src
    src = frame.copy()

    # Convert the frame to the HLS color space
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

    # Define HLS color range values for white and yellow regions
    lower_white = (low_H_white, low_L_white, low_S_white)
    upper_white = (255, 255, 255)

    lower_yellow = (low_H_yellow, low_L_yellow, low_S_yellow)
    upper_yellow = (255, 255, 255)

    # Create masks for white and yellow regions
    white_regions = cv2.inRange(hls, lower_white, upper_white)
    yellow_regions = cv2.inRange(hls, lower_yellow, upper_yellow)

    # Display white and yellow regions for debugging
    cv2.imshow("white_regions", white_regions)
    cv2.imshow("yellow_regions", yellow_regions)
    cv2.waitKey(1)
