import cv2
import numpy as np
from .utilities import Distance_, Cord_Sort

def IsPathCrossingMid(midlane,mid_cnts,outer_cnts):

    is_Ref_to_path_Left = 0
    Ref_To_CarPath_Image = np.zeros_like(midlane)
    
    Midlane_copy = midlane.copy()

    if not mid_cnts:
        print("[Warning!!!] NO Midlane detected")
    
    Mid_cnts_Rowsorted = Cord_Sort(mid_cnts,"rows")
    Outer_cnts_Rowsorted = Cord_Sort(outer_cnts,"rows")
    Mid_Rows = Mid_cnts_Rowsorted.shape[0]
    Outer_Rows = Outer_cnts_Rowsorted.shape[0]

    Mid_bottom_Pt = Mid_cnts_Rowsorted[Mid_Rows-1,:]
    Outer_bottom_Pt = Outer_cnts_Rowsorted[Outer_Rows-1,:]

    CarTraj_bottom_Pt = ( int( (Mid_bottom_Pt[0] + Outer_bottom_Pt[0]  ) / 2 ) , int( (Mid_bottom_Pt[1]  + Outer_bottom_Pt[1] ) / 2 ) )
    

    cv2.line(Ref_To_CarPath_Image,CarTraj_bottom_Pt,(int(Ref_To_CarPath_Image.shape[1]/2),Ref_To_CarPath_Image.shape[0]),(255,255,0),2)# line from carstart to car path
    cv2.line(Midlane_copy,tuple(Mid_bottom_Pt),(Mid_bottom_Pt[0],Midlane_copy.shape[0]-1),(255,255,0),2)# connecting midlane to bottom
    

    is_Ref_to_path_Left = ( (int(Ref_To_CarPath_Image.shape[1]/2) - CarTraj_bottom_Pt[0]) > 0 )

    if( np.any( (cv2.bitwise_and(Ref_To_CarPath_Image,Midlane_copy) > 0) ) ):
        # Midlane and CarPath Intersets (MidCrossing)
        return True,is_Ref_to_path_Left
    else:
        return False,is_Ref_to_path_Left

def GetYellowInnerEdge(outer_lanes, mid_lane, outer_lane_points):
    offset_correction = 0
    outer_lanes_ret = np.zeros_like(outer_lanes)

    # Extracting Mid and Outerlane contours
    mid_contours = cv2.findContours(mid_lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    outer_contours = cv2.findContours(outer_lanes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    # Check if Outerlane was present initially or not
    if not outer_contours:
        noOuterLane_before = True
    else:
        noOuterLane_before = False

    # Setting the first contour of midlane as reference
    # If mid_contours are present, use the first contour point as ref to find nearest yellow point
    ref = (0,0)
    if (mid_contours):
        ref = tuple(mid_contours[0][0][0])

    if mid_contours:
        # If both midlane and outerlane are detected
        if (len(outer_lane_points) == 2):
            point_a = outer_lane_points[0]
            point_b = outer_lane_points[1]

            closest_index = 0

            # Fetching side of outelane nearest to midlane
            if(Distance_(point_a, ref) <= Distance_(point_b, ref)):
                closest_index = 0
            elif (len(outer_contours) > 1):
                closest_index = 1

            outer_lanes_ret = cv2.drawContours(outer_lanes_ret, outer_contours, closest_index, 255, 1)
            outer_contours_ret = [outer_contours[closest_index]]
    
            # Check if correct outerlane is detected
            isPathCrossing , isCrossingLeft = IsPathCrossingMid(mid_lane,mid_contours,outer_contours_ret)
            if isPathCrossing:
                outer_lanes = np.zeros_like(outer_lanes)
            else:
                return outer_lanes_ret ,outer_contours_ret, mid_contours,0
            
        elif( np.any(outer_lanes > 0) ):
            isPathCrossing , isCrossingLeft = IsPathCrossingMid(mid_lane,mid_contours,outer_contours)
            if isPathCrossing:
                outer_lanes = np.zeros_like(outer_lanes) #Empty outerLane
            else:
                return outer_lanes ,outer_contours, mid_contours,0
            
        # If midlane is present but no outerlane detected
        if (not np.any(outer_lanes > 0)):
            # Fetching the column of the lowest point of the midlane
            mid_contours_rowSorted = Cord_Sort(mid_contours, "rows")
            mid_rows = mid_contours_rowSorted.shape[0]
            mid_lowP = mid_contours_rowSorted[mid_rows-1,:]
            mid_highP = mid_contours_rowSorted[0,:]
            mid_low_col = mid_lowP[0]

            # Addressing wich side to draw the outerlane
            draw_right = False
            if noOuterLane_before:
                if (mid_low_col < int(mid_lane.shape[1]/2)):
                    draw_right = True
            else:
                if isCrossingLeft:
                    draw_right = True

            # Setting outerlane upperand lower points column to the right if draw right and vice versa
            if draw_right:
                low_col=(int(mid_lane.shape[1])-1)
                high_col=(int(mid_lane.shape[1])-1)
                offset_correction = 20
            else:
                low_col=0
                high_col=0
                offset_correction = -20
            
            mid_lowP[1] = mid_lane.shape[0]
            lane_point_lower =  (low_col , int( mid_lowP[1] ) )
            lane_point_top   =  (high_col, int( mid_highP[1]) )
            outer_lanes = cv2.line(outer_lanes,lane_point_lower,lane_point_top,255,1)
            outer_contours = cv2.findContours(outer_lanes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
            return outer_lanes, outer_contours, mid_contours, offset_correction

    else:
        return outer_lanes, outer_contours, mid_contours, offset_correction

def ExtendShortLane(mid_lane, mid_contours, outer_contours, outer_lane):
    # Sorting the Mid and Outer contours on basis of rows
    if (mid_contours and outer_contours):
        mid_contours_rowSorted = Cord_Sort(mid_contours, "rows")
        outer_contours_rowSorted = Cord_Sort(outer_contours, "rows")
        image_bottom = mid_lane.shape[0]
        number_of_contours_midlane = mid_contours_rowSorted.shape[0]
        number_of_contours_outerlane = outer_contours_rowSorted.shape[0]

        bottom_point_mid = mid_contours_rowSorted[number_of_contours_midlane-1,:]
        if (bottom_point_mid[1] < image_bottom):
           mid_lane = cv2.line(mid_lane,tuple(bottom_point_mid),(bottom_point_mid[0],image_bottom),255,2)

        # Connect Outerlane to image bottom
        # Extend Outerlane in the direction of its slope - Taking last 20 points to estimate slope
        bottom_point_outer = outer_contours_rowSorted[number_of_contours_outerlane-1,:]
        if (bottom_point_outer[1] < image_bottom):
            if(number_of_contours_outerlane>20):
                shift=20
            else:
                shift=2
            refLast10Points = outer_contours_rowSorted[number_of_contours_outerlane-shift:number_of_contours_outerlane-1:2,:]

            # Estimating Slope
            if(len(refLast10Points)>1):# Atleast 2 points needed to estimate a line
                ref_x = refLast10Points[:,0]#cols
                ref_y = refLast10Points[:,1]#rows
                ref_parameters = np.polyfit(ref_x, ref_y, 1)
                ref_slope = ref_parameters[0]
                ref_y_intercept = ref_parameters[1]

                # Extending outerlane in the direction of its slope
                if(ref_slope < 0):
                    ref_LineTouchPoint_col = 0
                    ref_LineTouchPoint_row = ref_y_intercept
                else:
                    ref_LineTouchPoint_col = outer_lane.shape[1]-1 # Cols have lenth of ColLength But traversal is from 0 to ColLength-1
                    ref_LineTouchPoint_row = ref_slope * ref_LineTouchPoint_col + ref_y_intercept
                ref_TouchPoint = (ref_LineTouchPoint_col,int(ref_LineTouchPoint_row))#(col ,row)
                ref_BottomPoint_tup = tuple(bottom_point_outer)
                outer_lane = cv2.line(outer_lane,ref_TouchPoint,ref_BottomPoint_tup,255,2)

                # If required, connect outerlane to bottom by drawing a vertical line
                if(ref_LineTouchPoint_row < image_bottom):
                    ref_TouchPoint_Ref = (ref_LineTouchPoint_col,image_bottom)
                    outer_lane = cv2.line(outer_lane,ref_TouchPoint,ref_TouchPoint_Ref,255,3)

    return mid_lane,outer_lane
