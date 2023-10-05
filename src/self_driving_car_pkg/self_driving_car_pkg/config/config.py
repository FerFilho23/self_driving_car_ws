#============================================ Paramters for Lane Detection =======================================
Resized_width = 320#320#240#640#320 # Control Parameter
Resized_height = 240#240#180#480#240

Ref_imgWidth = 1920
Ref_imgHeight = 1080

Frame_pixels = Ref_imgWidth * Ref_imgHeight

Resize_Framepixels = Resized_width * Resized_height

Lane_Extraction_minArea_per = 1000 / Frame_pixels
minArea_resized = int(Resize_Framepixels * Lane_Extraction_minArea_per)

BWContourOpen_speed_MaxDist_per = 600 / Ref_imgHeight
MaxDist_resized = int(Resized_height * BWContourOpen_speed_MaxDist_per)

CropHeight = 630 # Required in Camera mounted on top of car 640p
CropHeight_resized = int( (CropHeight / Ref_imgHeight ) * Resized_height )