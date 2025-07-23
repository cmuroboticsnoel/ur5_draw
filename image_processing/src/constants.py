import os

#------------------------------------------------
# GLOBAL CONSTANTS
#------------------------------------------------

IMAGES_PATH=os.path.expanduser('~/ur5_draw/image_processing/images')
IMAGE_DESCRIPTION_PATH=os.path.expanduser('~/ur5_draw/ur5_draw_ws/src/ur5_drawing/image_description')
# Create debug directory if it doesn't exist
DEBUG_PATH=os.path.expanduser('~/ur5_draw/image_processing/debug')
if not os.path.exists(DEBUG_PATH):
    os.makedirs(DEBUG_PATH)
DEBUG_COUNTER=[0]