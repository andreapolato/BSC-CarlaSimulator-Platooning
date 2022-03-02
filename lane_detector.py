import os
import re
import cv2
import numpy as np
from PIL import ImageFile
ImageFile.LOAD_TRUNCATED_IMAGES = True


def detect(img):
    # create a copy of the original frame
    ofc = img.copy()
    
    # setup the region of interest
    polygon = np.array([[0,720], [500,350], [780,350], [1280,720]])

    # fill polygon with ones
    stencil = stencil = np.zeros_like(img[:,:,0])
    cv2.fillConvexPoly(stencil, polygon, 1)

    # build img
    img = cv2.bitwise_and(img[:,:,0], img[:,:,0], mask=stencil)

    # apply threshold
    ret, thresh = cv2.threshold(img, 144, 200, cv2.THRESH_BINARY)
    lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 50, maxLineGap=100)

    # draw Hough lines
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(ofc, (x1, y1), (x2, y2), (0, 0, 255), 3)

    return cv2.cvtColor(ofc, cv2.COLOR_BGR2RGB)
