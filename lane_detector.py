import os
import re
import cv2
import numpy as np
from PIL import ImageFile
ImageFile.LOAD_TRUNCATED_IMAGES = True

from matplotlib import pyplot as plt
from math import acos


def detect():
    #change to the name of the image you want to work with
    name = 90810
    img = cv2.imread('./camera_samples/%d.png'%name)
    dmy = img.copy()

    edges = cv2.Canny(img,100,200)
    cv2.imwrite('./camera_samples/%d_edge.png'%name, edges)

    stencil = np.zeros_like(edges)

    max_lines=0

    #polygons = [np.array([[0,720], [0,600], [600,350], [900,350], [1280,600], [1280,720]]),
    #            np.array([[0,720], [0,600], [640,350], [760,350], [1280,600], [1280,720]]),
    #            np.array([[0,720], [0,600], [420,350], [720,350], [1280,600], [1280,720]])]
    polygon = np.array([[0,720], [0,600], [600,350], [680,350], [1280,600], [1280,720]])

    #for polygon in polygons:
    cv2.fillConvexPoly(stencil, polygon, 1)

    edges = cv2.bitwise_and(edges, edges, mask=stencil)

    cv2.imwrite('./camera_samples/%d_stencil.png'%name, edges)

    def_lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=100)
    #n=0

    #for line in lines:
    #    n += 1
    #if n>max_lines:
    #    max_lines=n
    #    def_lines=lines

    left_points=[]
    right_points=[]
    for line in def_lines:
        left = False
        aux=[]
        x1, y1, x2, y2 = line[0]
        if y2<y1:
            left = True
            cy = y1
            cx = x1
            y1 = y2
            x1 = x2
            y2 = cy
            x2 = cx
        for i in range(y1,y2+1):
            if left:
                aux.append(i)
                extra = [x for x in aux if x not in left_points]
                left_points.extend(extra)
            else:
                aux.append(i)
                extra = [x for x in aux if x not in right_points]
                right_points.extend(extra)
        cv2.line(dmy, (x1, y1), (x2, y2), (0, 0, 255), 3)

    cv2.imwrite('./camera_samples/%d_hough.png'%name, dmy)

    comm_points = [x for x in left_points if x in right_points]
    comm_points.sort()
    x = 350
    best_fit = min(comm_points, key=lambda n:abs(n-x))
    found_left=False
    found_right=False
    xl = xr = 640
    for line in def_lines:
        left = False
        aux=[]
        x1, y1, x2, y2 = line[0]
        if y2<y1:
            left = True
            cy = y1
            cx = x1
            y1 = y2
            x1 = x2
            y2 = cy
            x2 = cx

        if y1<best_fit and y2>best_fit:
            if x1 <= 639 and x2 <= 639 and not found_left:
                xl =  int((best_fit-y1)*(x2-x1)/(y2-y1)+x1)
            elif x1 > 639 and x2 > 639 and not found_right:
                xr = int((best_fit-y1)*(x2-x1)/(y2-y1)+x1)
        if y1==best_fit or y2==best_fit:
            if left:
                xl =x1 if y1==best_fit else x2
                found_left=True
            else:
                xr = x1 if y1==best_fit else x2
                found_right=True
    lane_center=int((xl+xr)/2)
    cv2.line(dmy, (xl, best_fit), (xr, best_fit), (255,0,0), 3)
    cv2.line(dmy, (640, 350), (640, 720), (0,255,0), 2)
    cv2.line(dmy, (lane_center, best_fit), (640, 720), (0,255,0), 2)
    
    l = 720-best_fit
    c = lane_center-640
    ip = (l**2 + abs(c)**2)**0.5
    cos_alpha = l/ip
    
    cv2.imwrite('./camera_samples/%d_output.png'%name, dmy)

    plt.subplot(121)
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB),cmap = 'gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122)
    plt.imshow(cv2.cvtColor(dmy, cv2.COLOR_BGR2RGB),cmap = 'gray')
    plt.title('Elaborated Image'), plt.xticks([]), plt.yticks([])
    plt.show()

    return dmy, c

if __name__ == '__main__':

    try:
        detect()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')