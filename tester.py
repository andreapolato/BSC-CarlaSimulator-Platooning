from cmath import acos, cos, sqrt
import numpy as np
import cv2
from matplotlib import pyplot as plt
import math
import os

img = cv2.imread('./recs/1857.png')
edges = cv2.Canny(img,100,200)

stencil = np.zeros_like(edges)

max_lines=0

# specify coordinates of the polygon
polygons = [np.array([[0,720], [0,600], [600,350], [900,350], [1280,600], [1280,720]]),np.array([[0,720], [0,600], [640,350], [760,350], [1280,600], [1280,720]]),np.array([[0,720], [0,600], [420,350], [720,350], [1280,600], [1280,720]])]
#polygon = np.array([[0,720], [0,600], [600,350], [680,350], [1280,600], [1280,720]])

# fill polygon with ones
for polygon in polygons:
    cv2.fillConvexPoly(stencil, polygon, 1)

    # build img
    edges = cv2.bitwise_and(edges, edges, mask=stencil)

    ret, thresh = cv2.threshold(edges, 144, 200, cv2.THRESH_BINARY)
    lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 50, maxLineGap=100)
    n=0

    for line in lines:
        n += 1
    if n>max_lines:
        max_lines=n
        def_lines=lines
dmy = img.copy()


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

comm_points = [x for x in left_points if x in right_points]
comm_points.sort()
x = 420
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
      print("FOUND POTENTIAL LEFT LINE: " +str([x1,y1,x2,y2])+"\nxl= %d"%xl)
    elif x1 > 639 and x2 > 639 and not found_right:
      xr = int((best_fit-y1)*(x2-x1)/(y2-y1)+x1)
      print("FOUND POTENTIAL RIGHT LINE: " +str([x1,y1,x2,y2])+"\nxr= %d"%xr)
  if y1==best_fit or y2==best_fit:
    if left:
      print("FOUND LEFT EDGE with x = %d"%(x1 if y1==best_fit else x2))
      xl =x1 if y1==best_fit else x2
      found_left=True
    else:
      print("FOUND RIGHT EDGE with x = %d"%(x1 if y1==best_fit else x2))
      xr = x1 if y1==best_fit else x2
      found_right=True
lane_center=int((xl+xr)/2)
print(xl,xr,best_fit,lane_center)
cv2.line(dmy, (xl, best_fit), (xr, best_fit), (255,0,0), 3)
cv2.line(dmy, (640, 350), (640, 720), (0,255,0), 2)
cv2.line(dmy, (lane_center, best_fit), (640, 720), (0,255,0), 2)
l = 720-best_fit
c = abs(640-lane_center)
print(l**2 + c**2)
ip = (l**2 + c**2)**0.5
cos_alpha = l/ip
print(l,ip,acos(cos_alpha))
print(ip*cos_alpha)
plt.subplot(121)
plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(dmy,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.show()