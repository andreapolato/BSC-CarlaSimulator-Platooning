import numpy as np
import cv2
from matplotlib import pyplot as plt
import os

img = cv2.imread('./recs/91502.png')
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
print("common points: "+ str(comm_points))
plt.subplot(121)
plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(dmy,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.show()