import os
import re
import cv2
import numpy as np
from tqdm.notebook import tqdm
import matplotlib.pyplot as plt
from PIL import ImageFile
ImageFile.LOAD_TRUNCATED_IMAGES = True

col_frames = os.listdir('recs/')
#col_frames.sort(key=lambda f: int(re.sub('\D', '', f)))

# load frames
col_images = []
for i in tqdm(col_frames):
    img = cv2.imread('recs/'+i)
    col_images.append(img)

# select img
idx = 0

stencil = np.zeros_like(col_images[idx][:,:,0])

# specify coordinates of the polygon
polygon = np.array([[0,720], [500,350], [780,350], [1280,720]])

# fill polygon with ones
cv2.fillConvexPoly(stencil, polygon, 1)

# build img
img = cv2.bitwise_and(col_images[idx][:,:,0], col_images[idx][:,:,0], mask=stencil)

# set threshold
ret, thresh = cv2.threshold(img, 144, 200, cv2.THRESH_BINARY)

lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 50, maxLineGap=100)

# create a copy of the original frame
dmy = col_images[idx].copy()

# draw Hough lines
for line in lines:
  x1, y1, x2, y2 = line[0]
  cv2.line(dmy, (x1, y1), (x2, y2), (0, 0, 255), 3)

# plot frame
plt.figure(figsize=(10,10))
plt.imshow(cv2.cvtColor(dmy, cv2.COLOR_BGR2RGB))
plt.show()