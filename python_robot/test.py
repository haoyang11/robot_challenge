import cv2
import os
import matplotlib.pyplot as plt
from PIL  import Image

filaname="Image.jpg"
img=Image.open(filaname)
# cv2.imshow("frame:",img)
plt.show(img)
raw_input()