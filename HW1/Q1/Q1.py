import cv2
import numpy as np


path = '/home/rushi/College/Fall_2022/Perception/HW1/Q1/for_watson.png'

img = cv2.imread(path)

img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

unique_pts = np.unique(img_gray)


temp_imp = img_gray 

for i in unique_pts:
    ret, thresh_img = cv2.threshold(img_gray, i, 255, cv2.THRESH_BINARY)
    temp_imp = cv2.bitwise_xor(thresh_img, temp_imp, mask = None)

cv2.imshow('grayscale', temp_imp)
# cv2.imwrite("/home/rushi/College/Fall_2022/Perception/HW1/Q1/decoded.png", temp_imp)
cv2.waitKey()

if cv2.waitKey(20) & 0xFF == 27:
    cv2.destroyAllWindows()

