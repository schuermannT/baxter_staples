#!/usr/bin/env python
#-*- coding:utf-8 -*-
import cv2 as cv
import sys
import numpy as np
from matplotlib import pyplot as plot



def main():
    #calculations
    kernel = np.ones((5,1), np.uint8)
    img = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/gzla_3.jpg")
    if img is None:
        sys.exit("can't load image")
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    #blur = cv.bilateralFilter(img, 7, 30, 30)
    blur = cv.GaussianBlur(gray, (5, 3), 0)
    aThresh = cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2)
    morph = cv.morphologyEx(aThresh, cv.MORPH_OPEN, kernel)
    canny = cv.Canny(gray, 1, 100)
    contours = cv.findContours(canny.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[1]
    #print(len(contours))
    cnt = contours[8]
    rect = cv.minAreaRect(cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    cv.drawContours(img, [box], 0, (0,255,0), 1)
    #maske der kontur erstellen und speichern
    mask = np.ones(img.shape[:2], np.uint8)
    cv.drawContours(mask, [box], 0, 255, 0)
    cv.imwrite("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/contour0.jpg", mask)

    #show in plot
    plot.subplot(611), plot.imshow(img), plot.title("original"), plot.xticks([]), plot.yticks([])
    plot.subplot(612), plot.imshow(blur, cmap = "gray"), plot.title("blurred"), plot.xticks([]), plot.yticks([])
    plot.subplot(613), plot.imshow(aThresh, cmap = "gray"), plot.title("gauss adaptive threshold after blur"), plot.xticks([]), plot.yticks([])
    plot.subplot(614), plot.imshow(morph, cmap = "gray"), plot.title("opening after gat after blur"), plot.xticks([]), plot.yticks([])
    plot.subplot(615), plot.imshow(canny, cmap = "gray"), plot.title("canny edge detection"), plot.xticks([]), plot.yticks([])
    plot.subplot(616), plot.imshow(mask, cmap = "gray"), plot.title("mask"), plot.xticks([]), plot.yticks([])
    plot.show()
    #cv.waitKey(0)


if __name__ == "__main__":
    main()


    