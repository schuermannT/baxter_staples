#!/usr/bin/env python
#-*- coding:utf-8 -*-
import cv2 as cv
import sys
import numpy as np
from matplotlib import pyplot as plot

def create_contour_mask():
    img = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/gzla_2.jpg", 0)
    edges = cv.Canny(img, 10, 100)
    contours = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[1]
    cnt = contours[8]
    rect = cv.minAreaRect(cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    cv.drawContours(img, [box], 0, (0,255,0), 1)
    mask = np.ones(img.shape[:2], np.uint8)
    cv.drawContours(mask, [box], 0, 255, -1)
    return mask

def compare_to_mask(box):
    cmp_img = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/contour-1.jpg", 0)
    print cv.matchShapes(box, cmp_img, 1, 0.0)
    plot.subplot(211), plot.imshow(cmp_img, cmap = "gray"), plot.title("mask"), plot.xticks([]), plot.yticks([])
    plot.subplot(212), plot.imshow(box, cmap = "gray"), plot.title("box"), plot.xticks([]), plot.yticks([])
    plot.show()

def main():
    box = create_contour_mask()
    compare_to_mask(box)


if __name__ == "__main__":
    main()