#!/usr/bin/env python
#-*- coding:utf-8 -*-
import cv2 as cv
import sys
import numpy as np
from matplotlib import pyplot as plot

def create_contour_mask(img):
    edges = cv.Canny(img, 10, 100)
    contours = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[1]
    cnt = contours[8]
    rect = cv.minAreaRect(cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    mask = np.ones(img.shape[:2], np.uint8)
    cv.drawContours(mask, [box], 0, 255, -1)
    return mask

def compare_to_staple(input_mask):
    cmp_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/contour-1.jpg", 0)
    print cv.matchShapes(cmp_mask, input_mask, 1, 0.0)
    plot.subplot(211), plot.imshow(cmp_mask, cmap = "gray"), plot.title("comparator"), plot.xticks([]), plot.yticks([])
    plot.subplot(212), plot.imshow(input_mask, cmap = "gray"), plot.title("input"), plot.xticks([]), plot.yticks([])
    plot.show()

def main():
    img = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/gzla_2.jpg", 0)
    mask = create_contour_mask(img)
    compare_to_staple(mask)


if __name__ == "__main__":
    main()