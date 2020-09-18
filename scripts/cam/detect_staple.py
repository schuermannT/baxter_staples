#!/usr/bin/env python
#-*- coding:utf-8 -*-
import cv2 as cv
import sys
import numpy as np
from matplotlib import pyplot as plot
from copy import deepcopy

def compare_to_mask(img):
    cmp_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/contour-1.jpg", 0)
    edges = cv.Canny(img, 10, 100)
    contours = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[1]
    best_match = 1.0
    for c in contours:
        work_mask = np.ones(img.shape[:2], np.uint8)
        rect = cv.minAreaRect(c)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        cv.drawContours(work_mask, [box], 0, 255, -1)
        match = cv.matchShapes(cmp_mask, work_mask, 1, 0.0)
        if match < best_match:
            best_match = match
            best_mask = deepcopy(work_mask)
    plot.subplot(211), plot.imshow(cmp_mask, cmap = "gray"), plot.title("comparator"), plot.xticks([]), plot.yticks([])
    plot.subplot(212), plot.imshow(best_mask, cmap = "gray"), plot.title(best_match), plot.xticks([]), plot.yticks([])
    plot.show()
    cv.imwrite("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/contour_rota05.jpg", best_mask)

def main():
    img = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/rot_05.jpg", 0)
    compare_to_mask(img)


if __name__ == "__main__":
    main()