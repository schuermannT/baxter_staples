#!/usr/bin/env python
#-*- coding:utf-8 -*-
import cv2 as cv
import sys
import numpy as np
from matplotlib import pyplot as plot
from copy import deepcopy

def create_box_mask(contour, image_shape):
    mask = np.ones(image_shape, np.uint8)
    rect = cv.minAreaRect(contour)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    cv.drawContours(mask, [box], 0, 255, -1)
    return mask

def find_match(img, comparator):
    edges = cv.Canny(img, 10, 100)
    contours = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[1]
    best_match = 1.0
    for c in contours:
        work_mask = create_box_mask(c, img.shape)
        match = cv.matchShapes(comparator, work_mask, 1, 0.0)
        if match < best_match:
            best_match = match
            best_mask = deepcopy(work_mask)
            best_contour = deepcopy(c)
    print("deviation for best match at: {}%".format(float(best_match)*100))
    return best_contour, best_mask

def detect_staple(img):
    cmp_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/masks/staple1.jpg", 0)
    found_match = find_match(img, cmp_mask)
    img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    cv.drawContours(img, found_match, 0, (0,255,0), 2)
    plot.subplot(211), plot.imshow(cmp_mask, cmap = "gray"), plot.title("comparator"), plot.xticks([]), plot.yticks([])
    plot.subplot(212), plot.imshow(img), plot.title("found staple"), plot.xticks([]), plot.yticks([])
    plot.show()
    #cv.imwrite("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/contour_rota05.jpg", best_mask)

def detect_paper(img):
    #substract background
    sublayer_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/masks/background.jpg", 0)
    sublayer_mask = cv.resize(sublayer_mask, img.shape[1::-1])
    paper_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/masks/document.jpg", 0)
    paper_mask = cv.resize(paper_mask, img.shape[1::-1])
    erode_kernel = np.ones((11,11), np.uint8)
    sublayer_mask = cv.erode(sublayer_mask, erode_kernel, iterations=1)
    minus_background = cv.bitwise_and(img, sublayer_mask)
    paper_cnt, paper_mask = find_match(img, paper_mask)
    (x,y),(MA,ma),angle = cv.fitEllipse(paper_cnt)
    print("MA: {} ma: {} angle: {}".format(MA, ma, angle))
    show_img = cv.cvtColor(minus_background, cv.COLOR_GRAY2BGR)
    cv.drawContours(show_img, paper_cnt, 0, (0,255,0), 0)
    minus_sublayer = cv.bitwise_and(img, paper_mask)
    rim = MA*0.0095

    plot.subplot(411), plot.imshow(img, cmap = "gray"), plot.title("image"), plot.xticks([]), plot.yticks([])
    plot.subplot(412), plot.imshow(sublayer_mask, cmap = "gray"), plot.title("substracted background"), plot.xticks([]), plot.yticks([])
    plot.subplot(413), plot.imshow(show_img, cmap = "gray"), plot.title("paper_detected"), plot.xticks([]), plot.yticks([])
    plot.subplot(414), plot.imshow(minus_sublayer, cmap = "gray"), plot.title("substracted sublayer"), plot.xticks([]), plot.yticks([])
    plot.show()

def main():
    img = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/paper640_5.jpg", 0)
    detect_paper(img)


if __name__ == "__main__":
    main()