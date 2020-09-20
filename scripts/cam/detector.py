#!/usr/bin/env python
#-*- coding:utf-8 -*-
import cv2 as cv
import sys
import numpy as np
from matplotlib import pyplot as plot
from copy import deepcopy

def create_rim(contour, mask):
    (x,y),(MA,ma),angle = cv.fitEllipse(contour)
    print("MA: {} ma: {} angle: {}".format(MA, ma, angle))
    rim_width_y = MA*(20.0/210.0) #rim width [mm] in y divided by DIN A4 width [mm]
    rim_width_x = ma*(25.0/297.0) #rim width [mm] in x divided by DIN A4 length [mm]
    rect = cv.minAreaRect(contour)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    work_angle = (angle - 90.0)*np.pi/180
    rim_box = deepcopy(box)
    if work_angle <= 0.0:
        rim_box[0][0] = box[0][0] + (rim_width_x*np.cos(work_angle)) #bottom left x
        rim_box[0][1] = box[0][1] - (rim_width_y*np.cos(work_angle)) #bottom left y
        rim_box[1][0] = box[1][0] + (rim_width_x*np.cos(work_angle)) #top left x
        rim_box[1][1] = box[1][1] + (rim_width_y*np.cos(work_angle)) #top left y
        rim_box[2][0] = box[2][0] - (rim_width_x*np.cos(work_angle)) #top right x
        rim_box[2][1] = box[2][1] + (rim_width_y*np.cos(work_angle)) #top right y
        rim_box[3][0] = box[3][0] - (rim_width_x*np.cos(work_angle)) #bottom right x
        rim_box[3][1] = box[3][1] - (rim_width_y*np.cos(work_angle)) #bottom right y
    elif work_angle > 0.0:
        rim_box[0][0] = box[0][0] - (rim_width_x*np.cos(work_angle)) #bottom right x
        rim_box[0][1] = box[0][1] - (rim_width_y*np.cos(work_angle)) #bottom right y
        rim_box[1][0] = box[1][0] + (rim_width_x*np.cos(work_angle)) #bottom left x
        rim_box[1][1] = box[1][1] - (rim_width_y*np.cos(work_angle)) #bottom left y
        rim_box[2][0] = box[2][0] + (rim_width_x*np.cos(work_angle)) #top left x
        rim_box[2][1] = box[2][1] + (rim_width_y*np.cos(work_angle)) #top left y
        rim_box[3][0] = box[3][0] - (rim_width_x*np.cos(work_angle)) #top right x
        rim_box[3][1] = box[3][1] + (rim_width_y*np.cos(work_angle)) #top right y     
    cv.drawContours(mask, [rim_box], 0, 0, -1)
    return rim_box, mask

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
    minus_sublayer = cv.bitwise_and(img, paper_mask)
    rim_mask = create_rim(paper_cnt, paper_mask)[1]
    only_rim = cv.bitwise_and(img, rim_mask)
    show_img = cv.cvtColor(minus_sublayer, cv.COLOR_GRAY2BGR)
    
    plot.subplot(411), plot.imshow(img, cmap = "gray"), plot.title("image"), plot.xticks([]), plot.yticks([])
    plot.subplot(412), plot.imshow(minus_background, cmap = "gray"), plot.title("substracted background"), plot.xticks([]), plot.yticks([])
    plot.subplot(413), plot.imshow(minus_sublayer, cmap = "gray"), plot.title("substracted sublayer"), plot.xticks([]), plot.yticks([])
    plot.subplot(414), plot.imshow(only_rim, cmap = "gray"), plot.title("only paper rim"), plot.xticks([]), plot.yticks([])
    plot.show()


def main():
    img = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/paper640_5.jpg", 0)
    detect_paper(img)


if __name__ == "__main__":
    main()