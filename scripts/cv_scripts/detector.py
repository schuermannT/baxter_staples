#!/usr/bin/env python
#-*- coding:utf-8 -*-
import cv2 as cv
import sys
import numpy as np
from matplotlib import pyplot as plot
from copy import deepcopy

verbose = False

def create_rim(contour, mask, rim_width):
    (x,y),(MA,ma),angle = cv.fitEllipse(contour)
    if verbose:
        print("MA: {} ma: {} angle: {}".format(MA, ma, angle))
    rim_width_y = MA*(rim_width[0]/210.0) #rim width [mm] in y divided by DIN A4 width [mm]
    rim_width_x = ma*(rim_width[1]/297.0) #rim width [mm] in x divided by DIN A4 length [mm]
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
    box = create_box_contour(contour)
    cv.drawContours(mask, [box], 0, 255, -1)
    return mask

def create_box_contour(contour):
    rect = cv.minAreaRect(contour)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    return box

def find_match(img, comparator, maxL=1280.0, minL=0.0):
    if len(img.shape) > 2:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(img, 10, 100)
    contours = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[1]
    best_match = 1.0
    best_match = None
    best_mask = None
    best_contour = None
    for c in contours:
        work_mask = create_box_mask(c, img.shape)
        match = cv.matchShapes(comparator, work_mask, 1, 0.0)
        arcL = cv.arcLength(create_box_contour(c), True)
        if arcL < maxL and arcL > minL and match < best_match:
            if verbose:
                print(arcL)
            best_match = match
            best_mask = deepcopy(work_mask)
            best_contour = deepcopy(c)
            if verbose:
                if len(contours) < 50:
                    cv.destroyAllWindows()
                    show_img = deepcopy(img)
                    show_img = cv.cvtColor(show_img, cv.COLOR_GRAY2BGR)
                    cv.drawContours(show_img, [c], 0, (0,255,0), 1)
                    """ cv.imshow("{}".format(match), show_img)
                    if cv.waitKey(0) == 119:
                        cv.imwrite("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/masks/small_staple.jpg", best_mask)    """             
                    plot.subplot(311), plot.imshow(comparator, cmap = "gray"), plot.title("comparator"), plot.xticks([]), plot.yticks([])
                    plot.subplot(312), plot.imshow(work_mask, cmap = "gray"), plot.title("found mask"), plot.xticks([]), plot.yticks([])
                    plot.subplot(313), plot.imshow(show_img), plot.title("found contour: {}".format(match)), plot.xticks([]), plot.yticks([])
                    plot.show()
    if verbose:
        print("deviation for best match at: {}%".format(float(best_match)*100))
    return best_contour, best_mask

def detect_staple(img):
    if len(img.shape) > 2:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    cmp_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/masks/small_staple.jpg", 0)
    found_match = find_match(img, cmp_mask, 50.0, 20.0)[0]
    if found_match is None:
        print("cannot find staple...")
        return (0,0)
    img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    cv.drawContours(img, found_match, -1, (0,255,0), 2)
    plot.subplot(211), plot.imshow(cmp_mask, cmap = "gray"), plot.title("comparator"), plot.xticks([]), plot.yticks([])
    plot.subplot(212), plot.imshow(img), plot.title("found staple"), plot.xticks([]), plot.yticks([])
    plot.show()
    return found_match

def detect_paper(img):
    #substract background
    if len(img.shape) > 2:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    sublayer_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/masks/background.jpg", 0)
    sublayer_mask = cv.resize(sublayer_mask, img.shape[1::-1])
    paper_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/masks/document.jpg", 0)
    paper_mask = cv.resize(paper_mask, img.shape[1::-1])
    erode_kernel = np.ones((11,13), np.uint8)
    sublayer_mask = cv.erode(sublayer_mask, erode_kernel, iterations=1)
    minus_background = cv.bitwise_and(img, sublayer_mask)
    paper_cnt, paper_mask = find_match(img, paper_mask)
    if paper_cnt is None:
        print("cannot find document... please rearrange in workspace")
        return img
    minus_sublayer = cv.bitwise_and(img, paper_mask)
    rim_mask = create_rim(paper_cnt, paper_mask, (20.0, 25.0))[1]
    only_rim = cv.bitwise_and(img, rim_mask)
    if verbose:
        plot.subplot(411), plot.imshow(img, cmap = "gray"), plot.title("image"), plot.xticks([]), plot.yticks([])
        plot.subplot(412), plot.imshow(minus_background, cmap = "gray"), plot.title("substracted background"), plot.xticks([]), plot.yticks([])
        plot.subplot(413), plot.imshow(minus_sublayer, cmap = "gray"), plot.title("substracted sublayer"), plot.xticks([]), plot.yticks([])
        plot.subplot(414), plot.imshow(only_rim, cmap = "gray"), plot.title("only paper rim"), plot.xticks([]), plot.yticks([])
        plot.show()
    return only_rim


def main():
    img = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/paper640_5.jpg", 0)
    only_rim = detect_paper(img)
    detect_staple(only_rim)


if __name__ == "__main__":
    main()