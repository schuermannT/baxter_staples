#!/usr/bin/env python
#-*- coding:utf-8 -*-
import cv2 as cv
import sys
import numpy as np
import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plot
from copy import deepcopy

verbose = False

def create_rim(contour, mask, rim_width):
    rect = cv.minAreaRect(contour)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    main_length, sec_length, angle, mainline_endpoint = get_box_info(box)
    if main_length == 0:
        return False, None
    if verbose:
        print("main_length: {} sec_length: {} angle: {}".format(main_length, sec_length, angle))
    rim_corner_length = np.sqrt(np.square(main_length*(rim_width[1]/297.0)) + np.square(sec_length*(rim_width[0]/210.0)))
    rim_box = deepcopy(box)
    if mainline_endpoint == 3 :
        angle += (np.pi/4)
        cols = (rim_corner_length*np.sin(angle))
        rows = (rim_corner_length*np.cos(angle))
        rim_box[0][0] = box[0][0] + cols #bottom left col
        rim_box[0][1] = box[0][1] - rows #bottom left row
        rim_box[1][0] = box[1][0] + cols #top left col
        rim_box[1][1] = box[1][1] + rows #top left colrow
        rim_box[2][0] = box[2][0] - cols #top right col
        rim_box[2][1] = box[2][1] + rows #top right row
        rim_box[3][0] = box[3][0] - cols #bottom right col
        rim_box[3][1] = box[3][1] - rows #bottom right row
    else:
        angle -= (np.pi/4)
        cols = (rim_corner_length*np.cos(angle))
        rows = (rim_corner_length*np.sin(angle))
        rim_box[0][0] = box[0][0] - cols #bottom right col
        rim_box[0][1] = box[0][1] - rows #bottom right row
        rim_box[1][0] = box[1][0] + cols #bottom left col
        rim_box[1][1] = box[1][1] - rows #bottom left row
        rim_box[2][0] = box[2][0] + cols #top left col
        rim_box[2][1] = box[2][1] + rows #top left row
        rim_box[3][0] = box[3][0] - cols #top right col
        rim_box[3][1] = box[3][1] + rows #top right row
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

def get_box_info(box_cnt):
    l03 = np.sqrt(np.square(box_cnt[3][0] - box_cnt[0][0]) + np.square(box_cnt[3][1] - box_cnt[0][1]))
    l01 = np.sqrt(np.square(box_cnt[1][0] - box_cnt[0][0]) + np.square(box_cnt[1][1] - box_cnt[0][1]))
    ankathete = float(box_cnt[3][1] - box_cnt[0][1])
    if ankathete == 0:
        return 0, None, None, None
    gkdak = float(box_cnt[3][0] - box_cnt[0][0]) / ankathete
    angle03 = np.arctan(gkdak)+(np.pi/2)
    if l03 > l01 :
        main_length = l03
        secondary_length = l01
        angle = angle03
        mainline_endpoint = 3
    else:
        main_length = l01
        secondary_length = l03
        angle = angle03
        mainline_endpoint = 1
    return main_length, secondary_length, angle, mainline_endpoint


def find_match(img, comparator, maxL=1280.0, minL=0.0):
    if len(img.shape) > 2:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(img, 10, 100)
    contours = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[1]
    best_match = 1.0
    best_contour = None
    i = 0
    for c in contours:
        work_mask = create_box_mask(c, img.shape)
        match = cv.matchShapes(comparator, work_mask, 1, 0.0)
        arcL = cv.arcLength(create_box_contour(c), True)
        if arcL < maxL and arcL > minL and match < best_match:
            i+=1
            if verbose:
                print(arcL)
            best_match = match
            best_mask = deepcopy(work_mask)
            best_contour = create_box_contour(c)
        if verbose and img.shape[0] == 200:
            cv.destroyAllWindows()
            show_img = deepcopy(img)
            show_img = cv.cvtColor(show_img, cv.COLOR_GRAY2BGR)
            cv.drawContours(show_img, [best_contour], 0, (0,255,0), 1)
            """ cv.imshow("{}".format(match), show_img)
            if cv.waitKey(0) == 119:
                cv.imwrite("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/masks/small_staple.jpg", best_mask)    """             
            plot.subplot(311), plot.imshow(comparator, cmap = "gray"), plot.title("comparator"), plot.xticks([]), plot.yticks([])
            plot.subplot(312), plot.imshow(work_mask, cmap = "gray"), plot.title("found mask"), plot.xticks([]), plot.yticks([])
            plot.subplot(313), plot.imshow(show_img), plot.title("found contour: {}".format(match)), plot.xticks([]), plot.yticks([])
            plot.show()
    if verbose:
        print("deviation for best match at: {}%\nnumber of best matches: {}".format(float(best_match)*100, i))
    if not best_contour is None:
        show_img = cv.cvtColor(deepcopy(img), cv.COLOR_GRAY2BGR)
        cv.drawContours(show_img, [best_contour], 0, (0,255,0), 1)
        if verbose:
            plot.subplot(111), plot.imshow(show_img), plot.title("found contour: {}".format(match)), plot.xticks([]), plot.yticks([])
            plot.show()
        return True, best_contour, best_mask, show_img
    else:
        return False, None, None, None

def sortkey_first(x):
    return x[0]

def contour_image(cnt, img):
    if len(img.shape) <= 2:
        img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    return cv.drawContours(img, [cnt], 0, (0,255,0), 1)

def find_matches(img, comparator, maxL=1280.0, minL=0.0):
    if len(img.shape) > 2:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(img, 10, 100)
    contours = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[1]
    match_thresh = 0.15
    best_matches = list()
    for c in contours:
        work_mask = create_box_mask(c, img.shape)
        match = cv.matchShapes(comparator, work_mask, 1, 0.0)
        arcL = cv.arcLength(create_box_contour(c), True)
        cnt_img = contour_image(c, deepcopy(img))
        if arcL < maxL and arcL > minL and match < match_thresh:
            if verbose:
                print(arcL)
            best_matches.append([match,create_box_contour(c), deepcopy(work_mask), deepcopy(cnt_img)])
    best_matches.sort(key=sortkey_first)
    #if verbose:
    print("max deviation for matches at: {}%\n\n".format(match_thresh*100))
    print("number of matches: {}".format(len(best_matches)))
    print("deviation for best match at {}%".format(best_matches[0][0]))
    if len(best_matches) > 0:
        return True, best_matches
    else:
        return False, best_matches

def detect_staple(img):
    if len(img.shape) > 2:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    if img.shape[0] == 400:
        cmp_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/masks/small_staple.jpg", 0)
        maxL = 50.0
        minL = 20.0
        success, found_matches = find_matches(img, cmp_mask, maxL=maxL, minL=minL)
        if not success:
            print("cannot find any staple")
            return False, img, None
        return success, found_matches
    elif img.shape[0] == 200:
        cmp_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/masks/staple1.jpg", 0)
        maxL = 120.0
        minL = 60.0
        success, found_contour, best_mask, cnt_img = find_match(img, cmp_mask, maxL=maxL, minL=minL)
        if not success:
            print("cannot find any staple")
            return False, img, None
        return success, found_contour, cnt_img
    else:
        print("given image has wrong resolution. Please provide a picture with following format: 640x400")
        return False, img, None


def detect_paper(img):
    if len(img.shape) > 2:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    if img.shape[0] != 400:
        print("given image has wrong resolution. Please provide a picture with following format: 640x400")
        return False, img
    #get background mask
    workplate_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/masks/background.jpg", 0)
    workplate_mask = cv.resize(workplate_mask, img.shape[1::-1])
    #get workplate mask
    paper_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/masks/document.jpg", 0)
    paper_mask = cv.resize(paper_mask, img.shape[1::-1])
    #subtract background
    erode_kernel = np.ones((11,13), np.uint8)
    workplate_mask = cv.erode(workplate_mask, erode_kernel, iterations=1)
    minus_background = cv.bitwise_and(img, workplate_mask)
    #find document and subtract workplate 
    success, paper_cnt, paper_mask, cnt_img = find_match(minus_background, paper_mask)
    if not success:
        print("cannot find document... please rearrange on workplate")
        return False, img, None
    paper_mask = cv.erode(paper_mask, erode_kernel, iterations=1)
    minus_workplate = cv.bitwise_and(img, paper_mask)
    #create ROI as mask and apply to image
    rim_mask = create_rim(paper_cnt, paper_mask, (20.0, 25.0))[1]
    only_rim = cv.bitwise_and(img, rim_mask)
    if verbose:
        plot.subplot(411), plot.imshow(img, cmap = "gray"), plot.title("image"), plot.xticks([]), plot.yticks([])
        plot.subplot(412), plot.imshow(minus_background, cmap = "gray"), plot.title("subtracted background"), plot.xticks([]), plot.yticks([])
        plot.subplot(413), plot.imshow(minus_workplate, cmap = "gray"), plot.title("subtracted workplate"), plot.xticks([]), plot.yticks([])
        plot.subplot(414), plot.imshow(only_rim, cmap = "gray"), plot.title("only paper rim"), plot.xticks([]), plot.yticks([])
        plot.show()
    return True, only_rim, cnt_img

def mask_window(img, gripper_action_point):
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    mask = np.zeros(img.shape, np.uint8)
    field = 50
    cv.rectangle(mask, (gripper_action_point[0]-field, gripper_action_point[1]+(field)), (gripper_action_point[0]+field, gripper_action_point[1]-(field/10)), 255, -1)
    img = cv.bitwise_and(img, mask)
    if verbose:
        plot.subplot(211), plot.imshow(mask, cmap="gray"), plot.title("x"), plot.xticks([]), plot.yticks([])
        plot.subplot(212), plot.imshow(img, cmap="gray"), plot.title("x"), plot.xticks([]), plot.yticks([])
        plot.show()
    return img

def distance_to_point(point, gripper_action_point, arm_z):
    factor = -9530.9 * arm_z + 1949.7
    distance_x = (point[0] - gripper_action_point[0]) / factor
    distance_y = -(point[1] - gripper_action_point[1]) / factor
    print distance_x
    print distance_y
    return (distance_x, distance_y)


def main():
    img = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/paper640_0.jpg", 0)
    success, only_rim = detect_paper(img)
    if not success:
        return False
    detect_staple(only_rim)


if __name__ == "__main__":
    main()