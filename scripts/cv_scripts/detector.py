#!/usr/bin/env python
#-*- coding:utf-8 -*-
import cv2 as cv
import sys
import numpy as np
from copy import deepcopy

def create_rim(contour, mask, rim_width, verbose=False):
    """
    Modifies the given mask by a smaller version of the given contour to create a white rim zone in accordance to the given width.

        Parameters:
            contour:    Contour of the document on which the rim shall be created
            mask:       Mask of the document on which the rim shall be created
            rim_width:  Width of the rim to be created
            verbose:    Flag to print messages for debugging
        Return:
            mask:       Modified version of the given mask; None if rim is not creatable
    """
    rect = cv.minAreaRect(contour)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    main_length, sec_length, angle, mainline_endpoint = get_box_info(box)
    if main_length == 0:
        return None
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
    return mask

def create_box_mask(contour, image_shape):
    """
    Creates a mask with a box contour of the given contour.
    
        Parameters:
            contour:        Contour as base for the mask
            image_shape:    Shape of the box to be created
        Return:
            mask:           Created mask of box contour
    """
    mask = np.ones(image_shape, np.uint8)
    box = create_box_contour(contour)
    cv.drawContours(mask, [box], 0, 255, -1)
    return mask

def create_box_contour(contour):
    """
    Creates a box contour on basis of the given contour.
    
        Parameters:
            contour:    Basis for the box contour to be created
        Return:
            box:        Box contour
    """
    rect = cv.minAreaRect(contour)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    return box

def get_box_info(box_cnt):
    """
    Returns a number of informations of the given box contour.

    The mainline of the box contour refers to the longer one of the edges starting from the first point of box_cnt.
    The first point of a box contour is always the one with the highest row value, therefore the one closest to the bottom of the screen if displayed.
    The following points of a box contour are then clockwise arranged.
    So the mainline goes either from 0 to 1 or 0 to 3.
    For more information on this function please see "Metallentfernung an Dokumenten durch den Forschungsroboter Baxter" by "Timo Schürmann"
    
        Parameters:
            box_cnt:            Box contour
        Return:
            main_length:        Length of the given box / Lenght of the mainline
            secondary_length:   Width of the given box
            angle:              Angle of the mainline of the box
            mainline_endpoint:  Endpoint of the mainline of the box. Either 1 or 3
    """
    l03 = np.sqrt(np.square(box_cnt[3][0] - box_cnt[0][0]) + np.square(box_cnt[3][1] - box_cnt[0][1]))
    l01 = np.sqrt(np.square(box_cnt[1][0] - box_cnt[0][0]) + np.square(box_cnt[1][1] - box_cnt[0][1]))
    ankathete = float(box_cnt[3][1] - box_cnt[0][1])
    if ankathete != 0.0:
        gkdak = float(box_cnt[3][0] - box_cnt[0][0]) / ankathete
        angle03 = np.arctan(gkdak)+(np.pi/2)
    else:
        angle03 = 0.0
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

def find_match(img, comparator, maxL=1280.0, minL=0.0, verbose=False):
    """
    Extracts all contours from the given image, finds the best matching one to the comparator and returns it in form of different datatypes.

    img and comparator must have the same resolution.
    
        Parameters:
            img:            Image on which the designated contour shall be found
            comparator:     Mask which resembles the box contour to be found
            maxL:           Maximal mainlength of the box contour to be found
            minL:           Minimal mainlength of the box contour to be found
            verbose:        Flag to print messages for debugging
        Return:
            True/False:     Whether a match was found or not
            best_contour:   Box contour of the best match; None if no match was found
            best_mask:      Mask of the box contour of the best match; None if no match was found
            show_img:       Grayscale input image with found best matching box contour drawn on it in green; None if no match was found
    """
    if len(img.shape) > 2:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(img, 10, 100)
    contours = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[1]
    best_match = 1.0
    best_contour = None
    i = 0
    for c in contours:
        work_mask = create_box_mask(c, img.shape)
        match = cv.matchShapes(comparator, work_mask, 2, 0.0)
        arcL = cv.arcLength(create_box_contour(c), True)
        if arcL < maxL and arcL > minL and match < best_match:
            i+=1
            if verbose:
                print(arcL)
            best_match = match
            best_mask = deepcopy(work_mask)
            best_contour = create_box_contour(c)
    if verbose:
        print("deviation for best match at: {}%\nnumber of best matches: {}".format(float(best_match)*100, i))
    if not best_contour is None:
        show_img = cv.cvtColor(deepcopy(img), cv.COLOR_GRAY2BGR)
        cv.drawContours(show_img, [best_contour], 0, (0,255,0), 1)
        return True, best_contour, best_mask, show_img
    else:
        return False, None, None, None

def sortkey_first(x):
    """
    Sortkey for find_matches. Sorts the list in accordance to the first element.
    """
    return x[0]

def find_matches(img, comparator, maxL=1280.0, minL=0.0, deviation_thresh=0.15, verbose=False):
    """
    Extracts all contours from the given image and returns all found matches with a deviation lower than 0.15 as a sorted list.

    img and comparator must have the same resolution.
    
        Parameters:
            img:                Image on which the designated contour shall be found
            comparator:         Mask which resembles the box contour to be found
            maxL:               Maximal mainlength of the box contour to be found; Default: 1280.0
            minL:               Minimal mainlength of the box contour to be found; Default: 0.0
            deviation_thresh:   Maximal deviation of found contour to comparator; Default: 0.15
            verbose:            Flag to print messages for debugging
        Return:
            True/False:         Whether one or more matches were found or not
            best_matches:       List of all found matches, sorted by deviation in ascending order; Empty if no matches were found
    """
    if len(img.shape) > 2:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(img, 10, 100)
    contours = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[1]
    best_matches = list()
    for c in contours:
        work_mask = create_box_mask(c, img.shape)
        match = cv.matchShapes(comparator, work_mask, 2, 0.0)
        arcL = cv.arcLength(create_box_contour(c), True)
        if arcL < maxL and arcL > minL and match < deviation_thresh:
            if verbose:
                print(arcL)
            best_matches.append([match,create_box_contour(c), deepcopy(work_mask)])
    best_matches.sort(key=sortkey_first)
    if len(best_matches) > 0:
        print("max deviation for matches at: {}%\n\n".format(deviation_thresh*100))
        print("number of matches: {}".format(len(best_matches)))
        print("deviation for best match at {}%".format(best_matches[0][0]))
        return True, best_matches
    else:
        return False, best_matches

def detect_staple(img):
    """
    Function to detect staples in a given image.

    The given image must have a resolution of 640x400 or 320x200.
    This functions uses different approaches and return parameters for the different image resolutions. 
    So please choose the right resolution for the effect you want to accomplish.

    640x400 -> The Method searches for all contours on the image that resemble that of a staple from afar. It returns a sorted list of all found contours with a deviation of maximum 15%
    320x200 -> The Method searches for the contour that resembles that of a staple from near with the least deviation.
    
        Parameters:
            img:                Image on which the designated contour shall be found
        Return:
            640x400:
                success:        True if at least one match was found, else False
                found_matches:  List of all found matches, sorted by deviation in ascending order; None if no matches were found
                None:           Only for compatibility with the return parameters for 320x200 images
            320x200:
                success:        True if a match was found, else False
                found_contour:  Box contour of the found match
                cnt_img:        Given image with the found contour drawn on it
    """
    if len(img.shape) > 2:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    if img.shape[0] == 400:
        cmp_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_images/masks/small_staple.jpg", 0)
        maxL = 50.0
        minL = 20.0
        success, found_matches = find_matches(img, cmp_mask, maxL=maxL, minL=minL)
        if not success:
            print("cannot find any staple")
            return success, None, None
        return success, found_matches, None
    elif img.shape[0] == 200:
        cmp_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_images/masks/staple1.jpg", 0)
        maxL = 120.0
        minL = 60.0
        img = cv.GaussianBlur(img, (5,5), 0)
        success, found_contour, best_mask, cnt_img = find_match(img, cmp_mask, maxL=maxL, minL=minL)
        if not success:
            print("cannot find any staple")
            return success, img, None
        return success, found_contour, cnt_img
    else:
        print("given image has wrong resolution. Please provide a picture with following format: 640x400")
        return success, img, None


def detect_paper(img):
    """
    Function to detect a document on an appropriate workplate and apply a mask on it, so that only the clear rim of the document remains.

    For this to function a specially arranged workspace is needed. Please provide such as described in "Metallentfernung an Dokumenten durch den Forschungsroboter Baxter" by "Timo Schürmann".
    
        Parameters:
            img:        Image on which the designated contour shall be found. Must be of resolution 640x400
        Return:
            success:    True if a document was found, else False
            only_rim:   Given image masked, so that only the clear rim remains; img if no document was found
            paper_cnt:  Box contour of the found document; None if no document was found
    """
    if len(img.shape) > 2:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    if img.shape[0] != 400:
        print("given image has wrong resolution. Please provide a picture with following format: 640x400")
        return False, img, None
    #get background mask
    workplate_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_images/masks/background.jpg", 0)
    workplate_mask = cv.resize(workplate_mask, img.shape[1::-1])
    #get workplate mask
    paper_mask = cv.imread("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_images/masks/document.jpg", 0)
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
    return True, only_rim, paper_cnt

def mask_window(img, gripper_action_point):
    """
    Masks the given image, so that only a field around the given action point remains visible.
    
        Parameters:
            img:                    Image to be masked
            gripper_action_point:   Action point of the used end effector
        Return:
            masked_img:             Given image with applied mask
    """
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    mask = np.zeros(img.shape, np.uint8)
    field = 60
    cv.rectangle(mask, (gripper_action_point[0]-field, gripper_action_point[1]+field), (gripper_action_point[0]+field, gripper_action_point[1]-10), 255, -1)
    masked_img = cv.bitwise_and(img, mask)
    return masked_img

def draw_cnt_on_img(cnt, img):
    """
    Draws a given contour on a given image.
    
        Parameters:
            cnt:    Contour to be drawn  
            img:    Image to be drawn on
        Return:
            Image:  Given image with contour drawn on it
    """
    if len(img.shape) == 2:
        img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    return cv.drawContours(img, [cnt], 0, (0,0,255), 1)