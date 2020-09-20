#!/usr/bin/env python
#-*- coding:utf-8 -*-

import cv2 as cv
import sys

img = cv.imread("/home/user/schuermann_BA/sparrow.jpg")
if img is None:
    sys.exit("Bild geht nicht...")

cv.imshow("Vugel", img)
key = cv.waitKey(0)

cv.rectangle(img, (300,200), (500,400), (255,0,0), 2)
cv.imshow("Vugel mit Quadrat", img)
key = cv.waitKey(0)

if key == ord("s"):
    cv.imwrite("/home/user/schuermann_BA/sparrow2.jpg",img)