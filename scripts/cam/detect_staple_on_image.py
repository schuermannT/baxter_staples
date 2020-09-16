#!/usr/bin/env python
#-*- coding:utf-8 -*-
import cv2 as cv
import sys
import numpy as np
from matplotlib import pyplot as plot



def main():
    #calculations
    kernel = np.ones((7,3), np.uint8)
    img = cv.imread("/home/user/schuermann_BA/cv_tests/gardinen_zu_licht_an.jpg", 0) #die 0 wandelt das bild direkt zu Grayscale
    gauss_blur = cv.GaussianBlur(img, (5,5), 0)
    gauss_gauss = cv.adaptiveThreshold(gauss_blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2)
    gg_e = cv.erode(gauss_gauss, kernel)
    canny = cv.Canny(img, 100, 200)

    #show in plot
    plot.subplot(511), plot.imshow(img, cmap = "gray"), plot.title("original"), plot.xticks([]), plot.yticks([])
    plot.subplot(512), plot.imshow(gauss_blur, cmap = "gray"), plot.title("gauss blur"), plot.xticks([]), plot.yticks([])
    plot.subplot(513), plot.imshow(gauss_gauss, cmap = "gray"), plot.title("gauss adaptive threshold after gauss blur"), plot.xticks([]), plot.yticks([])
    plot.subplot(514), plot.imshow(gg_e, cmap = "gray"), plot.title("erosion after gat after gb"), plot.xticks([]), plot.yticks([])
    plot.subplot(515), plot.imshow(canny, cmap = "gray"), plot.title("canny edge detection"), plot.xticks([]), plot.yticks([])
    plot.show()
    #cv.waitKey(0)


if __name__ == "__main__":
    main()