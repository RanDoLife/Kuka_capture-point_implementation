#!/usr/bin/env python3

import cv2
import numpy as np

image_path = '/home/lirs/KUKA/kuka_ws/src/KUKA-ROS-Control/scripts/univer5004e594c2b611342498196kgu.jpg'
image = cv2.imread(image_path)
flipimg=cv2.flip(image,1)

flipimg=cv2.cvtColor(flipimg, cv2.COLOR_BGR2GRAY)
width = image.shape[1]
height = image.shape[0]
print("width", width)
print("height", height)
cv2.imshow('flip Image', flipimg)

def logo(img):
    blured_image = cv2.GaussianBlur(img, (7, 7), 0)
    edges = cv2.Canny(blured_image, 10, 30)
    # kernel = np.ones((5,5),np.uint8)
    # closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    cv2.imshow('edges', edges)
    # cv2.imshow('closed_edges', closed_edges)

    contours, _ = cv2.findContours(edges, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    black_image = np.zeros(( height,width, 3), dtype=np.uint8)
    cv2.drawContours(black_image, contours, -1, (255,255,255), 2)
    drawing = cv2.bitwise_not(black_image)
    cv2.imshow('LOGO', drawing)
    print("logo", len(contours))


def portrait(img):
    blured_image = cv2.GaussianBlur(image, (7, 7), 0)
    edges = cv2.Canny(blured_image, 30, 70)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    black_image = np.zeros(( height,width, 3), dtype=np.uint8)
    cv2.drawContours(black_image, contours, -1, (255,255,255), 2)
    drawing = cv2.bitwise_not(black_image)
    cv2.imshow('portrait', drawing)
    print("portrait", len(contours))

logo(flipimg)
portrait(flipimg)




cv2.waitKey(0)
cv2.destroyAllWindows()