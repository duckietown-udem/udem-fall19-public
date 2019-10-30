import cv2
import numpy as np
import rospy

## Software Exercise 6: Choose your category (1 or 2) and replace the cv2 code by your own!

## CATEGORY 1
def inRange(hsv_image, low_range, high_range):
	return cv2.inRange(hsv_image, low_range, high_range)

def bitwise_or(bitwise1, bitwise2):
	return cv2.bitwise_or(bitwise1, bitwise2)

def bitwise_and(bitwise1, bitwise2):
	return cv2.bitwise_and(bitwise1, bitwise2)

def getStructuringElement(shape, size):
	return cv2.getStructuringElement(shape, size)

def dilate(bitwise, kernel):
	return cv2.dilate(bitwise, kernel)



## CATEGORY 2
def Canny(image, threshold1, threshold2, apertureSize=3):
	return cv2.Canny(image, threshold1, threshold2, apertureSize=3)


## CATEGORY 3 (This is a bonus!)
def HoughLinesP(image, rho, theta, threshold, lines, minLineLength, maxLineGap):
	return cv2.HoughLinesP(image, rho, theta, threshold, lines, minLineLength, maxLineGap)