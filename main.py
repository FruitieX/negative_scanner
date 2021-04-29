import cv2 as cv
import numpy as np
import serial
from matplotlib import pyplot as plt

print("opening serial port...")
ser = serial.Serial('COM3', 19200)

print("starting video capture...")
cap = cv.VideoCapture(0)

cv.namedWindow('controls')


def nothing(x):
    pass


cv.createTrackbar('A', 'controls', 4, 11, nothing)
cv.createTrackbar('B', 'controls', 255, 512, nothing)
cv.createTrackbar('C', 'controls', 5, 20, nothing)
cv.createTrackbar('C2', 'controls', 2, 20, nothing)
cv.createTrackbar('D', 'controls', 7, 11, nothing)


def filter_large_contours(contour):
    area = cv.contourArea(contour)
    return area > 100000


def sort_by_contour_centroid(contour):
    M = cv.moments(contour)
    cx = int(M['m10']/M['m00'])
    cx


MANUAL_MODE = True

searching_frame = True
waiting_for_shutter = False

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    A = cv.getTrackbarPos('A', 'controls')
    B = cv.getTrackbarPos('B', 'controls')
    C = cv.getTrackbarPos('C', 'controls')
    C2 = cv.getTrackbarPos('C2', 'controls')
    D = cv.getTrackbarPos('D', 'controls')

    img = frame.copy()
    w = img.shape[1]
    # img = img[41:-59, 52:-52]
    img = img[22:-28, 52:-52]
    img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    # img = cv.medianBlur(img, 5)
    img = cv.medianBlur(img, A * 2 + 1)

    img_thr = cv.adaptiveThreshold(
        # img, 255, cv.THRESH_BINARY, cv.ADAPTIVE_THRESH_GAUSSIAN_C, 51, 0)
        # img, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 5, 2)
        img, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, C, C2)

    # kernel = np.ones((img.shape[0], 1), dtype=np.uint8) * 255
    kernel = np.ones((img.shape[0], 1), dtype=np.uint8) * 255
    img_mop = cv.morphologyEx(img_thr, cv.MORPH_OPEN, kernel)

    # img_mop = cv.medianBlur(img_mop, 5)
    img_mop = cv.medianBlur(img_mop, D * 2 + 1)

    # img_canny = cv.Canny(img_mop, 1, 3)

    img_mop = cv.bitwise_not(img_mop)

    contours, hierarchy = cv.findContours(
        img_mop, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    contours = list(filter(filter_large_contours, contours))
    contours.sort(key=sort_by_contour_centroid)

    cv.drawContours(img, contours, -1, (0, 255, 0), 3)

    cv.imshow('frame', frame)
    cv.imshow('img', img)
    cv.imshow('img_thr', img_thr)
    cv.imshow('img_mop', img_mop)

    key = cv.waitKey(1)

    if key == ord('q'):
        break

    if MANUAL_MODE:
        # move film forward
        if key == ord('e'):
            ser.write(b'LLLLLLLLLLLLLLLLLLLLLLLL')
        # move film back
        if key == ord('a'):
            ser.write(b'RRRRRRRRRRRRRRRRRRRRRRRR')
        # shutter
        if key == ord('s'):
            ser.write(b'S')

    else:
        if contours:
            last = contours[-1]
            M = cv.moments(last)
            cx = int(M['m10']/M['m00'])
            print("last contour centroid:")
            print(cx)
        else:
            ser.write(b'RRRRRRRRRRRRRRRRRRRRRRRR')

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()

cv.imwrite('out.png', img)
