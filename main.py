import cv2 as cv
import numpy as np
import serial
from matplotlib import pyplot as plt

# template = cv.imread('template.png')
# template = cv.cvtColor(template, cv.COLOR_RGBA2RGB)
# mask = cv.imread('mask.png')
# mask = cv.cvtColor(mask, cv.COLOR_RGBA2RGB)
# img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)
# template = cv.imread('mario_coin.png',0)
# w, h = template.shape[::-1]

# res = cv.matchTemplate(img_gray,template,cv.TM_CCOEFF_NORMED)
# threshold = 0.8
# loc = np.where( res >= threshold)
# for pt in zip(*loc[::-1]):
#     cv.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)

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

    # Our operations on the frame come here
    # img = cv.cvtColor(frame, cv.COLOR_RGB2RGBA)
    # img = cv.bitwise_not(frame).copy()

    # img = cv.imread('out.png')
    # img_display = img.copy()

    # img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # img = cv.medianBlur(img, 5)

    # img = cv.adaptiveThreshold(
    #     img, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)
    # img = cv.cvtColor(img, cv.COLOR_GRAY2RGB)

    # print(img, template, mask)
    # result = cv.matchTemplate(img, template, cv.TM_CCORR_NORMED, None, mask)

    # cv.normalize(result, result, 0, 1, cv.NORM_MINMAX, -1)
    # _minVal, _maxVal, minLoc, maxLoc = cv.minMaxLoc(result, None)

    # matchLoc = maxLoc

    # print(matchLoc)
    # print(_minVal, _maxVal)

    # # Display the resulting frame
    # if (abs(_minVal) < 1e-07):
    #     cv.rectangle(img_display, matchLoc,
    #                  (matchLoc[0] + template.shape[0], matchLoc[1] + template.shape[1]), (0, 0, 0), 2, 8, 0)
    #     cv.rectangle(result, matchLoc, (matchLoc[0] + template.shape[0],
    #                                     matchLoc[1] + template.shape[1]), (0, 0, 0), 2, 8, 0)

    # cv.imshow('img_display', img_display)
    cv.imshow('frame', frame)
    cv.imshow('img', img)
    cv.imshow('img_thr', img_thr)
    cv.imshow('img_mop', img_mop)
    # cv.imshow('img_canny', img_canny)
    # cv.imshow('result', result)

    key = cv.waitKey(1)

    if key == ord('q'):
        break

    # move film forward
    if key == ord('e'):
        ser.write(b'LLLLLLLLLLLLLLLLLLLLLLLL')
    # move film back
    if key == ord('a'):
        ser.write(b'RRRRRRRRRRRRRRRRRRRRRRRR')
    # shutter
    if key == ord('s'):
        ser.write(b'S')

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()

cv.imwrite('out.png', img)
