import cv2
import numpy as np

def get_contour(image):
    img = cv2.GaussianBlur(cv2.Canny(image, 25, 255), (3, 3), 0)
    contours, hier = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    xy, app = [], []
    for i, contour in enumerate(contours):
        if i == 0: continue

        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
        cv2.drawContours(image, [contour], 0, (0, 255, 0), 1)
        M = cv2.moments(contour)
        if M['m00'] != 0:
            x = int(M['m10'] / M['m00'])
            y = int(M['m01'] / M['m00'])

        xy.append(tuple([x, y]))
        app.append(approx)
    return xy, app, contours

def shape_detect(image):
    img = image.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    xy, app, contours = get_contour(img)
    
    shapes = []
    for i, contour in enumerate(contours[:-1]):
        cv2.drawContours(img, [contour], 0, (0, 255, 0), 1)

        sides = len(app[i])
        if sides == 3:
            label = "Triangle"
        elif sides == 4:
            label = "Rectangle"
        else:
            label = "Circle"

        cv2.putText(img, label, xy[i], cv2.FONT_HERSHEY_COMPLEX, 0.3, (0, 255, 255), 1)
        shapes.append(label)
    return img, shapes

def color_detect(image):
    def add_label(label, contour, xy):
        for i, cont in enumerate(contour[:-1]):
            cv2.drawContours(img, [cont], 0, (0, 255, 0), 1)
            cv2.putText(img, label, xy[i], cv2.FONT_HERSHEY_COMPLEX, 0.4, (0, 255, 255), 1)

    img = image.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    orange_down = np.array([5, 100, 200], np.uint8)
    orange_up = np.array([15, 255, 255], np.uint8)

    white_down = np.array([0, 0, 200], np.uint8)
    white_up = np.array([180, 25, 255], np.uint8)

    orange_mask = cv2.inRange(hsv, orange_down, orange_up)
    white_mask = cv2.inRange(hsv, white_down, white_up)

    xy_or, _, contours_or = get_contour(orange_mask)
    add_label("Orange", contours_or, xy_or)

    xy_w, _, contours_w = get_contour(white_mask)
    add_label("White", contours_w, xy_w)

    orange_area_count = len(contours_or[:-1])
    white_area_count = len(contours_w[:-1])

    return img, orange_area_count, white_area_count

def detect_all(image):
    detect_color, orange, white = color_detect(image)
    detect_all, shapes = shape_detect(detect_color)
    return detect_all, [orange, white], shapes