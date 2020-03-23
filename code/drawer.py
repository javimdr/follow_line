

import cv2
import numpy as np


def draw_points(img, x_points, y_points, color=(255,0,0), size=6):
    for x, y in zip(x_points, y_points):
        cv2.circle(img, (int(x), int(y)), size, color, -1)


def draw_line(img, line, y_inf, y_sup, color=(0,255,0)):
    x_inf = int((y_inf - line[1]) / line[0])
    x_sup = int((y_sup - line[1]) / line[0])
    p0 = (x_inf, y_inf)
    p1 = (x_sup, y_sup)
    cv2.line(img, p0, p1, color, 4)
