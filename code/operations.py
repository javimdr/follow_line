
import numpy as np


def obtein_x(line, y):
    return float(y - line[1]) / line[0]


def generate_points(img, y_points):
    lines = np.asarray([img[i, :] for i in y_points])
    centroids = []
    y_coords = [] # por si no existe punto x
    for line, y in zip(lines, y_points):
        indx = np.where(line > 0)
        if indx[0].size > 1:
            x = indx[0]
            centroids.append((x[0]+x[-1]) // 2)
            y_coords.append(y)
    return centroids, y_coords


def centroid_of_multiple_vectors(arr):
    centers = []
    for v in arr:
        cen = centroid_of_vector(v)
        if cen:
            centers.append(cen)
    return centers


def centroid_of_vector(v):
    v = v / 255
    index = np.argwhere(v > 0).ravel()
    if len(index) > 0:
        return int(np.mean(index))
    return None


def calculate_line(x_points, y_points):
    if len(x_points) > 1 and len(y_points) > 1:
        f = np.polyfit(x_points, y_points, deg=1)
        if f[0] != 0:
            return f
    return None


def calculate_line_polyfit_2p(p0, p1):
    x = [p0[0], p1[0]] if p0[0] != p1[0] else [p0[0]+1, p1[0]]
    y = [p0[1], p1[1]]
    f = np.polyfit(x, y, deg=1)
    if f[0] != 0:
        return f
    return None

def calculate_line_two_points(p0, p1):
    x0, y0 = p0
    x1, y1 = p1
    if x0 == x1:
        x0 += 1
    aux = x1 - x0

    if aux != 0:
        m = float(y1 - y0) / aux
        if m != 0:
            n = y1 - m*x1
            return [m, n]

    return None
