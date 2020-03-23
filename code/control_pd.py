
import numpy as np
import math
import cv2


class pd_controller(object):
    """docstring for control_pd."""
    def __init__(self, ref, kp=0.5, kd=1.5):
        super(pd_controller, self).__init__()
        self.ref = ref
        self.kp = kp  # cte proporcional
        self.kd = kd  # cte derivativa
        self.last_error = 0

    def process(self, value):
        error = self.ref - value
        deriv = error - self.last_error
        self.last_error = error

        return error*self.kp + deriv*self.kd



class RoadAnalizator:
    def __init__(self, angle):
        self.ref_angle = 90 + angle


    def road_state(self, m):
        # recta, curva izq, curva dcha, None
        if m == None or m == 0:
            return None

        angle = self._to_degrees(m)
        diff = angle - self.ref_angle

        if abs(diff) < 10:
            return 'recta'
        elif angle > self.ref_angle:
            return 'curva derecha'
        else:
            return 'curva izquierda'


    def road_state_two_lines(self, inf_line, sup_line, y_conexion):
        # recta, curva izq, curva dcha, None
        m_inf, n_inf = inf_line
        m_sup, n_sup = sup_line

        if m_sup == None or m_inf == 0:
            return None

        x_0 = (y_conexion - n_inf) // m_inf
        x_1 = (y_conexion - n_sup) // n_sup
        #print(x_0, ' ', x_1)
        #print('umbral union: ', abs(x_0) - abs(x_1))
        #print('umbral recta: ', abs(m_inf) - abs(m_sup))

        if abs(x_0) - abs(x_1) > 5:
            return None

        if abs(m_inf) - abs(m_sup) < 5:
            return 'recta'
        else:
            return 'curva'


    def car_state(self):
        # centrado, izq, dcha
        pass

    def direction_state(self, car_state, road_state):
        #
        pass

    def _to_degrees(self, m):
        return 90 + math.degrees(math.atan(m))




class ImageAnalizator(object):
    """docstring for analize_img."""
    def __init__(self):
        super(ImageAnalizator, self).__init__()
        self.lower_color = np.array([  0, 225,   0], np.uint8) #  0, 255, 0
        self.upper_color = np.array([  0, 255, 255], np.uint8) #  0, 255, 255


    def color_filter(self, img):
        """
        Recibe una image RGB y devuelve una imagen binaria con la linea de
        la carretera segmentada.
        """

        t_img = cv2.GaussianBlur(img, (7, 7), 0)
        t_img = cv2.cvtColor(t_img, cv2.COLOR_RGB2HSV)
        t_img = cv2.inRange(t_img, self.lower_color, self.upper_color)

        kernel = np.ones((7,7),np.uint8)
        t_img = cv2.morphologyEx(t_img, cv2.MORPH_CLOSE, kernel)
        return t_img



    def centroids_of_lines(self, img, y_points):
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


    def calculate_line_(self, x_points, y_points):
        if len(x_points) > 1 and len(y_points) > 1:
            m, n = np.polyfit(x_points, y_points, deg=1)
            return m, n
        return 0, 0


    def calculate_line(self, img, y_points):
        """
        """
        aux = [img[i, :] for i in y_points]

        x_coords = []
        y_coords = []
        for line, y in zip(aux, y_points):
            indx = np.where(line == 255)
            if indx[0].size > 1:
                x = indx[0]
                x_coords.append((x[0]+x[-1]) // 2)
                y_coords.append(479 - y)

        if x_coords:
            m, n = np.polyfit(x_coords, y_coords, deg=1)
            return m, n
        return 0, 0


    def line_angle(self, pendiente):
        return math.degrees(math.atan(pendiente))


    def calculate_x(self, m, n, y_points):
        # (y - n) / m = x
        return [int((y - n) / m) for y in y_points]


    def calculate_radius(self, x, y):
        """ https://en.wikipedia.org/wiki/Radius_of_curvature """

        if not(len(x) > 2 and len(y) > 2):
            print('Insuficientes args')
            return 0

        f = np.polyfit(x, y, deg=2)
        if f[0] == 0:
            print('a=0')
            return 0

        x_mid = 1 #(np.max(x) + np.min(x)) // 2
        return np.absolute(((1 + 2*f[0]*x_mid + f[1])**1.5 / 2*f[0]))
