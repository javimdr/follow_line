
from control_pd import *
from operations import *
from drawer import *


W_REF = 320#0.49
Kp_w = 1.1#1.2
Kd_w = 4.5#5

V_REF = 0
Kp_v = 40#20#40#60.3
Kd_v = 10#30#50#100

class controller:

    CONT_MAX = 10
    ROWS = np.arange(249, 280, 3)  #(249, 400, 10)[249, 274, 299, 324, 349, 374, 399]
    UP_ROW = 249  # 320
    DOWN_ROW = 225  # 325

    def __init__(self):
        #self.w_controller = pd_controller(W_REF, Kp_w, Kd_w)
        #self.v_controller = pd_controller(V_REF, Kp_v, Kd_v)

        self.w_controller = pd_controller(320, 0.015, 0.0095) #320, 0.015, 0.0095#0.005#0.015
        self.v_controller = pd_controller(0, 4, -5)  #0, 1.6, 0.5#(320, 0.02, 0.015)

        self.last_ref = None  # ultima referencia (t-1)
        self.cont_lost_line = self.CONT_MAX  # contador de busqueda de linea

        self.v = {'min_fl': 3,   # v min estado 1
                  'max_fl': 18,  # v max estado 1
                  'min_ll': 3,   # v min estado 2
                  'max_ll': 5,   # v max estado 2
                  'sl'    : 0.1} # v estado 3
        self.w = {'sl': 0.2}
        self._test = 1.02


    def execute(self, bw_img, color_img, info_img):
        """ CONTROL """
        line = self.build_line(bw_img)

        # Estado 1: linea encontrada
        if line != []:
            estado = 1
            self.cont_lost_line = 0
            w, v = self.follow_line(line)

        # Estado 2: linea perdida
        elif self.cont_lost_line < self.CONT_MAX:
            estado = 2
            self.cont_lost_line += 1
            w, v = self.lost_line()

        # Estado 3: Buscar linea
        else:
            estado = 3
            w, v = self.search_line()

        self.print_info(estado, v, w, line, color_img, info_img)
        return v, w


    def print_info(self, estado, v, w, line, rgb_img, info_img):
        # draw ref
        draw_points(rgb_img, [W_REF], [self.UP_ROW], (0,0,255))
        cv2.line(rgb_img, (0, 249), (600, 249), (255, 255, 0), 1)
        cv2.line(rgb_img, (0, 280), (600, 280), (255, 255, 0), 1)

        # draw info
        if estado == 1:
            estado = 'FOLLOW LINE'
            x = '{:.3f}'.format(self.last_ref)
            error = '{:.3f}'.format(W_REF - self.last_ref)

            draw_line(rgb_img, line, 0, 400)
            draw_points(rgb_img, [self.last_ref], [self.UP_ROW])

            # test
            r = self.road(line)
        if estado == 2:
            estado = 'LOST LINE, cont: %i' % self.cont_lost_line
            x = '{:.3f}'.format(self.last_ref)
            error = '{:.3f}'.format(W_REF - self.last_ref)

            draw_points(rgb_img, [self.last_ref], [self.UP_ROW])
        if estado == 3:
            estado = 'SEARCHING LINE'
            x = 'None'
            error = 'None'

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(info_img, 'Estado: {}'.format(estado), (10, 150), font, 1, (0, 0, 0), 2)
        cv2.putText(info_img, '     v: {:.3f}'.format(v), (10, 200), font, 1, (0, 0, 0), 2)
        cv2.putText(info_img, '     w: {:.3f}'.format(w), (10, 250), font, 1, (0, 0, 0), 2)
        cv2.putText(info_img, '     x: {}'.format(x), (10, 300), font, 1, (0, 0, 0), 2)
        cv2.putText(info_img, ' error: {}'.format(error), (10, 350), font, 1, (0, 0, 0), 2)

        if estado == 'FOLLOW LINE':
            cv2.putText(info_img, '  ROAD: {}'.format(r), (10, 390), font, 1, (0, 0, 0), 2)


    def road(self, line):
        x_row_ref = self.last_ref
        x_row_down = float(self.DOWN_ROW - line[1]) / line[0]
        up_u   = 20
        down_u = 40
        if abs(W_REF - self.last_ref) < up_u and \
            abs(325 - x_row_down) < down_u:
            return 'Recta'

        return 'Curva'
        """
        diff = abs(x_row_ref - x_row_down)

        umbral = 40
        if diff > umbral:
            return 'Curva'
        return 'Recta'
        """


    # ----- ESTADOS -----
    def follow_line(self, line):
        """ Estado 1: Siguiendo linea. """
        x = float(self.UP_ROW - line[1]) / line[0]# self.calc_ref(line, self.UP_ROW, 660)
        self.last_ref = x
        w = self.w_controller.process(x)
        v = self.v_controller.process(abs(w)) #* (self.v['max_fl'] - self.v['min_fl'])

        v = max(self.v['min_fl'],
                self.v['max_fl'] - abs(v))
        r = self.road(line)
        if r == 'Curva':
            v = min(11, v)
        return w, v

    def lost_line(self):
        """ Estado 2: Linea perdida. """
        x = self.last_ref * self._test
        self.last_ref = x
        w = self.w_controller.process(x)
        v = self.v_controller.process(w) #* (self.v['max_fl'] - self.v['min_fl'])

        v = max(self.v['min_ll'], self.v['max_ll'] - abs(v))
        return w, v

    def search_line(self):
        """ Estado 3: Buscando linea. """
        self.last_ref = None
        w, v = self.w['sl'], self.v['sl']

        return w, v

    # Main Functions
    def build_line(self, bw_img):
        """ Aproxima la linea detectada con dos puntos """
        p_sup = self.search_point(bw_img)
        p_inf = self.search_point(bw_img, flip=True)

        if (not p_sup or not p_inf) or p_sup == p_inf:
            return []

        return calculate_line_polyfit_2p(p_inf, p_sup)

    def calc_ref(self, line, y=400, norm_value=660):
        x = float(y - line[1]) / line[0]
        return x / float(norm_value)


    # Helper Functions
    def search_point(self, img, flip=False):
        rows = self.ROWS[::-1] if flip else self.ROWS
        for y in rows:
            x = centroid_of_vector(img[y, :])
            if x:
                return (x, y)
        return None
