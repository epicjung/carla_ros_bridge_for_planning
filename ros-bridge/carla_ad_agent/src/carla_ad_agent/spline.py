"""
cubic spline planner

Author: Atsushi Sakai

"""
import math
import numpy as np
import bisect
import scipy.interpolate as interp

class Spline:
    u"""
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []
        self.a = []
        self.x = x
        self.y = y

        self.left_type = None
        self.left_val = None
        self.right_type = None
        self.right_val = None

    def set_two_points(self):

        self.nx = len(self.x)
        self.a = [iy for iy in self.y]

        A = np.zeros((self.nx, self.nx))
        B = np.zeros(self.nx)
        for i in range(1, self.nx-1, 1):
            A[i,i-1] = 1.0/3.0*(self.x[i]-self.x[i-1])
            A[i,i] = 2.0/3.0*(self.x[i+1]-self.x[i-1])
            A[i,i+1] = 1.0/3.0*(self.x[i+1]-self.x[i])
            B[i] = (self.y[i+1]-self.y[i])/(self.x[i+1]-self.x[i]) - (self.y[i]-self.y[i-1])/(self.x[i]-self.x[i-1])
        
        if self.left_type == 2:
            A[0,0] = 2.0
            A[0,1] = 0.0
            B[0] = self.left_val
        elif self.left_type == 1:
            A[0,0] = 2.0*(self.x[1]-self.x[0])
            A[0,1] = 1.0*(self.x[1]-self.x[0])
            B[0] = 3.0*((self.y[1]-self.y[0])/(self.x[1]-self.x[0])-self.left_val)

        if self.right_type == 2:
            A[self.nx-1, self.nx-1] = 2.0
            A[self.nx-1, self.nx-2] = 0.0
            B[self.nx-1] = self.right_val
        elif self.right_type == 1:
            A[self.nx-1, self.nx-1] = 2.0*(self.x[self.nx-1]-self.x[self.nx-2])
            A[self.nx-1, self.nx-2] = 1.0*(self.x[self.nx-1]-self.x[self.nx-2])
            B[self.nx-1] = 3.0 * (self.right_val - (self.y[self.nx-1]-self.y[self.nx-2])/(self.x[self.nx-1]-self.x[self.nx-2]))

        self.c = np.linalg.solve(A,B)
        for i in range(0, self.nx-1, 1):
            self.d.append(1.0/3.0*(self.c[i+1]-self.c[i])/(self.x[i+1]-self.x[i]))
            self.b.append((self.y[i+1]-self.y[i])/(self.x[i+1]-self.x[i])
                            - 1.0/3.0*(2.0*self.c[i]+self.c[i+1])*(self.x[i+1]-self.x[i]))

        h = self.x[self.nx-1]-self.x[self.nx-2]
        self.d.append(0.0)
        self.b.append(3.0*self.d[self.nx-2]*h*h+2.0*self.c[self.nx-2]*h+self.b[self.nx-2])


    def set_points(self):
        self.nx = len(self.x)  # dimension of x
        h = np.diff(self.x)

        # calc coefficient c
        self.a = [iy for iy in self.y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)
        
        # right extrapolation coeff
        self.d.append(0.0)
        self.b.append(3.0*self.d[self.nx-2]*h[self.nx-2]**2 + \
            2.0*self.c[self.nx-2]*h[self.nx-2] + self.b[self.nx-2])

    def calc(self, t):
        u"""
        Calc position

        if t is outside of the input x, return None

        """

        # if t < self.x[0]:
        #     return None
        # elif t > self.x[-1]:
        #     return None

        # extrapolation to the left
        if t < self.x[0]:
            h = t - self.x[0]
            return (self.c[0]*h + self.b[0])*h + self.a[0]
        # extrapolation to the right
        elif t > self.x[-1]:
            h = t - self.x[-1]
            return (self.c[-1]*h + self.b[-1])*h + self.a[-1]

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0
        return result

    def calcd(self, t):
        u"""
        Calc first derivative

        if t is outside of the input x, return None
        """
        if t < self.x[0]:
            h = t - self.x[0]
            return 2.0*self.c[0]*h + self.b[0]
        elif t > self.x[-1]:
            h = t - self.x[-1]
            return 2.0*self.c[-1]*h + self.b[-1]

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        u"""
        Calc second derivative
        """

        if t < self.x[0]:
            h = t - self.x[0]
            return 2.0*self.c[0]*h
        elif t > self.x[-1]:
            h = t - self.x[-1]
            return 2.0*self.c[-1]

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def set_boundary(self, left_type, left_val, right_type, right_val):
        self.left_type = left_type
        self.left_val = left_val
        self.right_type = right_type
        self.right_val = right_val    
    
    def __search_index(self, x):
        u"""
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        u"""
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        
        if self.left_type == 2: # second derivative
            A[0,0] = 2.0
            A[0,1] = 0.0
        elif self.left_type == 1: # first derivative
            A[0,0] = 2.0 * h[0]
            A[0,1] = 1.0 * h[0]

        if self.right_type == 2: # second derivative
            A[self.nx-1, self.nx-1] = 2.0
            A[self.nx-1, self.nx-2] = 0.0
        elif self.left_type == 1: # first derivative
            A[self.nx-1, self.nx-1] = 2.0 * h[self.nx-2]
            A[self.nx-1, self.nx-2] = 1.0 * h[self.nx-2]

        return A

    def __calc_B(self, h):
        u"""
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]

        if self.left_type == 2: # second derivative
            B[0] = self.left_val * 1.0
        elif self.left_type == 1: # first derivative
            B[0] = 3.0 * ((self.a[1]-self.a[0]/h[0]-self.left_val))
        
        if self.right_type == 2: # second derivative
            B[self.nx-1] = self.right_val * 1.0
        elif self.left_type == 1: # first derivative
            B[self.nx-1] = 3.0 * (self.right_val-(self.a[self.nx-1]-self.a[self.nx-2])/h[self.nx-2])

        return B


class Spline2D:
    u"""
    2D Cubic Spline class

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)
        self.sx.set_points()
        self.sy.set_points()

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        u"""
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        u"""
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / (dx ** 2 + dy ** 2)
        return k

    def calc_yaw(self, s):
        u"""
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = []
        try:
            yaw = math.atan2(dy, dx)
        except:
            print("dx: {}, dy: {}").format(dx, dy)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s


def main():
    print("Spline 2D test")
    import matplotlib.pyplot as plt
    x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]

    sp = Spline2D(x, y)
    s = np.arange(0, sp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    flg, ax = plt.subplots(1)
    plt.plot(x, y, "xb", label="input")
    plt.plot(rx, ry, "-r", label="spline")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    flg, ax = plt.subplots(1)
    plt.plot(s, [math.degrees(iyaw) for iyaw in ryaw], "-r", label="yaw")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("yaw angle[deg]")

    flg, ax = plt.subplots(1)
    plt.plot(s, rk, "-r", label="curvature")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("curvature [1/m]")

    plt.show()


if __name__ == '__main__':
    main()