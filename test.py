from line import Line
from point import Point
from polynomials import Parabola

poly = Parabola(a=-0.0035, b=0.2695, c=-4.4243)
print("poly1 = [%0.4f, %0.4f, %0.4f]" % (poly.a, poly.b, poly.c))
poly.shift_right(steps_right=-5)
print("poly2 = [%0.4f, %0.4f, %0.4f]" % (poly.a, poly.b, poly.c))
