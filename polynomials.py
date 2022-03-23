

class Parabola:
    a: float
    b: float
    c: float
    h: float
    k: float

    def __init__(self, a: float, b: float, c: float):
        self.a = a
        self.b = b
        self.c = c
        self.update_h_and_k()

    def update_h_and_k(self):
        self.h = -self.b/(2*self.a)
        self.k = self.c - self.a * self.h * self.h

    def shift_right(self, steps_right: float):
        self.h = self.h + steps_right
        self.b = -2*self.a*self.h
        self.c = self.a*self.h*self.h + self.k
