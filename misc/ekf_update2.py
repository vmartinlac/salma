import sympy
import re

x1, x2, x3 = sympy.symbols(('x1', 'x2', 'x3'))
a0, a1, a2, a3 = sympy.symbols(('a0', 'a1', 'a2', 'a3'))
y1, y2, y3 = sympy.symbols(('y1', 'y2', 'y3'))

R11 = 1 - 2*(a2*a2 + a3*a3);
R12 = 2*(a1*a2 - a3*a0);
R13 = 2*(a1*a3 + a2*a0);
R21 = 2*(a1*a2 + a0*a3);
R22 = 1 - 2*(a1*a1 + a3*a3);
R23 = 2*(a2*a3 - a1*a0);
R31 = 2*(a1*a3 - a0*a2);
R32 = 2*(a2*a3 + a1*a0);
R33 = 1 - 2*(a1*a1 + a2*a2);

d1 = y1 - x1
d2 = y2 - x2
d3 = y3 - x3

ycam1 = R11*d1 + R21*d2 + R31*d3
ycam2 = R12*d1 + R22*d2 + R32*d3
ycam3 = R13*d1 + R23*d2 + R33*d3

aa = [x1, x2, x3, a0, a1, a2, a3, y1, y2, y3]
bb = [ycam1, ycam2, ycam3]

guess = [
    [-R11, -R21, -R31, 0, 0, 0, 0, R11, R21, R31],
    [-R12, -R22, -R32, 0, 0, 0, 0, R12, R22, R32],
    [-R13, -R23, -R33, 0, 0, 0, 0, R13, R23, R33] ]

for i in range(3):
    for j in range(10):
        s = str( bb[i].diff(aa[j]) - guess[i][j] )
        print(s)

