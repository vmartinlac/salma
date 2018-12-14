#!/usr/bin/env python3

import sympy
import sympy.printing

qr, qi, qj, qk = sympy.symbols(('qr', 'qi', 'qj', 'qk'))

s = sympy.Rational(1,1) / (qi*qi + qj*qj + qk*qk + qr*qr)

R11 = 1 - 2*s*(qj*qj + qk*qk);
R12 = 2*s*(qi*qj - qk*qr);
R13 = 2*s*(qi*qk + qj*qr);

R21 = 2*s*(qi*qj + qr*qk);
R22 = 1 - 2*s*(qi*qi + qk*qk);
R23 = 2*s*(qj*qk - qi*qr);

R31 = 2*s*(qi*qk - qr*qj);
R32 = 2*s*(qj*qk + qi*qr);
R33 = 1 - 2*s*(qi*qi + qj*qj);

i = 0
for coeff in [R11, R12, R13, R21, R22, R23, R31, R32, R33]:
    j = 0
    for var in (qi, qj, qk, qr):
        derivative = coeff.diff(var)
        text = sympy.printing.ccode(derivative)
        print("JQ2R( " + str(i) + ", " + str(j)+" ) = " + text + ";")
        j += 1
    i += 1

