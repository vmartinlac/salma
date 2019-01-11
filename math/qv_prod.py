#!/usr/bin/env python3

import sympy
import sympy.printing

qr, qi, qj, qk = sympy.symbols(('qr', 'qi', 'qj', 'qk'))
vi, vj, vk = sympy.symbols( ('vi', 'vj', 'vk') )

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

ui = R11*vi + R12*vj + R13*vk
uj = R21*vi + R22*vj + R23*vk
uk = R31*vi + R32*vj + R33*vk

i = 0
for outvar in (ui, uj, uk):
    j = 0
    for invar in (qi, qj, qk, qr, vi, vj, vk):
        derivative = outvar.diff(invar)
        text = sympy.printing.ccode(derivative)
        print("J(" + str(i) + ", " + str(j)+") = " + text + ";")
        j += 1
    i += 1

i = 0
for outvar in (ui, uj, uk):
    print("ret("+str(i)+") = " + sympy.printing.ccode(outvar) + ";")
    i += 1

