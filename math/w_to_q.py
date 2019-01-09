
#!/usr/bin/env python3

import sympy
import sympy.printing

# Compute the jacobian of the map which associates a quaternion to a rotation vector.

wx, wy, wz = sympy.symbols(('wx', 'wy', 'wz'))

n = (wx**2 + wy**2 + wz**2)**0.5

costheta = sympy.cos(n/2)
sintheta = sympy.sin(n/2)

ax = wx/n
ay = wy/n
az = wz/n

qx = sintheta*ax
qy = sintheta*ay
qz = sintheta*az
qw = costheta

i = 0
for outvar in (qx, qy, qz, qw):
    j = 0
    for invar in (wx, wy, wz):
        derivative = outvar.diff(invar)
        text = sympy.printing.ccode(derivative)
        print("J( " + str(i) + ", " + str(j)+" ) = " + text + ";")
        j += 1
    i += 1

i = 0
for outvar in (qx, qy, qz, qw):
    print("ret("+str(i)+") = " +sympy.printing.ccode(outvar) + ";")
    i += 1

