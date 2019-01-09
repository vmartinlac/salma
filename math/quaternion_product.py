
#!/usr/bin/env python3

import sympy
import sympy.printing

# Compute the jacobian of quaternion product wrt its factors.
# u = p*q

pi, pj, pk, pr = sympy.symbols(('pr', 'pi', 'pj', 'pk'))
qi, qj, qk, qr = sympy.symbols(('qr', 'qi', 'qj', 'qk'))

ui = pr * qi + qr * pi + (pj*qk - pk*qj)
uj = pr * qj + qr * pj + (pk*qi - pi*qk)
uk = pr * qk + qr * pk + (pi*qj - pj*qi)
ur = pr*qr - pi*qi - pj*qj - pk*qk

i = 0
for outvar in (ui, uj, uk, ur):
    j = 0
    for invar in (pi, pj, pk, pr, qi, qj, qk, qr):
        derivative = outvar.diff(invar)
        text = sympy.printing.ccode(derivative)
        print("J( " + str(i) + ", " + str(j)+" ) = " + text + ";")
        j += 1
    i += 1

