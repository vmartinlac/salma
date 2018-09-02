import sympy

x1, x2, x3 = sympy.symbols( ('vx', 'vy', 'vz') )
a, b, c, d = sympy.symbols( ('aw', 'ax', 'ay', 'az') )

R11 = a**2 + b**2 - c**2 - d**2
R12 = 2*b*c - 2*a*d
R13 = 2*b*d + 2*a*c
R21 = 2*b*c + 2*a*d
R22 = a**2 - b**2 + c**2 - d**2
R23 = 2*c*d - 2*a*b
R31 = 2*b*d - 2*a*c
R32 = 2*c*d + 2*a*b
R33 = a**2 - b**2 - c**2 + d**2

invert = True
if invert:
    u1 = R11*x1 + R21*x2 + R31*x3
    u2 = R12*x1 + R22*x2 + R32*x3
    u3 = R13*x1 + R23*x2 + R33*x3
else:
    u1 = R11*x1 + R12*x2 + R13*x3
    u2 = R21*x1 + R22*x2 + R23*x3
    u3 = R31*x1 + R32*x2 + R33*x3

alpha = [a, b, c, d]
beta = [u1, u2, u3]
for i in range(3):
    for j in range(4):
        print( "J("+str(i)+", "+str(j)+") = ", beta[i].diff(alpha[j]) , ";")
    print("")
