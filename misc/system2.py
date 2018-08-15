import sympy

x1, x2, x3 = sympy.symbols(('x1', 'x2', 'x3'))
a0, a1, a2, a3 = sympy.symbols(('a0', 'a1', 'a2', 'a3'))
v1, v2, v3 = sympy.symbols(('v1', 'v2', 'v3'))
w1, w2, w3 = sympy.symbols(('w1', 'w2', 'w3'))

dt = sympy.symbols('dt')

x1n = x1 + dt*v1
x2n = x2 + dt*v2
x3n = x3 + dt*v3

norm = (w1*w1 + w2*w2 + w3*w3)**0.5
angle = dt*norm
cos = sympy.cos(angle/2)
sin = sympy.sin(angle/2)
rot0 = cos
rot1 = sin*w1/norm
rot2 = sin*w2/norm
rot3 = sin*w3/norm

a0n = a0*rot0 + a1*rot1 + a2*rot2 + a3*rot3
a1n = a0*rot1 + rot0*a1 + (a2*rot3 - a3*rot2)
a2n = a0*rot2 + rot0*a2 + (a3*rot1 - a1*rot3)
a3n = a0*rot3 + rot0*a3 + (a1*rot2 - a2*rot1)

v1n = v1
v2n = v2
v3n = v3

w1n = w1
w2n = w2
w3n = w3

X = sympy.Array([x1, x2, x3, a0, a1, a2, a3, v1, v2, v3, w1, w2, w3])
#S = sympy.Array([v1, v2, v3, w1, w2, w3])
#Xn = sympy.Array([x1n, x2n, x3n, a0n, a1n, a2n, a3n, v1n, v2n, v3n, w1n, w2n, w3n])

#J = sympy.derive_by_array(Xn,X).transpose()
#print(J)

#print(sympy.derive_by_array(a0n, X))
print( sympy.derive_by_array(a0n, [w1, w2, w3]))

