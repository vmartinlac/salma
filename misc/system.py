import sympy

x1, x2, x3 = sympy.symbols(('x1', 'x2', 'x3'))
a0, a1, a2, a3 = sympy.symbols(('a0', 'a1', 'a2', 'a3'))
v1, v2, v3 = sympy.symbols(('v1', 'v2', 'v3'))
w1, w2, w3 = sympy.symbols(('w1', 'w2', 'w3'))

dt = sympy.symbols('dt')

x1n = x1 + dt*v1
x2n = x2 + dt*v2
x3n = x3 + dt*v3

a0tmp = a0 + 0.5*dt*( -w1*a1 - w2*a2 - w3*a3 )
a1tmp = a1 + 0.5*dt*( a0*w1 + w2*a3 - w3*a2 )
a2tmp = a2 + 0.5*dt*( a0*w2 + w3*a1 - w1*a3 )
a3tmp = a3 + 0.5*dt*( a0*w3 + w1*a2 - w2*a1 )

normalize = True
if normalize:
    norm = (a1tmp**2 + a2tmp**2 + a3tmp**2 + a0tmp**2)**0.5
    a0n = a0tmp / norm
    a1n = a1tmp / norm
    a2n = a2tmp / norm
    a3n = a3tmp / norm
else:
    a0n = a0tmp
    a1n = a1tmp
    a2n = a2tmp
    a3n = a3tmp

v1n = v1
v2n = v2
v3n = v3

w1n = w1
w2n = w2
w3n = w3

X = sympy.Array([x1, x2, x3, a0, a1, a2, a3, v1, v2, v3, w1, w2, w3])
S = sympy.Array([v1, v2, v3, w1, w2, w3])
Xn = sympy.Array([x1n, x2n, x3n, a0n, a1n, a2n, a3n, v1n, v2n, v3n, w1n, w2n, w3n])

#J = sympy.derive_by_array(Xn,X).transpose()
#print(J)

#print(sympy.derive_by_array(a0n, X))
print(sympy.derive_by_array(a0n, [a0]))

