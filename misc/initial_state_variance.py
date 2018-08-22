import sympy
import numpy

alpha, theta, phi = sympy.symbols(('alpha', 'theta', 'phi'))

x = sympy.sin(phi) * sympy.cos(theta)
y = sympy.sin(phi) * sympy.sin(theta)
z = sympy.cos(phi)

a0 = sympy.cos(0.5*alpha)
a1 = x*sympy.sin(0.5*alpha)
a2 = y*sympy.sin(0.5*alpha)
a3 = z*sympy.sin(0.5*alpha)

J = numpy.ndarray( (4,3) )
dico = {theta:0, phi:sympy.pi*0.5, alpha:sympy.pi*0.5}
output_list = [a0, a1, a2, a3]
input_list = [alpha, theta, phi]
for i in range(4):
    for j in range(3):
        derivative = output_list[i].diff(input_list[j]).subs(dico)
        J[i,j] = derivative.evalf()

delta = numpy.pi*2.0/180.0
print(J.dot(numpy.array([delta, delta, delta])))
