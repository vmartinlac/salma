import sympy
import re

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

a0n = a0*rot0 - a1*rot1 - a2*rot2 - a3*rot3
a1n = a0*rot1 + rot0*a1 + (a2*rot3 - a3*rot2)
a2n = a0*rot2 + rot0*a2 + (a3*rot1 - a1*rot3)
a3n = a0*rot3 + rot0*a3 + (a1*rot2 - a2*rot1)

v1n = v1
v2n = v2
v3n = v3

w1n = w1
w2n = w2
w3n = w3

pre = [x1, x2, x3, a0, a1, a2, a3, v1, v2, v3, w1, w2, w3]
post = [x1n, x2n, x3n, a0n, a1n, a2n, a3n, v1n, v2n, v3n, w1n, w2n, w3n]

print("// Generated automatically by python script system2.py.")
print("// BEGIN")

for i in [3,4,5,6]:
    for j in [3,4,5,6,10,11,12]:
        s = str( post[i].diff(pre[j]) )
        s = s.replace("cos(dt*(w1**2 + w2**2 + w3**2)**0.5/2)", "cos_theta")
        s = s.replace("sin(dt*(w1**2 + w2**2 + w3**2)**0.5/2)", "sin_theta")
        s = s.replace('(w1**2 + w2**2 + w3**2)**(-0.5)', 'norm_w')
        s = re.subn('w(.)\\*w(.)\\*\\(w1\\*\\*2 \\+ w2\\*\\*2 \\+ w3\\*\\*2\\)\\*\\*\\(-1\\.0\\)', 'axis\\1*axis\\2', s)[0]
        s = re.subn('w(.)\\*\\*2\\*\\(w1\\*\\*2 \\+ w2\\*\\*2 \\+ w3\\*\\*2\\)\\*\\*\\(-1\\.0\\)', 'axis\\1*axis\\1', s)[0]
        s = re.subn('w(.)\\*w(.)\\*\\(w1\\*\\*2 \\+ w2\\*\\*2 \\+ w3\\*\\*2\\)\\*\\*\\(-1\\.5\\)\\*sin_theta', 'axis\\1*axis\\2*sin_theta/norm_w', s)[0]
        s = re.subn('w(.)\\*\\*2\\*\\(w1\\*\\*2 \\+ w2\\*\\*2 \\+ w3\\*\\*2\\)\\*\\*\\(-1\\.5\\)\\*sin_theta', 'axis\\1*axis\\1*sin_theta/norm_w', s)[0]
        s = re.subn('w(.)\\*norm_w\\*sin_theta', 'r\\1', s)[0]
        if j <= 6:
            s = s.replace('cos_theta', 'r0')
        s = s.replace('sin_theta/norm_w', 'sin_theta_over_norm_w')

        prefix = "J.insert("+str(i)+","+str(j)+") = "
        suffix = ";"
        print(prefix + s + suffix)

print("// END")
