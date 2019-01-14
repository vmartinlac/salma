import sympy

def make_camera_matrix(prefix):
    fx = sympy.symbols(prefix+'_fx')
    fy = sympy.symbols(prefix+'_fy')
    cx = sympy.symbols(prefix+'_cx')
    cy = sympy.symbols(prefix+'_cy')
    return (fx, 0, cx, 0, fy, cy, 0, 0, 1)

def make_vector2d(prefix):
    vx = sympy.symbols(prefix+'_x')
    vy = sympy.symbols(prefix+'_y')
    return (vx, vy)

def make_vector(prefix):
    vx = sympy.symbols(prefix+'_x')
    vy = sympy.symbols(prefix+'_y')
    vz = sympy.symbols(prefix+'_z')
    return (vx, vy, vz);

def make_quaternion(prefix):
    vx = sympy.symbols(prefix+'_x')
    vy = sympy.symbols(prefix+'_y')
    vz = sympy.symbols(prefix+'_z')
    vw = sympy.symbols(prefix+'_w')
    return (vx, vy, vz, vw);

def make_matrix(prefix):
    M11 = sympy.symbols(prefix+'_11')
    M12 = sympy.symbols(prefix+'_12')
    M13 = sympy.symbols(prefix+'_13')
    M21 = sympy.symbols(prefix+'_21')
    M22 = sympy.symbols(prefix+'_22')
    M23 = sympy.symbols(prefix+'_23')
    M31 = sympy.symbols(prefix+'_31')
    M32 = sympy.symbols(prefix+'_32')
    M33 = sympy.symbols(prefix+'_33')
    return (M11, M12, M13, M21, M22, M23, M31, M32, M33)

def invert_camera_matrix(cam):
    fx = cam[0]
    fy = cam[4]
    cx = cam[2]
    cy = cam[5]
    return (1/fx, 0, -cx/fx, 0, 1/fy, -cy/fy, 0, 0, 1)

def w2q(w):
    wx, wy, wz = w

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

    return qx, qy, qz, qw

def q2r(q):
    qi, qj, qk, qr = q

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

    return (R11, R12, R13, R21, R22, R23, R31, R32, R33)

def add_vectors(a,b):
    return (
        a[0]+b[0],
        a[1]+b[1],
        a[2]+b[2])

def multiply_matrix_vector(a,v):
    vx, vy, vz = v

    ux = a[0]*vx + a[1]*vy + a[2]*vz
    uy = a[3]*vx + a[4]*vy + a[5]*vz
    uz = a[6]*vx + a[7]*vy + a[8]*vz

    return (ux, uy, uz)

def multiply_quaternion_quaternion(p, q):

    pi, pj, pk, pr = p
    qi, qj, qk, qr = q

    ui = pr * qi + qr * pi + (pj*qk - pk*qj)
    uj = pr * qj + qr * pj + (pk*qi - pi*qk)
    uk = pr * qk + qr * pk + (pi*qj - pj*qi)
    ur = pr*qr - pi*qi - pj*qj - pk*qk

    return (ui, uj, uk, ur)

def scalar_product(u,v):
    vx, vy, vz = v
    ux, uy, uz = u
    return vx*ux + vy*uy + vz*uz

def print_function_and_jacobian(outvars, invars, result_name, jacobian_name):

    i=0
    for outv in outvars:
        print(result_name + "(" + str(i) + ") = " + sympy.printing.ccode(outv) + ";")
        i += 1

    i=0
    for outv in outvars:
        j = 0
        for inv in invars:
            txt = sympy.printing.ccode( outv.diff(inv) )
            print(jacobian_name + "(" + str(i) + ", " + str(j) + ") = " + txt + ";")
            j += 1
        i += 1

def rigid_transform_demo():
    q = make_quaternion("q")
    t = make_vector("t")
    v = make_vector("v")
    r = q2r(q)

    u = add_vectors( multiply_matrix_vector(q2r(q), v), t)
    print_function_and_jacobian((*u, *t, *q), (*v, *t, *q), "res", "J")

def compute_direction_demo():

    P1 = make_vector2d("P1")
    P2 = make_vector2d("P2")

    R1 = make_matrix("R1")
    R2 = make_matrix("R2")

    K1 = make_camera_matrix("K1")
    K2 = make_camera_matrix("K2")

    invK1 = invert_camera_matrix(K1)
    invK2 = invert_camera_matrix(K2)

    A1 = multiply_matrix_vector(invK1, (*P1, 1))
    A2 = multiply_matrix_vector(invK2, (*P2, 1))

    D1 = multiply_matrix_vector(R1, A1)
    D2 = multiply_matrix_vector(R2, A2)

    print_function_and_jacobian( (*D1, *D2), (*P1, *P2), "res1", "J1")

def triangulation_demo():

    O1 = make_vector('O1')
    O2 = make_vector('O2')

    D1 = make_vector("D1")
    D2 = make_vector("D2")

    A11 = scalar_product(D1, D1)
    A12 = scalar_product(D1, D2)
    A22 = scalar_product(D2, D2)

    det = A11*A22 - A12*A12

    B11 = A11/det
    B12 = -A12/det
    B22 = A22/det

    delta = ( O2[0] - O1[0], O2[1] - O1[1], O2[2] - O1[2] )

    C1 = scalar_product(D1, delta)
    C2 = scalar_product(D2, delta)

    alpha1 = B11 * C1 + B12 * C2
    alpha2 = B12 * C1 + B22 * C2

    P_x = ( (O1[0] + alpha1 * D1[0]) + (O2[0] - alpha2 * D2[0]) ) / 2
    P_y = ( (O1[1] + alpha1 * D1[1]) + (O2[1] - alpha2 * D2[1]) ) / 2
    P_z = ( (O1[2] + alpha1 * D1[2]) + (O2[2] - alpha2 * D2[2]) ) / 2

    outvars = (P_x, P_y, P_z)
    invars = (*D1, *D2)
    print_function_and_jacobian(outvars, invars, "res2", "J2")

def q2r_demo():
    q = make_quaternion("q")
    r = q2r(q)
    print_function_and_jacobian( r, q, "res", "J" )

def w2q_demo():
    w = make_vector("w")
    q = w2q(w)
    print_function_and_jacobian( q, w, "res", "J")

#compute_direction_demo()
#triangulation_demo()
q2r_demo()

