import sympy

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

def q2r(q):
    qr, qi, qj, qk = q

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

def multiply_matrix_vector(a,v):
    vx, vy, vz = v

    ux = a[0]*vx + a[1]*vy + a[2]*vz
    uy = a[3]*vx + a[4]*vy + a[5]*vz
    uz = a[6]*vx + a[7]*vy + a[8]*vz

    return (ux, uy, uz)

def scalar_product(u,v):
    vx, vy, vz = v
    ux, uy, uz = u
    return vx*ux + vy*uy + vz*uz

#XL = make_vector("xleft")
#XR = make_vector("xright")

leftcam2world_t = make_vector('O1')
#leftcam2world_q = make_quaternion('leftcam2world_q')
rightcam2world_t = make_vector('O2')
#rightcam2world_q = make_quaternion('rightcam2world_q')

#leftcam2world_r = make_matrix("leftcam2world_r") #q2r(leftcam2world_q)
#rightcam2world_r = make_matrix("rightcam2world_r") #q2r(rightcam2world_q)

#left_dir = multiply_matrix_vector(leftcam2world_r, XL)
#right_dir = multiply_matrix_vector(rightcam2world_r, XR)

left_dir = make_vector("D1")
right_dir = make_vector("D2")

A11 = scalar_product(left_dir, left_dir)
A12 = scalar_product(left_dir, right_dir)
A22 = scalar_product(right_dir, right_dir)

det = A11*A22 - A12*A12

B11 = A11/det
B12 = -A12/det
B22 = A22/det

delta = ( rightcam2world_t[0] - leftcam2world_t[0], rightcam2world_t[1] - leftcam2world_t[1], rightcam2world_t[2] - leftcam2world_t[2] )

C1 = scalar_product(left_dir, delta)
C2 = scalar_product(right_dir, delta)

alpha1 = B11 * C1 + B12 * C2
alpha2 = B12 * C1 + B22 * C2

P_x = ( (leftcam2world_t[0] + alpha1 * left_dir[0]) + (rightcam2world_t[0] - alpha2 * right_dir[0]) ) / 2
P_y = ( (leftcam2world_t[1] + alpha1 * left_dir[1]) + (rightcam2world_t[1] - alpha2 * right_dir[1]) ) / 2
P_z = ( (leftcam2world_t[2] + alpha1 * left_dir[2]) + (rightcam2world_t[2] - alpha2 * right_dir[2]) ) / 2

outvars = (P_x, P_y, P_z)
invars = (*left_dir, *leftcam2world_t, *right_dir, *rightcam2world_t)
i = 0
for outv in outvars:
    j = 0
    for inv in invars:
        txt = sympy.printing.ccode( outv.diff(inv) )
        print("J("+str(i)+", "+str(j)+") = " + txt + ";")
        j += 1
    print()
    i += 1

