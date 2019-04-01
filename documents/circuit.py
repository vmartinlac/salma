def demo1():
    C = 1.0e-6 # 1 micro farad
    f = 25 # 25 Hz
    gamma = 0.5 # th / (th + tl)


    tmp = 1.0/(f * 0.693 * C) # tmp = RA + 2RB
    RA = tmp * gamma / (2-gamma)
    RB = tmp * (1-gamma) / (2-gamma)

    print("C = " + str(C))
    print("RA = " + str(RA))
    print("RB = " + str(RB))

def demo2():
    RA = 20000
    RB = 20000
    C = 1.0e-6
    f = 1.0/(0.693*(RA+2*RB)*C)
    print("f = " + str(f))

demo2()


# 2 résistances de 20 000 ohm
# 1 capacité de 1.0e-6 farad

# 1 résistance de 1 000 ohm
# 1 capacité de 1.0e-8 farad

