import vtk
import numpy
import numpy.linalg
import os

def numpy2vtk(T_from):
    T_to = vtk.vtkMatrix4x4()
    T_to.Identity()
    for i in range(4):
        for j in range(4):
            T_to.SetElement(i, j, T_from[i,j])
    return T_to

pattern = vtk.vtkPNGReader()
pattern.SetFileName("pattern.png")

texture = vtk.vtkTexture()
texture.SetInputConnection(pattern.GetOutputPort())

plane = vtk.vtkPlaneSource()
plane.SetOrigin(0, 0, 0)
plane.SetPoint1(10, 0, 0)
plane.SetPoint2(0, 7, 0)

mapper = vtk.vtkPolyDataMapper()
mapper.SetInputConnection( plane.GetOutputPort() )

actor = vtk.vtkActor()
actor.SetMapper(mapper)
actor.SetTexture(texture)

ren = vtk.vtkRenderer()
ren.AddActor(actor)
ren.SetBackground(1.0, 1.0, 1.0)

win = vtk.vtkRenderWindow()
win.AddRenderer(ren)
win.SetOffScreenRendering(True)
win.SetSize(1024, 768)

f = vtk.vtkWindowToImageFilter()
f.SetInput(win)

writer = vtk.vtkPNGWriter()
writer.SetInputConnection( f.GetOutputPort() )

state = numpy.ndarray((13,))

# position
state[0] = 5.0
state[1] = 3.5
state[2] = 20.0

# attitude
state[3] = 1.0
state[4] = 0.0
state[5] = 0.0
state[6] = 0.0
#state[3] = numpy.cos(0.5*numpy.pi*0.11)
#state[4] = numpy.sin(0.5*numpy.pi*0.11)
#state[1] = -7

# linear velocity
state[7] = 0.0
state[8] = 0.0
state[9] = 1.0

# angular velocity
state[10] = 0.0
state[11] = 0.0
state[12] = 0.0

win.Start()

dt = 1.0/30.0

frame_id = 0

os.system("rm frames/*.png")

while True:

    if frame_id*dt >= 5.0:
        break

    # sync camera with mechanical state.

    a0 = state[3]
    a1 = state[4]
    a2 = state[5]
    a3 = state[6]

    R = numpy.ndarray((3,3))
    R[0,0] = 1.0 - 2.0*(a2*a2 + a3*a3)
    R[0,1] = 2.0*(a1*a2 - a3*a0)
    R[0,2] = 2.0*(a1*a3 + a2*a0)
    R[1,0] = 2.0*(a1*a2 + a0*a3)
    R[1,1] = 1.0 - 2.0*(a1*a1 + a3*a3)
    R[1,2] = 2.0*(a2*a3 - a1*a0)
    R[2,0] = 2.0*(a1*a3 - a0*a2)
    R[2,1] = 2.0*(a2*a3 + a1*a0)
    R[2,2] = 1.0 - 2.0*(a1*a1 + a2*a2)

    T = numpy.ndarray((4,4))
    T[:,3] = 0.0
    T[3,:] = 0.0
    T[3,3] = 1.0
    T[:3,:3] = R.T
    T[:3,3] = -R.T.dot(state[:3])
    T[3,:] = 0.0
    T[3,3] = 1.0

    ren.GetActiveCamera().SetModelTransformMatrix( numpy2vtk(T) )

    #print("Frame #"+str(frame_id))
    #print(state[:3])
    #print()

    # produce the image and save it.

    win.Render()

    f.Modified()
    writer.SetFileName("frames/frame_"+str(frame_id).zfill(4)+".png")
    writer.Write()

    frame_id += 1

    # update state.

    state[0:3] += dt*state[7:10]

    theta = numpy.linalg.norm( state[10:13] )
    if abs(theta) > 1.0e-7:
        b = numpy.ndarray((4,))
        b[0] = numpy.cos(0.5*theta)
        b[1:4] = state[10:13] * numpy.sin(0.5*theta) / theta

        a = state[3:7]

        state[3] = a[0]*b[0] - a[1:].dot(b[1:])
        state[4:7] = a[0]*b[1:] + b[0]*a[1:] + numpy.cross(a[1:], b[1:])
        state[3:7] /= numpy.linalg.norm( state[3:] )

w,h = win.GetSize()
print("cx = ", 0.5*w)
print("cy = ", 0.5*h)
f = 0.5*h / numpy.tan( ren.GetActiveCamera().GetViewAngle()*0.5*numpy.pi/180.0 )
print("fx = ", f)
print("fy = ", f)

