import os
import pyvista as pv
import numpy as np
import math

class Plot():

    def __init__(self, object='iris', fps = 20, objectColor = 'red'):
        self.ground = pv.Plane(i_size=100, j_size=100)

        self.camera = pv.Camera()
        self.cameraOffSet = (25.0, 25.0, 25.0)
        self.cameraView = 'fixed'
        self.camera.position = (25.0, 25.0, 25.0)
        self.camera.focal_point = (0.0, 0.0, 0.0)
        self.camera.roll = -120.0

        self.object = pv.read('./Simulation/'+object+'.stl')
        self.object.points *= 0.03

        self.object.x = 0
        self.object.y = 0
        self.object.z = 0
        self.object.roll = 0
        self.object.pitch = 0
        self.object.yaw = 0
        self.object.rotationAxis = (0,0,1)
        self.object.rotationAngle = 0
        self.object.rotationOffset = (0,0,0)

        self.axes = pv.Axes(show_actor=True, actor_scale=1.0, line_width=5)
        self.axes.origin = (0.0, 0.0, 0.0)
        self.axes.actor.rotate_x(180)

        self.plot = pv.Plotter()  # 캔버스 정의

        self.plot.add_actor(self.axes.actor)
        self.plot.add_mesh(self.ground, color='grey')
        self.plot.add_mesh(self.object, opacity=1, color=objectColor)
        self.plot.camera = self.camera

        self.plot.open_gif('move.gif', fps=fps)


    def RotationAxis(self, roll, pitch, yaw):
        R = np.zeros((3, 3))

        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(-pitch)
        sp = np.sin(-pitch)
        cy = np.cos(-yaw)
        sy = np.sin(-yaw)

        R[0, 0] = cy * cp
        R[0, 1] = -sy * cr + cy * sp * sr
        R[0, 2] = sy * sr + cy * sp * cr

        R[1, 0] = sy * cp
        R[1, 1] = cy * cr + sy * sp * sr
        R[1, 2] = -cy * sr + sy * sp * cr

        R[2, 0] = -sp
        R[2, 1] = cp * sr
        R[2, 2] = cp * cr

        R = R.T

        q4 = 0.5 * np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2])
        q1 = 0.25 * (R[1, 2] - R[2, 1]) / q4
        q2 = 0.25 * (R[2, 0] - R[0, 2]) / q4
        q3 = 0.25 * (R[0, 1] - R[1, 0]) / q4

        theta = 2 * math.acos(q4)
        e = (q1 / (0.001 + np.sin(theta / 2)), q2 / (0.001 + np.sin(theta / 2)), q3 / (0.001 + np.sin(theta / 2)))

        return e, theta

    def NED2PlotXyz(self, N, E, D, roll, pitch, yaw):
        self.object.x = N
        self.object.y = -E
        self.object.z = -D
        self.object.roll = roll + self.object.rotationOffset[0]
        self.object.pitch = pitch
        self.object.yaw = yaw

    def UpdatePlot(self, N, E, D, roll, pitch, yaw):

        self.object.translate((-self.object.x, -self.object.y, -self.object.z), inplace=True)
        self.object.rotate_vector(vector=self.object.rotationAxis, angle=-self.object.rotationAngle * 180 / np.pi, inplace=True)

        self.NED2PlotXyz(N, E, D, roll, pitch, yaw)

        self.object.rotationAxis, self.object.rotationAngle = self.RotationAxis(self.object.roll, self.object.pitch,self.object.yaw)
        self.object.translate((self.object.x, self.object.y, self.object.z), inplace=True)
        self.object.rotate_vector(vector=self.object.rotationAxis, angle=self.object.rotationAngle * 180 / np.pi,point=(self.object.x, self.object.y, self.object.z), inplace=True)


        self.UpdateCamera()
        self.plot.write_frame()
        self.plot.render()

    def UpdateCamera(self):
        if(self.cameraView == 'Chase'):
            self.plot.camera.position = (self.object.x + self.cameraOffSet[0], self.object.y + self.cameraOffSet[1],self.object.z + self.cameraOffSet[2])
            self.plot.camera.focal_point = (self.object.x, self.object.y, self.object.z)
        else:
            self.plot.camera.focal_point = (self.object.x, self.object.y, self.object.z)

    def EndPlot(self):
        self.plot.close()



def EulerDCM(roll, pitch, yaw):
    R       =   np.zeros((3,3))

    cr      =   np.cos(roll)
    sr      =   np.sin(roll)
    cp      =   np.cos(pitch)
    sp      =   np.sin(pitch)
    cy      =   np.cos(yaw)
    sy      =   np.sin(yaw)

    R[0, 0] = cy*cp
    R[0, 1] = -sy*cr + cy*sp*sr
    R[0, 2] = sy*sr + cy*sp*cr

    R[1, 0] = sy*cp
    R[1, 1] = cy*cr + sy*sp*sr
    R[1, 2] = -cy*sr + sy*sp*cr

    R[2, 0] = -sp
    R[2, 1] = cp*sr
    R[2, 2] = cp*cr

    return R

def DCM2RotationAxis(R):
    q4 = 0.5*np.sqrt(1+R[0,0] + R[1,1] + R[2,2])
    q1 = 0.25*(R[1,2]-R[2,1])/q4
    q2 = 0.25*(R[2,0]-R[0,2])/q4
    q3 = 0.25*(R[0,1]-R[1,0])/q4

    theta = 2*math.acos(q4)
    e = (q1/(0.001+np.sin(theta/2)), q2/(0.001+np.sin(theta/2)), q3/(0.001+np.sin(theta/2)))

    return e, theta


# ground = pv.Plane(i_size = 10, j_size = 10)
# ductedfanModel = pv.read('ductedfan.stl')
# ductedfanModel.points *=0.03
# # ductedfanModel.flip_z()
# # ductedfanModel.flip_y()
# # ductedfanModel.flip_x()
#
# camera = pv.Camera()
# camera.position = (18.0, 18.0, 18.0)
# camera.focal_point = (0.0, 0.0, 0.0)
# camera.roll = -120.0
#
# axes = pv.Axes(show_actor=True, actor_scale=2.0, line_width=5)
# axes.origin = (0.0, 0.0, 0.0)
#
# p = pv.Plotter() # 캔버스 정의
#
# p.add_actor(axes.actor)
# p.add_mesh(ground, color='grey')
# p.add_mesh(ductedfanModel, opacity=1, color='red')
# p.camera = camera
# phi = 0
# e = (0,0,1)
# R = EulerDCM(0, 00 * np.pi / 180, -90 * np.pi / 180).T
# # phi, e = DCM2RotationAxis(R)
# # ductedfanModel.translate((1,1,2),inplace=True)
# # ductedfanModel.rotate_vector(vector=e, angle=phi*180/np.pi, point = (1,1,2), inplace=True)
# # p = Plot()
#
# # p.UpdatePlot(1,1,-2,0,0,np.pi/2)
# p.show()
# p.open_gif('move.gif', fps=20)
# for i in range(100):
#     ductedfanModel.rotate_vector(vector=e, angle=-phi * 180 / np.pi, inplace=True)
#
#     if (i<=70):
#         R = EulerDCM(0,0,-i*np.pi/180).T
#         phi, e = DCM2RotationAxis(R)
#
#     else:
#         R = EulerDCM(0, -(i-70) * np.pi / 180, -70 * np.pi / 180).T
#         phi, e = DCM2RotationAxis(R)
#
#     ductedfanModel.rotate_vector(vector=e, angle=phi*180/np.pi, inplace=True)
#     p.write_frame()
# p.close()

# R = EulerDCM(0,30*np.pi/180,90*np.pi/180).T
# phi, e = DCM2RotationAxis(R)
# ductedfanModel.rotate_vector(vector=e, angle=phi*180/np.pi, inplace=True)

# ductedfanModel.rotate_z(-60,point = (0,0,0), inplace=True)
# ductedfanModel.rotate_z(-30, point=(0,0,0), inplace=True)
# p.open_gif('move.gif',fps=20)
# for i in range(100):
#     # ductedfanModel.translate([0,0,0.005*i],inplace=True)
#     ductedfanModel.rotate_x(0.5*np.sin(2*np.pi*0.02*i), point = (0,0,0), inplace=True)
#     # p.camera.direction = (-0.57, -0.57+0.05*i, -0.57)
#     p.camera_set = True
#     p.write_frame()
# p.close()