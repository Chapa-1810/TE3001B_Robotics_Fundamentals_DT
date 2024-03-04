import numpy as np
import time

def jacobian(dennavit_harten):
  jacobian = np.zeros((6, 3))
  homogen = np.zeros((4, 4))
  rots = []
  ds = []

  for i in range(3):
    homogen = np.array([[np.cos(dennavit_harten[i, 0]), -np.sin(dennavit_harten[i, 0])*np.cos(dennavit_harten[i, 1]), np.sin(dennavit_harten[i, 0])*np.sin(dennavit_harten[i, 1]), dennavit_harten[i, 2]*np.cos(dennavit_harten[i, 0])],
                        [np.sin(dennavit_harten[i, 0]), np.cos(dennavit_harten[i, 0])*np.cos(dennavit_harten[i, 1]), -np.cos(dennavit_harten[i, 0])*np.sin(dennavit_harten[i, 1]), dennavit_harten[i, 2]*np.sin(dennavit_harten[i, 0])],
                        [0, np.sin(dennavit_harten[i, 1]), np.cos(dennavit_harten[i, 1]), dennavit_harten[i, 3]],
                        [0, 0, 0, 1]])
    # Grab rotation matrix from homogen matrix
    rots.append(homogen[:3, :3])
    # Grab displacement from homogen matrix
    ds.append(homogen[:3, 3])

  # Calculate jacobian
  for i in range(3):
    jacobian[:3, i] = np.cross(rots[i][:, 2], ds[2] - ds[i])
    jacobian[3:, i] = rots[i][:, 2]

  return jacobian 


# Dennavit-Hartenberg parameters for RRR robot
a1 = 10
a2 = 40
a3 = 20
alpha1 = 1.57
alpha2 = 0
alpha3 = 0
theta1 = 0
theta2 = 0
theta3 = 0
d1 = 0
d2 = 0
d3 = 0

dennavit_harten = np.array([[theta1, alpha1, a1, d1],
                            [theta2, alpha2, a2, d2],
                            [theta3, alpha3, a3, d3]])

#Test jacobian in range of theta1
np.set_printoptions(suppress=True)
theta = 0

# test jacobian for a circular path
x = 0
y = 0
z = 0

x = 0
y = 0
z = 0
wx = 0
wy = 0
wz = 0

velocities_matrix = np.array([x, y, z, wx, wy, wz])
start_time = time.time()

while time.time() - start_time < 10:
  dennavit_harten[0, 0] = theta
  inverse_jacobian = np.linalg.pinv(jacobian(dennavit_harten))
  print(inverse_jacobian)
  print("------")
  q1, q2, q3 = np.dot(inverse_jacobian, velocities_matrix)
  vx = np.sin(time.time() - start_time)
  vy = np.cos(time.time() - start_time)
  vz = 0
  velocities_matrix = np.array([vx, vy, vz, wx, wy, wz])
  print(q1, q2, q3)
  print("------")
  time.sleep(0.1)
  

"""
while theta < 6.28:
  dennavit_harten[0, 0] = theta
  inverse_jacobian = np.linalg.pinv(jacobian(dennavit_harten))
  
  #print(inverse_jacobian)
  #print("------")"""