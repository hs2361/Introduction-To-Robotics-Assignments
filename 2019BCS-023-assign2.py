# Name: Harsh Sharma
# Roll Number: 2019BCS-023
import numpy as np

coordinates = np.array(list(map(float, input(
    "Enter coordinates of initial"
    " point separated by spaces: ").split()))).reshape((3, 1))

angleX = np.deg2rad(float(input("Enter rotation about X axis: ")))
angleY = np.deg2rad(float(input("Enter rotation about Y axis: ")))
angleZ = np.deg2rad(float(input("Enter rotation about Z axis: ")))

rotationX = np.array([[1, 0, 0],
                      [0, np.cos(angleX), -np.sin(angleX)],
                      [0, np.sin(angleX), np.cos(angleX)]])

rotationY = np.array([[np.cos(angleY), 0, np.sin(angleY)],
                      [0, 1, 0],
                      [-np.sin(angleY), 0, np.cos(angleY)]])

rotationZ = np.array([[np.cos(angleZ),  -np.sin(angleZ), 0],
                      [np.sin(angleZ), np.cos(angleZ), 0],
                      [0, 0, 1]])

rotation = ((rotationX @ rotationY) @ rotationZ)
print("New coordinates: ", rotation@coordinates)
