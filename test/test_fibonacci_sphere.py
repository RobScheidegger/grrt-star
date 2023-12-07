import math
import matplotlib.pyplot as plt

def fibonacci_sphere(samples=1000, sphere_radius=2, x_off=-2, y_off=10, z_off=-15):

    points = []
    phi = math.pi * (math.sqrt(5.) - 1.)  # golden angle in radians

    for i in range(samples):
        y = 1 - (i / float(samples - 1)) * 2  # y goes from 1 to -1
        radius = math.sqrt(1 - y * y)  # radius at y
        # print(radius)

        theta = phi * i  # golden angle increment

        x = math.cos(theta) * radius
        z = math.sin(theta) * radius

        points.append(((x * sphere_radius) + x_off, (y * sphere_radius) + y_off, (z * sphere_radius) + z_off))

    return points

ans = fibonacci_sphere()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

xs = [point[0] for point in ans]
ys = [point[1] for point in ans]
zs = [point[2] for point in ans]

ax.scatter(xs, ys, zs)

plt.show()
