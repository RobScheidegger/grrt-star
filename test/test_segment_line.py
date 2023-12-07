import math
import matplotlib.pyplot as plt

def segment_line(start=(1, 3, -10), end=(23, 7, -20), step=2):
    distance = step
    line_length = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2 + (end[2] - start[2])**2)
    
    coords = []
    coords.append(start)
    
    while distance < line_length:
        a = distance
        b = line_length - distance
        
        x_delta = a / (a + b) * (end[0] - start[0])
        y_delta = a / (a + b) * (end[1] - start[1])
        z_delta = a / (a + b) * (end[2] - start[2])
        
        next_coords = (start[0] + x_delta, start[1] + y_delta, start[2] + z_delta)
        assert(math.isclose(math.sqrt((next_coords[0] - coords[-1][0])**2 + (next_coords[1] - coords[-1][1])**2 + (next_coords[2] - coords[-1][2])**2), step))
        
        print(next_coords)
        
        coords.append(next_coords)
        
        distance += step
        
    return coords

ans = segment_line()

# Plotting the coordinates in a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x = [coord[0] for coord in ans]
y = [coord[1] for coord in ans]
z = [coord[2] for coord in ans]

ax.scatter(x, y, z)

plt.show()

