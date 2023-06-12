import matplotlib.pyplot as plt

x_coords = []
y_coords = []

with open('trajectory.txt', 'r') as file:
    lines = file.readlines()

for i, line in enumerate(lines):
    values = line.strip().split(' ')
    x = float(values[0])
    y = float(values[1])
    x_coords.append(x)
    y_coords.append(y)
    if i % 10 == 0:
        plt.text(x, y, str(i), ha='center', va='center', fontsize=12, weight='bold')

plt.scatter(x_coords, y_coords)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Scatter Plot of X-Y Coordinates')
plt.show()
