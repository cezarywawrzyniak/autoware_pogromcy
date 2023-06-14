import cv2 as cv
import numpy as np
import yaml
from scipy.spatial.distance import cdist
import sys
import os


for file in os.listdir(str(sys.argv[1])):
    if file.endswith(".pgm"):
        print(os.path.join(str(sys.argv[1]), file))
        map = cv.imread(os.path.join(str(sys.argv[1]), file))
    elif file.endswith(".yaml"):
        with open(os.path.join(str(sys.argv[1]), file), "r") as stream:
            try:
                map_info = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

map_size = map.shape

origin_cv = [- round(map_info['origin'][0] / map_info['resolution']), map_size[0] + round(map_info['origin'][1] / map_info['resolution'])]

_, driveable = cv.threshold(map, int((map_info['free_thresh'] + map_info['occupied_thresh']) * 255), 255, cv.THRESH_BINARY_INV)

contours, hierarchy = cv.findContours(driveable[:,:,0], cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
mx = sorted(contours, key = cv.contourArea)
cv.drawContours(driveable, mx, len(mx)-3, (0,255,0), 3)	
cv.drawContours(driveable, mx, len(mx)-2, (0,255,0), 3)	
driveable_tmp = driveable.copy()
distance = cv.pointPolygonTest(mx[len(mx)-2], (0, 0), True) - cv.pointPolygonTest(mx[len(mx)-3], (0, 0), True)

cv.drawContours(driveable_tmp, mx, len(mx)-2, (255,255,255), int(distance))	
contours, hierarchy = cv.findContours(driveable_tmp[:,:,0], cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
mx = sorted(contours, key = cv.contourArea)
traj_cnt = mx[len(mx)-2]
index = np.where(traj_cnt==origin_cv)

cv.drawContours(driveable, [traj_cnt], -1, (255,233,85), 1)
indices = np.where(np.all(driveable==[255,233,85], axis=-1))
coordinates = np.column_stack((indices[1], indices[0]))
contour_points = coordinates.reshape(coordinates.shape[0], 2)
distances = cdist([origin_cv], contour_points)
closest_index = np.argmin(distances)
closest_point = contour_points[closest_index]

sorted_points = [closest_point]
remaining_points = np.array(coordinates)

while len(sorted_points) < len(coordinates):
    
    distances = cdist([sorted_points[-1]], remaining_points)
    min_index = np.argmin(distances)
    sorted_points.append(remaining_points[min_index])
    remaining_points = np.delete(remaining_points, min_index, axis=0)

trajectory = sorted_points[0::3]
trajectory = [[(x - origin_cv[0]) * map_info['resolution'],  (origin_cv[1] - y) * map_info['resolution']] for [x, y] in trajectory]


with open('trajectory.txt', 'w') as f:
    if trajectory[1][0] < trajectory[0][0]:
        for pt in reversed(trajectory):
            stringi = str(pt[0]) + ' ' + str(pt[1]) + ' 3.0\n'
            f.write(stringi)
    else:
        for pt in trajectory:
            stringi = str(pt[0]) + ' ' + str(pt[1]) + ' 3.0\n'
            f.write(stringi)



