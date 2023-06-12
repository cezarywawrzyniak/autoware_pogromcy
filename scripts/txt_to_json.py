import json

with open('trajectory.txt') as f:
    lines = f.readlines()

coordinates_list = []
for i in lines:
    #     print(i.split())
    point_dict = {"X": float(i.split()[0]), "Y": float(i.split()[1])}
    coordinates_list.append(point_dict)

dic = dict()
dic = {"coords": coordinates_list}

out_file = open("trajectory.json", "w")
json.dump(dic, out_file, indent=4, sort_keys=False)
out_file.close()





