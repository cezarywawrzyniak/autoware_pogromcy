def modify_third_number(file_path, line_range, new_value):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    for i, line in enumerate(lines):
        if i + 1 in line_range:
            values = line.strip().split(' ')
            if len(values) >= 3:
                values[2] = str(new_value)
                lines[i] = ' '.join(values) + '\n'

    with open(file_path, 'w') as file:
        file.writelines(lines)


file_path = 'trajectory.txt'
# line_range = range(0, 1655)  # range() is inclusive
line_range_1 = range(0, 201)
new_speed_1 = 3.0
line_range_2 = range(200, 301)
new_speed_2 = 1.0
line_range_3 = range(300, 411)
new_speed_3 = 3.0
line_range_4 = range(410, 521)
new_speed_4 = 1.0
line_range_5 = range(520, 561)
new_speed_5 = 1.0
line_range_6 = range(560, 721)
new_speed_6 = 3.0
line_range_7 = range(720, 801)
new_speed_7 = 1.0
line_range_8 = range(800, 881)
new_speed_8 = 3.0
line_range_9 = range(880, 931)
new_speed_9 = 1.0
line_range_10 = range(930, 1341)
new_speed_10 = 3.0
line_range_11 = range(1340, 1421)
new_speed_11 = 1.0
line_range_12= range(1420, 1655)
new_speed_12 = 3.0

modify_third_number(file_path, line_range_1, new_speed_1)
modify_third_number(file_path, line_range_2, new_speed_2)
modify_third_number(file_path, line_range_3, new_speed_3)
modify_third_number(file_path, line_range_4, new_speed_4)
modify_third_number(file_path, line_range_5, new_speed_5)
modify_third_number(file_path, line_range_6, new_speed_6)
modify_third_number(file_path, line_range_7, new_speed_7)
modify_third_number(file_path, line_range_8, new_speed_8)
modify_third_number(file_path, line_range_9, new_speed_9)
modify_third_number(file_path, line_range_10, new_speed_10)
modify_third_number(file_path, line_range_11, new_speed_11)
modify_third_number(file_path, line_range_12, new_speed_12)
