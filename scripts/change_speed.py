import os


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


def remove_second_line(file_path):
    # Extract file name and extension
    file_name, file_extension = os.path.splitext(file_path)

    # Construct new file name with "shortened" added
    new_file_name = file_name + "_shortened" + file_extension

    with open(file_path, 'r') as file:
        lines = file.readlines()

    modified_lines = lines[::2]  # Select every second line

    with open(new_file_name, 'w') as file:
        file.writelines(modified_lines)


file_path = 'trajectory_driven.txt'
# line_range = range(0, 1630)  # range() is inclusive
line_range_1 = range(0, 176)
new_speed_1 = 3.0
line_range_2 = range(175, 276)
new_speed_2 = 2.8
line_range_3 = range(275, 386)
new_speed_3 = 5.0
line_range_4 = range(385, 446)
new_speed_4 = 3.0
line_range_5 = range(445, 486)
new_speed_5 = 3.5
line_range_6 = range(485, 556)
new_speed_6 = 2.5
line_range_7 = range(555, 696)
new_speed_7 = 3.5
line_range_8 = range(695, 776)
new_speed_8 = 3.0
line_range_9 = range(775, 856)
new_speed_9 = 3.5
line_range_10 = range(855, 926)
new_speed_10 = 3.0
line_range_11 = range(925, 1116)
new_speed_11 = 4.0
line_range_12 = range(1115, 1316)
new_speed_12 = 5.0
line_range_13 = range(1315, 1441)
new_speed_13 = 2.7
line_range_14 = range(1440, 1630)
new_speed_14 = 2.8
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
modify_third_number(file_path, line_range_13, new_speed_13)
modify_third_number(file_path, line_range_14, new_speed_14)
remove_second_line(file_path)

file_path = 'trajectory_planned.txt'
# line_range = range(0, 1336)  # range() is inclusive
line_range_1 = range(0, 161)
new_speed_1 = 3.0
line_range_2 = range(160, 221)
new_speed_2 = 2.8
line_range_3 = range(220, 351)
new_speed_3 = 5.0
line_range_4 = range(350, 401)
new_speed_4 = 3.0
line_range_5 = range(400, 441)
new_speed_5 = 3.5
line_range_6 = range(440, 496)
new_speed_6 = 2.5
line_range_7 = range(495, 621)
new_speed_7 = 3.5
line_range_8 = range(620, 641)
new_speed_8 = 3.0
line_range_9 = range(640, 761)
new_speed_9 = 3.5
line_range_10 = range(760, 801)
new_speed_10 = 3.0
line_range_11 = range(800, 941)
new_speed_11 = 4.0
line_range_12 = range(940, 1101)
new_speed_12 = 5.0
line_range_13 = range(1100, 1151)
new_speed_13 = 2.7
line_range_14 = range(1150, 1336)
new_speed_14 = 2.8
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
modify_third_number(file_path, line_range_13, new_speed_13)
modify_third_number(file_path, line_range_14, new_speed_14)
remove_second_line(file_path)
