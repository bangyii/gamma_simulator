#!/usr/bin/env python
import cv2
import matplotlib.pyplot as plt
import sys

if len(sys.argv) != 3:
    print("Please specify map file to obtain obstacles from and specify file to write obstacles, example: ")
    print("python3 map_to_obstacles.py /home/user/map.pgm /home/user/obstacles.txt")
    exit()

print("Open map file: " + sys.argv[1])
print("Write obstacles to file: " + sys.argv[2])
map_file = sys.argv[1]
obstacle_file = sys.argv[2]

image = cv2.imread(map_file)
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Adjust map image to be correct side up to reflect real world. Rotate right, mirror, then flip
# image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
image = cv2.flip(image, 0)
# image = cv2.flip(image, 1)

f = open(obstacle_file, "w+")
# contents = f.readlines() # Returns array of lines, each index is a line
contents = []

# # Find scenario tag line, then insert obstacles at the following line
# obstacle_insert_line = 0 # Insert index is the index that the inserted object will be
# for i in range(len(contents)):
#     if contents[i].find('<scenario>') != -1:
#         obstacle_insert_line = i + 1
#         break

# # Add new line for clarity in new XML file
# contents.insert(obstacle_insert_line, '\n')

# Image processing & finding contours
gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
_, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
element = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Set origin and resolution of obstacles
x_origin = -12.5
y_origin = -35
resolution = 0.05
for i in range(len(contours)):
    if(len(contours[i]) < 2):
        continue

    contour_points = ""

    for j in range(len(contours[i])):
        x1 = contours[i][j][0][0] * resolution + x_origin
        y1 = contours[i][j][0][1] * resolution + y_origin
        contour_points += str(x1) + ',' + str(y1)
        if j < (len(contours[i]) - 1):
            contour_points += ':'

        # x2 = contours[i][j+1][0][0] * resolution + x_origin
        # y2 = contours[i][j+1][0][1] * resolution + y_origin
        # contents.insert(obstacle_insert_line, '\t<obstacle x1="' + str(x1) + '" y1="' + str(y1) + '" x2="' + str(x2) + '" y2="' + str(y2) + '"/>\n')

    contour_points += '\n'
    contents.append(contour_points)

# Seek back to start of file and then override with new content
f.seek(0)
f.writelines(contents)
f.truncate()
f.close()

# # Display image showing contours
# image = cv2.drawContours(image, contours, -1, (0,255,0), 2)
# plt.imshow(image)
# plt.show()
