#!/usr/bin/env python
import sys

if len(sys.argv) != 3:
    print("Please input .world file to modify, and number of actors to add")
    print("python3 generate_actor_xml.py /home/user/gazebo.world 30")
    exit()

world_file = sys.argv[1]
num_agents = int(sys.argv[2])

f = open(world_file, "r+")
contents = f.readlines()

# Find world tag
actor_insert_line = 0
for i in range(len(contents)):
    if contents[i].find('<world') != -1:
        actor_insert_line = i + 1
        break

for i in range(num_agents):
    actor_name = "walking" + str(i)
    contents.insert(actor_insert_line, '\t\t</actor>\n')
    contents.insert(actor_insert_line, '\t\t\t</plugin>\n')
    contents.insert(actor_insert_line, '\t\t\t\t</link>\n')
    contents.insert(actor_insert_line, '\t\t\t\t\t</model>\n')
    contents.insert(actor_insert_line, '\t\t\t\t\t\t<model_name>' + actor_name + '_collision_model</model_name>\n')
    contents.insert(actor_insert_line, '\t\t\t\t\t<model>\n')
    contents.insert(actor_insert_line, '\t\t\t\t\t<link_name>' + actor_name + '_pose</link_name>\n')
    contents.insert(actor_insert_line, '\t\t\t\t<link>\n')
    contents.insert(actor_insert_line, '\t\t\t<plugin name="actor' + str(i) + '_model_plugin" filename="libattach_collision_plugin.so">\n')
    contents.insert(actor_insert_line, '\t\t\t</plugin>\n')
    contents.insert(actor_insert_line, '\t\t\t\t<actor_id>' + str(i) + '</actor_id>\n')
    contents.insert(actor_insert_line, '\t\t\t<plugin name="actor' + str(i) + '_plugin" filename="libactor_plugin.so">\n')
    contents.insert(actor_insert_line, '\t\t\t</animation>\n')
    contents.insert(actor_insert_line, '\t\t\t\t<interpolate_x>true</interpolate_x>\n')
    contents.insert(actor_insert_line, '\t\t\t\t<filename>walk.dae</filename>\n')
    contents.insert(actor_insert_line, '\t\t\t<animation name="walking">\n')
    contents.insert(actor_insert_line, '\t\t\t</skin>\n')
    contents.insert(actor_insert_line, '\t\t\t\t<filename>walk.dae</filename>\n')
    contents.insert(actor_insert_line, '\t\t\t<skin>\n')
    # contents.insert(actor_insert_line, '\t\t\t<pose>' + str(i) + ' 0 1.0 0 0 0</pose>\n')
    contents.insert(actor_insert_line, '\t\t\t<pose>0 0 ' + str(-i-5) + ' 0 0 0</pose>\n')
    contents.insert(actor_insert_line, '\t\t<actor name="' + actor_name + '">\n')
    contents.insert(actor_insert_line, '\t\t</model>\n')
    contents.insert(actor_insert_line, '\t\t\t</link>\n')
    contents.insert(actor_insert_line, '\t\t\t\t</collision>\n')
    contents.insert(actor_insert_line, '\t\t\t\t\t</geometry>\n')
    contents.insert(actor_insert_line, '\t\t\t\t\t\t</box>\n')
    contents.insert(actor_insert_line, '\t\t\t\t\t\t\t<size>0.35 1.62 0.35</size>\n')
    contents.insert(actor_insert_line, '\t\t\t\t\t\t<box>\n')
    contents.insert(actor_insert_line, '\t\t\t\t\t<geometry>\n')
    contents.insert(actor_insert_line, '\t\t\t\t\t<pose>0 -0.18 0.05 0 -1.5707963267948966 0</pose>\n')
    contents.insert(actor_insert_line, '\t\t\t\t<collision name="' + actor_name + '_collision_link">\n')
    contents.insert(actor_insert_line, '\t\t\t<link name="' + actor_name + '_collision_link">\n')
    contents.insert(actor_insert_line, '\t\t\t<static>true</static>\n')
    contents.insert(actor_insert_line, '\t\t\t<pose>0 0 ' + str(-i-5) + ' 0 0 0</pose>\n')
    contents.insert(actor_insert_line, '\t\t<model name="' + actor_name + '_collision_model">\n')

f.seek(0)
f.writelines(contents)
f.truncate()
f.close()
