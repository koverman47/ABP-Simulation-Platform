#!/usr/bin/env python

import sys

num = int(sys.argv[1])

f = open("./launch/spawn.launch", "w")
f.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
f.write("<launch>\n")

f.write("\t<arg name=\"pose_x\" default=\"0.0\"/>\n")
f.write("\t<arg name=\"pose_y\" default=\"0.0\"/>\n")
f.write("\t<arg name=\"pose_z\" default=\"0.1\"/>\n")
f.write("\t<arg name=\"pose_yaw\" default=\"0.0\"/>\n")
f.write("\t<arg name=\"robot_id\" default=\"0\"/>\n\n")

x = str(0)
counter = 0
for i in range(num):
    s = str(i)
    if i % 2 == 0:
        x = str(2 * counter)
        counter += 1
    else:
        x = str(-2 * counter)

    f.write("\t<group ns=\"satlet"+str(i)+"\">\n")
    f.write("\t\t<include file=\"$(find abp_sim)/launch/satlet.launch\">\n")
    f.write("\t\t\t<arg name=\"pose_x\" value=\""+x+"\"/>\n")
    f.write("\t\t\t<arg name=\"pose_y\" value=\"$(arg pose_y)\"/>\n")
    f.write("\t\t\t<arg name=\"pose_z\" value=\"$(arg pose_z)\"/>\n")
    f.write("\t\t\t<arg name=\"pose_yaw\" value=\"$(arg pose_yaw)\"/>\n")
    f.write("\t\t\t<arg name=\"robot_id\" value=\""+str(i)+"\"/>\n")
    f.write("\t\t</include>\n")
    f.write("\t</group>\n")

f.write("</launch>\n")


f.close()
