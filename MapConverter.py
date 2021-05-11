'''
Taylor Shelby, ESE 650
Makes a gazebo compatible sdf from a .json file which uses the MEAM 620 obstacle format
Just swap out filename below with the name of the json file you want to convert, sans the .json
Make sure the json file is in the same folder as this code

For loading, put in a directory in ~/.gazebo/models with a config file that links to your model's sdf
The current hector quadrotor load chain
roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch
outdoor_flight_gazebo.launch (src/hector_quadrotor/hector_quadrotor_demo) ->
rolling_landscape_120m.launch (src/hector_gazebo/hector_gazebo_worlds/launch)-> ***Modified to call 3WallsWorld
3WallsWorld (src/hector_gazebo/hector_gazebo_worlds/worlds) ->
mymap (~/.gazebo/models/mymap) -> whatever map is in the model.config (currently overUnder)
TODO: figure out which can be skipped for our purposes

Potential future work: change the colors so they actually read from the file and/or are transparent
autogenerate config file and/or directory
'''


import json
import numpy as np

filename = 'forest'
inputfile = filename+'.json'
outputfile = filename+'.sdf'

with open(inputfile) as f:
    data = json.load(f)

blocks = data['blocks']

f = open(outputfile, "w")
f.write("<?xml version='1.0'?>\n<sdf version=" + '"1.6">\n  <model name="' + inputfile[0:-5] + '">\n')
f.write("   <static>true</static>\n\n")
for i in range(len(blocks)):
    npBlock_old = np.asarray(blocks[i]["extents"])
    npBlock= np.asarray([npBlock_old[0], npBlock_old[2], npBlock_old[4], npBlock_old[1], npBlock_old[3], npBlock_old[5]])
    dimensions = abs(npBlock[0:3] - npBlock[3:])
    pose = (dimensions / 2) + npBlock[0:3]

    f.write('   <link name ="link' + str(i) + '">\n')
    f.write("    <pose>" + str(pose)[1:-1] + " 0 0 0</pose>\n")
    f.write("    <collision name = 'collision'>\n")
    f.write("      <geometry>\n        <box>\n")
    f.write("         <size>" + str(dimensions)[1:-1] + "</size>\n")
    f.write("        </box>\n      </geometry>\n     </collision>\n")

    f.write("    <visual name = 'visual'>\n")
    f.write("      <geometry>\n        <box>\n")
    f.write("         <size>" + str(dimensions)[1:-1] + "</size>\n")
    f.write("        </box>\n      </geometry>\n     </visual>\n")

    f.write('   </link>\n\n')

f.write("</model>\n</sdf>")
f.close()
print("done")
