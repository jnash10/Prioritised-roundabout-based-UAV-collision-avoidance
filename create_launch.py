import numpy as np

radius =120
n = int(input("how many drones you want? : "))
starts = []

step = 2*np.pi/n
for i in range(n):
    theta = i*step
    starts.append((radius*np.cos(theta),radius*np.sin(theta)))
    
    #goals.append((-np.cos(theta)+np.random.normal(loc=(0),scale=np.pi/6), -np.sin(theta)+np.random.normal(loc=(0),scale=np.pi/6)))


file = open('/media/storage/agam/hector_ws/src/hector_quadrotor_noetic/hector_quadrotor/hector_quadrotor_gazebo/launch/decoupled_four.launch','w')

#headers
file.write('<?xml version="1.0"?>\n\n')
file.write('<launch>\n')
file.write('<arg name="model" default="$(find hector_quadrotor_description)/urdf/quadrotor.gazebo.xacro" />\n')

#body

for i in range(n):
    
#        <group ns="uav2">
#      <include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch">
#        <arg name="name" value="uav2" />
#        <arg name="tf_prefix" value="uav2" />
#        <arg name="model" value="$(arg model)" />
#        <arg name="x" value="0.0"/>
#        <arg name="y" value="15.0" />
#      </include>
#    </group>
    file.write(f'<group ns="uav{i+1}">\n')
    file.write('<include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch">\n')
    file.write(f'<arg name="name" value="uav{i+1}" />\n')
    file.write('<arg name="model" value="$(arg model)" />\n')
    file.write(f'<arg name="x" value="{starts[i][0]}"/>\n')
    file.write(f'<arg name="y" value="{starts[i][1]}" />\n')
    file.write('</include>\n')
    file.write('</group>\n\n')

    #footer
file.write('</launch>')

file.close()