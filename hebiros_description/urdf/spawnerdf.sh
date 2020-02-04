
cd ../../../../
catkin_make
cd src/HEBI-ROS/hebiros_description/urdf/

rosrun xacro xacro myArm.xacro > myArm.urdf
echo "xacro parsing done."
sleep .5
gz sdf -p myArm.urdf > my_sdf.sdf
echo "sdf parsing done."
sleep .5
cp my_sdf.sdf ../sdf/MyArm/MyArm.sdf
rm my_sdf.sdf
echo "sdf copied Done."
sleep .5
roslaunch hebiros_description MyArmGZ.launch
