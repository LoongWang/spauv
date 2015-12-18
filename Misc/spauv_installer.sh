#!/bin/bash
# SP AUV SOFTWARE INSTALLER 
# WRITTEN BY ALZAM BIN ABDUL GHANI

# CREATE AND INIT WORKSPACE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

# build workspace
cd ~/catkin_ws/
catkin_make

# add workspace to path
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc

# verify path
echo $ROS_PACKAGE_PATH

#END OF WORKSPACE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

#Installing ros camera driver >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
cd catkin_ws/src/ #change directory to your source folder
git clone https://github.com/ktossell/camera_umd.git #clone the package from its repo
rosdep install camera_umd uvc_camera jpeg_streamer
cd .. #go one dir up to catkin_ws
catkin_make #build the workspace

sudo apt-get install libv4l-dev

# Original Camera installation site : https://defendtheplanet.net/2014/11/05/using-ros-indigo-webcam-by-the-uvc_camera-usb-video-class-package/
# End of ros camera driver installer >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

# Setup Install Date >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#Set the date 
date_formatted=$(date +%m.%d.%y-%H:%M:%S)
#Show the date
echo " Successfully Installed @" $date_formatted
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

# INSTALL & RUN Main code >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

#Download Code
cd catkin_ws/src/
git clone spauv folder

#Changes
cd catkin_ws/src/spauv/shellscripts
sudo nano gui.sh

In file called 'gui.sh' :
Line no    Sentence
1	   #!/bin/bash 
2
3           cd /home/owner/catkin_ws/src/spauv/Scripts
4	   ./spauv_interface.py &
Change line 3 : cd /home/owner/catkin_ws/src/spauv/Scripts to cd /<ur location of spauv/Scripts>

#Build code
cd catkin_ws/
catkin_make

#Finally launch program 
roslaunch spauv spauv.launch

# END OF ROS INSTALLER >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

