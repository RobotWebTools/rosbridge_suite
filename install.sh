sudo apt-get install -y python-bson
sudo apt-get install -y python-websocket
cd ..
git clone https://github.com/ros-perception/image_common
git clone https://github.com/bosch-ros-pkg/usb_cam
git clone https://github.com/GT-RAIL/rosauth
git clone https://github.com/baalexander/rospy_message_converter
catkin_make
catkin_make install
