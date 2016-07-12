sudo apt-get install -y python-bson
sudo apt-get install -y python-websocket
sudo apt-get install -y libyaml-cpp-dev
cd ..
git clone https://github.com/ros-perception/image_common
git clone https://github.com/bosch-ros-pkg/usb_cam
git clone https://github.com/GT-RAIL/rosauth
git clone https://github.com/baalexander/rospy_message_converter
cd ..
catkin_make
catkin_make install
