sudo apt-get install -y python-bson
sudo apt-get install -y python-websocket
cd ..
cd ..
git clone https://github.com/GT-RAIL/rosauth
git clone https://github.com/baalexander/rospy_message_converter
cd ..
catkin_make
catkin_make install
