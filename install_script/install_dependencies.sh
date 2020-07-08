echo "Updating apt-get"
sudo apt update
sudo apt-get update

echo "Installing system packages"
sudo apt-get -y install python3-pip
sudo apt-get install python3-catkin-tools
sudo apt install ros-melodic-joint-state-publisher-gui
sudo apt install ros-melodic-rqt

echo "Installing tf2 with python3 support"
sudo apt install python3-numpy python3-catkin-pkg-modules python3-rospkg-modules python3-empy

echo "Installing pip packages"
pip install -r ./requirements.txt