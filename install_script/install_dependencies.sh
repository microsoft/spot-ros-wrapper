echo "Updating apt-get"
sudo apt-get update

echo "Installing system packages"
sudo apt-get -y install python3-pip
sudo apt-get install python3-catkin-tools
sudo apt install ros-melodic-joint-state-publisher-gui
sudo apt install ros-melodic-rqt

echo "Installing pip packages"
pip install -r ./requirements.txt