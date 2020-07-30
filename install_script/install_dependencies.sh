echo "Updating apt-get"
sudo apt update
sudo apt-get update

echo "Installing system packages"
sudo apt-get -y install python3-pip
sudo apt-get install python3-catkin-tools

sudo apt-get install ros-noetic-rosbridge-server
sudo apt-get install python-is-python3 #to point "python" to "python3"

echo "Installing pip packages"
pip install -r ./requirements.txt
