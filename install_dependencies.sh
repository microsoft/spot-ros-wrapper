echo "Updating apt-get"
sudo apt-get update

echo "Installing system packages"
sudo apt-get install python3.8
sudo apt-get -y install python3-pip
sudo apt-get install python3-catkin-tools

echo "Installing pip packages"
pip3 install -r ./requirements.txt
