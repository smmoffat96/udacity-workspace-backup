COMMANDS TO RUN ON STARTUP OF UDACITY WORKSPACE

cd /home
git clone https://github.com/smmoffat96/udacity-workspace-backup.git
git config --global user.email “smmoffat@wpi.edu”
git config --global user.name “Shannon Moffat”

cd /home/udacity-workspace-backup

sudo apt-get update && sudo apt-get upgrade -y

export GAZEBO_MODEL_PATH=/home/udacity-workspace-backup/
export OPENCV_DIR=/home/udacity-workspace-backup/
