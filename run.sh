mkdir apm
cd apm
#Specific branch
#git clone -b Copter-3.5.5 https://github.com/ardupilot/ardupilot
git clone https://github.com/ardupilot/ardupilot
cd ardupilot/
git submodule update --init --recursive
cd Tools/autotest/
pwd > directory
export PATH="$directory":$PATH > ~/.bashrc
source ~/.bashrc
