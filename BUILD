Ubuntu:

sudo apt-get install -y qt5-default libpcl-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev graphviz
git clone --recursive https://github.com/mad-de/openMVG_gui.git openMVG
mkdir openMVG_build
cd openMVG_build
cmake -DCMAKE_BUILD_TYPE=RELEASE . ../openMVG/src/ -DCMAKE_INSTALL_PREFIX=~/openMVG_build/openMVG_install
make
