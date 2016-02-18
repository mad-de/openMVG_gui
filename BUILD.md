# Ubuntu building
modified build instructions from [cdcseacave](https://github.com/cdcseacave/openMVS/edit/master/BUILD.md)
```
#Prepare and empty machine for building:
sudo apt-get update -qq && sudo apt-get install -qq
sudo apt-get -y install git subversion cmake libpng-dev libjpeg-dev libtiff-dev libglu1-mesa-dev libeigen3-dev libboost-iostreams-dev libboost-program-options-dev libboost-system-dev libboost-serialization-dev libopencv-dev libcgal-dev libatlas-base-dev libsuitesparse-dev qt5-default libpcl-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev graphviz
main_path=`pwd`

#VCGLib (Required)
svn checkout svn://svn.code.sf.net/p/vcg/code/trunk/vcglib vcglib

#Ceres (Required)
git clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver
mkdir ceres_build
cd ceres_build/
cmake . ../ceres-solver/ -DMINIGLOG=ON -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make
sudo make install
cd ..

#CMVS / PMVS (Optional)
git clone https://github.com/mad-de/CMVS-PMVS.git --branch cout
mkdir CMVS-PMVS_build && cd CMVS-PMVS_build
cmake ../CMVS-PMVS/program
make
sudo make install
cd ..

#OpenMVG (Required)
git clone https://github.com/mad-de/openMVG_gui.git openMVG
mkdir openMVG_build
cd openMVG_build
cmake -DCMAKE_BUILD_TYPE=RELEASE . ../openMVG/src/ -DCMAKE_INSTALL_PREFIX=$main_path/openMVG_build/openMVG_install -DBUILD_SFM_GUI=ON -DOPNENMVG_PMVS_PATH=$main_path/CMVS-PMVS_build
make
cd ..

#OpenMVS
git clone https://github.com/cdcseacave/openMVS.git openMVS
mkdir openMVS_build
cd openMVS_build
cmake . ../openMVS -DCMAKE_BUILD_TYPE=Release -DVCG_DIR="$main_path/vcglib" -DOpenCV_CAN_BREAK_BINARY_COMPATIBILITY=OFF -DBUILD_SHARED_LIBS=ON

#Install OpenMVS library:
sudo make install
```
# Usage example
```
cd ~/openMVG_build/Linux-x86_64-RELEASE
./openMVG_SfM_gui
```
# Extra libs
* qt5-default 
* libpcl-dev

# Extra cmake options:
* -DBUILD_SFM_GUI=       [STANDARD: ON]
* -DOPNENMVG_PMVS_PATH=  [STANDARD: $main_path/CMVS-PMVS_build]