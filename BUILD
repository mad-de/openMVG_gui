=====================================
OpenMVG (open Multiple View Geometry)
=====================================

------------------
Build instructions
------------------

Required tools:
* Cmake 
* Git
* c/c++ compiler (gcc or visual studio or clang)

Getting the sources:
$ git clone --recursive https://github.com/openMVG/openMVG.git
or
$ git clone https://github.com/openMVG/openMVG.git
$ cd openMVG
$ git submodule init
$ git submodule update

As openMVG use some C++11 features you must have a c++11 ready compiler:
- Visual studio >= 2013
- GCC >= 4.7

--------------------------
General informations
for openMVG cmake options
--------------------------
OpenMVG_BUILD_TESTS (ON/OFF(default))=> Build openMVG unit tests
OpenMVG_BUILD_EXAMPLES (ON/OFF(default))=> Build OpenMVG example applications.
  Does not affect binaries under 'software'

--------------------------
General informations
for openMVG SfM pipelines
--------------------------
OpenMVG can export graphs as graphviz .dot files and render them as SVG files.
If you want consider this graph visualization feature, please consider to install Graphviz.

-----------------
Linux compilation
-----------------

#Prepare and empty machine for building:
sudo apt-get update -qq && sudo apt-get install -qq

sudo apt-get install -y qt5-default libpcl-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev graphviz
git clone --recursive https://github.com/mad-de/openMVG_gui.git openMVG
mkdir openMVG_build
cd openMVG_build
cmake -DCMAKE_BUILD_TYPE=RELEASE . ../openMVG/src/ -DCMAKE_INSTALL_PREFIX=~/openMVG_build/openMVG_install
make

-------------------
Windows compilation
-------------------

Checkout the project
 $ git clone --recursive https://github.com/openMVG/openMVG.git

Open cmake-gui
Fill the source path with the src openMVG path.
Fill the build path with a new directory
Select your Visual Studio IDE and click configure and then generate

Open the .sln solution created in your build directory.
Change the target to Release.
Compile the libraries and binaries samples.

-------------------
Mac compilation
-------------------

 $ git clone --recursive https://github.com/openMVG/openMVG.git
 $ cd openMVG
 $ ls
  AUTHORS BUILD  docs  logo  README  src  ...
 $ cd ..
 $ mkdir openMVG_Build
 $ cd openMVG_Build
 $ cmake -DCMAKE_BUILD_TYPE=RELEASE -G "Xcode" . ../openMVG/src/
If you want enable unit tests and examples to the build:
 $ cmake -DCMAKE_BUILD_TYPE=RELEASE -DOpenMVG_BUILD_TESTS=ON -DOpenMVG_BUILD_EXAMPLES=ON -G "Xcode" . ../openMVG/src/
 $ xcodebuild -configuration Release

 
--------------------
Using openCV sample
--------------------

 Add -DOpenMVG_USE_OPENCV=ON to your cmake command line and set OpenCV_DIR variable to your openCV build directory
=> i.e.: -DOpenCV_DIR="/home/user/Dev/github/itseez/opencv_Build" -DOpenMVG_USE_OPENCV=ON
 
------------------------------------------------------------
Using OpenMVG as a third party library dependency in cmake
-------------------------------------------------------------

OpenMVG can be used as a third party once it have been installed.
Because it can use it's own ceres version, it's better to install it locally and not in system files.
So please consider using the CMAKE_INSTALL_PREFIX cmake variable to specify a local installation directory.

Here the syntax to add the variable to the cmake command line (use absolute path):
-DCMAKE_INSTALL_PREFIX:STRING="YourInstallPath"
i.e: -DCMAKE_INSTALL_PREFIX:STRING="/home/user/Dev/github/openMVG_Build/openMVG_install"

Perform "make" and "make install"

Once the library has been installed, go to your project that want use OpenMVG as an external library and add:

FIND_PACKAGE(OpenMVG REQUIRED)
INCLUDE_DIRECTORIES(${OPENMVG_INCLUDE_DIRS})
ADD_EXECUTABLE(main main.cpp)
TARGET_LINK_LIBRARIES(main ${OPENMVG_LIBRARIES})

Specify to CMAKE where OpenMVG have been installed by using the cmake OpenMVG_DIR variable that must point to:
-DOpenMVG_DIR:STRING="YourInstallPath"/share/openMVG/cmake

A message will be displayed if OpenMVG is found or not at the cmake configure step.
