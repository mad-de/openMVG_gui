#include <iostream>

#include <boost/thread/thread.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
using namespace std;

bool fexists(const std::string& filename) 
{
    ifstream ifile(filename.c_str());
    if ( !ifile.is_open() ) { return false; }
    else { return true; }
}

// --------------
// -----Main-----
// --------------
int main(int argc, char *argv[])
{
    string input_file, title, options, view;
    int pos;
    for(unsigned i = 1; i != argc; i++)
    { 
	if (string(argv[i]) == "-i")
    	{
	    input_file = string(argv[i+1]);
        }
        else if (string(argv[i]) == "-t")
	{
	    title = string(argv[i+1]);
	}
	else if (string(argv[i]) == "-o")
	{
	    options = string(argv[i+1]);
	    boost::replace_all(options, "[EOL]", "\n");
	}
    }

    // Load point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPLYFile(input_file, *cloud);

    // Display point cloud
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (title));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "ply cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ply cloud");

    // Set manual camera position for demo files
    if (input_file.find("ImageDataset_SceauxCastle") != string::npos)
    {
	double PositionX = 2; double PositionY = 0; double PositionZ = -2; double FocalPointX = 1; double FocalPointY = 0; double FocalPointZ = 0; double ViewUpX = 0; double ViewUpY = -1; double ViewUpZ = 0; int Viewpoint = 0;
	viewer->setCameraPosition(PositionX, PositionY, PositionZ, FocalPointX, FocalPointY, FocalPointZ, ViewUpX, ViewUpY, ViewUpZ, Viewpoint);
	viewer->updateCamera(); 
    }
    else
    {
    	viewer->initCameraParameters ();
    }

    // Display warning if file is not existent, otherwise display options
    if (!fexists(input_file))
    {
	viewer->addText ("Preview file could not be found.", 200, 300, "error", 0);
    }
    else
    {
    	viewer->addText (options + "File: " + input_file, 5, 20, "options", 0);
    }

    // Main loop
    while (!viewer->wasStopped ())
    {
	viewer->spinOnce ();
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

