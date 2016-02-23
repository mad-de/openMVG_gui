#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <fstream>
#include <string> 
using namespace std;
    
int user_data;
    
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    // Set Background to black
    viewer.setBackgroundColor(0.0, 0.0, 0.0);

    // Automatically set camera position - switch to manual camera position until viewer.removeOrientationMarkerWidgetAxes(); works.
    // viewer.initCameraParameters();

    // Manually set camera position 
    double PositionX = 2;
    double PositionY = 0;
    double PositionZ = -2;
    double FocalPointX = 1;
    double FocalPointY = 0;
    double FocalPointZ = 0;
    double ViewUpX = 0;
    double ViewUpY = -1;
    double ViewUpZ = 0;
    int Viewpoint = 0;
    viewer.setCameraPosition(PositionX, PositionY, PositionZ, FocalPointX, FocalPointY, FocalPointZ, ViewUpX, ViewUpY, ViewUpZ, Viewpoint);
    viewer.updateCamera(); 
}

void FileNotFound (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.addText ("Preview file could not be found.", 200, 300, "text", 0);
}

void viewerIteration (pcl::visualization::PCLVisualizer& viewer)
{
    //FIXME: possible race condition here:
    user_data++;
}

bool fexists(const std::string& filename) {
    ifstream ifile(filename.c_str());
    if ( !ifile.is_open() ) { return false; }
    else { return true; }
}

int main(int argc, char *argv[])
{
    // Get the variables
    string input_file, title;
    for(unsigned i = 1; i != argc; i++)
    { 
	if (string(argv[i]) == "-i")
	{
	    input_file = string(argv[2]);
	}
	else if (string(argv[i]) == "-t")
	{
	    title = string(argv[4]);
	}
    }
    // Generate Point Cloud Viewer
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPLYFile (input_file, *cloud);
    pcl::visualization::CloudViewer viewer(title);
    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);

    if (!fexists(input_file))
    {
	viewer.runOnVisualizationThreadOnce (FileNotFound);
    }
    else
    {
	// call once
	viewer.runOnVisualizationThreadOnce (viewerOneOff);
    }

    while (!viewer.wasStopped ())
    {
	user_data++;
    }
    return 0;
}
