#include <iostream>

#include <pcl/io/vtk_lib_io.h>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
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

    // Load files
    pcl::TextureMesh mesh;
    pcl::io::loadOBJFile(input_file, mesh);
    pcl::visualization::PCLVisualizer viewer(title);
    viewer.addTextureMesh(mesh, "material_0", 0);

    // Display warning if file is not existent, otherwise display options
    if (!fexists(input_file))
    {
	viewer.addText ("Preview file could not be found.", 200, 300, "error", 0);
    }
    else
    {
    	viewer.addText (options + "File: " + input_file, 5, 20, "options", 0);
    }

    // Main loop
    while (!viewer.wasStopped ())
    {
	viewer.spinOnce ();
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}
