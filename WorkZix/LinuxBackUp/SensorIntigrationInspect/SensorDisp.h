#ifndef SENSOR_DISP_H
#define SENSOR_DISP_H

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/timer.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations


#include <pcl/io/pcd_io.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <typeinfo>

#include <pcl/features/normal_3d.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

#include "OpenCVInc.h"
#include "LCM_SENSOR_FUSION_PACKAGE.hpp"

class CSensorDisp
{
public:
    CSensorDisp();
    ~CSensorDisp();
    int ShowPackData(LCM_SENSOR_FUSION_PACKAGE& Pack);

    pcl::visualization::PCLVisualizer* m_p;
    int m_vp1;
    long m_nCont;
    vector<vtkSmartPointer<vtkFollower> > m_VecText3D;
    vector<vtkSmartPointer<vtkLODActor> > m_VecCube;
};








#endif
