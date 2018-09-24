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

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <math.h>

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

#include "OpenCVInc.h"

#include <math.h>

#include "VLP16Graber.h"
#include "HDL32Grab.h"

void GetGroundPoints(PointCloud<PointXYZI>::Ptr pts_in,
    PointCloud<PointXYZI>::Ptr pts_out,
    PointCloud<PointXYZI>::Ptr pts_obj_out,
    pcl::ModelCoefficients::Ptr coefficients);

bool g_bIsPause = false;

void visualization_button_callback (const pcl::visualization::KeyboardEvent &event,
                                    void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
      if ((event.getKeySym () == "s" || event.getKeySym () == "S")&& event.keyDown ())
      {
            g_bIsPause = !g_bIsPause;
      }
}

int main(int argc, char ** argv)
{
    int nSourceType = atoi(argv[1]);
    char* pszSourceAddr = argv[2];
    float dMinX = atof(argv[3]);
    float dMaxX = atof(argv[4]);
    float dMinY = atof(argv[5]);
    float dMaxY = atof(argv[6]);
    float dHeading = atof(argv[7]);
    float dRoll = atof(argv[8]);
    float dPitch = atof(argv[9]);

//  1 /home/zix/zix/mywork/0.pcap -2 2 -50 50 0 0 0
//

    Eigen::Affine3f pose =
            Eigen::Translation3f (Eigen::Vector3f (0, 0, 0)) *
            Eigen::AngleAxisf (dHeading/180.0*CV_PI,   Eigen::Vector3f::UnitZ ()) *
            Eigen::AngleAxisf (dRoll/180.0*CV_PI, Eigen::Vector3f::UnitY ()) *
            Eigen::AngleAxisf (dPitch/180.0*CV_PI,  Eigen::Vector3f::UnitX ());

    VLPGrabber* pGrabber = NULL;
    if (nSourceType == 0)
    {
        pGrabber = new VLPGrabber(/*boost::asio::ip::address_v4::from_string(pszSourceAddr),2368*/);
    }
    else
    {
        pGrabber = new VLPGrabber(pszSourceAddr);
    }

    SimpleVLPViewer<PointXYZI> v(*pGrabber);

    v.start();

    pcl::visualization::PCLVisualizer* p = NULL;
    int vp_1, vp_2;
    p = new pcl::visualization::PCLVisualizer(argc,argv,"Pairwise Incremental Registration example");
    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
    p->addCoordinateSystem(10.0, vp_1);
    p->addCoordinateSystem(10.0, vp_2);
//    p->createViewPortCamera(vp_1);
//    p->createViewPortCamera(vp_2);
//    p->initCameraParameters();
    p->setCameraPosition(0.0, 0.0, 50.0, 0.0, 0.0, 0.0, vp_1);
//    p->setCameraPosition(0.0, 0.0, 50.0, 0.0, 0.0, 0.0, vp_2);

    boost::signals2::connection button_connection = p->registerKeyboardCallback(visualization_button_callback, (void*)p);

    int nCont = 0;
    while(1)
    {
        PointCloud<PointXYZI>::ConstPtr ptCloudPt;
        int nRt = v.GetData(ptCloudPt);
        if (nRt == 0)
        {
            usleep(10000);
            p->spinOnce();
            continue;
        }
        nCont++;
        boost::timer ter;

        PointCloud<PointXYZI>::Ptr CloudOriT0(new PointCloud<PointXYZI>);
        pcl::transformPointCloud (*ptCloudPt, *CloudOriT0, pose);

        PointCloud<PointXYZI>::Ptr CloudOri(new PointCloud<PointXYZI>);
//        (*CloudOri) += (*ptCloudPt);
        CloudOri->reserve(CloudOriT0->size());
        for(int i = 0; i < CloudOriT0->size(); i++)
        {
            PointXYZI pt = CloudOriT0->at(i);
            if(boost::math::isnan(pt.x) ||
                    boost::math::isnan(pt.y) ||
                    boost::math::isnan(pt.z))
            {
                continue;
            }
            if(pt.x < dMinX || pt.x > dMaxX ||pt.y < dMinY || pt.y > dMaxY)
            {
                continue;
            }
            CloudOri->push_back(pt);
        }

        PointCloud<PointXYZI>::Ptr pts_out(new PointCloud<PointXYZI>);
        PointCloud<PointXYZI>::Ptr pts_obj_out(new PointCloud<PointXYZI>);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        GetGroundPoints(CloudOri, pts_out, pts_obj_out, coefficients);

        PointCloudColorHandler<pcl::PointXYZI>* handler_1 = new PointCloudColorHandlerGenericField<PointXYZI> (CloudOri,"intensity");
        p->removePointCloud("1",vp_1);
        p->addPointCloud(CloudOri,*handler_1, "1", vp_1);
        delete handler_1;

        PointCloudColorHandler<pcl::PointXYZI>* handler_2 = new PointCloudColorHandlerGenericField<PointXYZI> (pts_out,"intensity");
        p->removePointCloud("2",vp_2);
        p->addPointCloud(pts_out,*handler_2,"2",vp_2);
        delete handler_2;

        std::cout<< "Time cost: " <<ter.elapsed()<<std::endl;

        while(g_bIsPause)
        {
            p->spinOnce();
        }

        p->spinOnce();


    }

    button_connection.disconnect();
    return (0);
}


int ReOrganizationPointsVLP(PointCloud<PointXYZI>::Ptr pts_in,
                            PointCloud<PointXYZI>::Ptr pts_out)
{
    int MapTable[] = {0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15};
    int nLayer = 16;
    bool bIsValid = ((pts_in->size()%nLayer)==0);
    if(!bIsValid)
    {
        printf("Invalid VLP data pack!\n");
        return 0;
    }
    int nHorizonTicks = pts_in->size()/nLayer;
    pts_out->clear();
    pts_out->resize(pts_in->size());
    pts_out->height = nLayer;
    pts_out->width = nHorizonTicks;
    pts_out->is_dense = false;
    for(int i = 0; i < nHorizonTicks; i++)
    {
        for(int j = 0; j < nLayer; j++)
        {
            PointXYZI& pt = pts_in->at(i*nLayer+j);
            pts_out->at(i,MapTable[j]) = pt;
        }
    }

    return 1;
}

void GetGroundPoints(PointCloud<PointXYZI>::Ptr pts_in,
    PointCloud<PointXYZI>::Ptr pts_out,
    PointCloud<PointXYZI>::Ptr pts_obj_out,
    pcl::ModelCoefficients::Ptr coefficients)
{
    double dNearRange = 10.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_near(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_near->reserve(pts_in->size());
    for(int i = 0; i < pts_in->size(); i++)
    {
        PointXYZI& pt = pts_in->at(i);
        if(boost::math::isnan(pt.x) || boost::math::isnan(pt.y) || boost::math::isnan(pt.z))
        {
            continue;
        }
        double dDist = sqrt(pow(pt.x,2)+pow(pt.y,2)+pow(pt.z,2));
        if(dDist >= dNearRange)
        {
            continue;
        }
        cloud_near->push_back(pt);
    }
    cloud_near->is_dense = true;

//    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>),
            cloud_p (new pcl::PointCloud<pcl::PointXYZI>),
            cloud_f (new pcl::PointCloud<pcl::PointXYZI>);

//    pcl::toPCLPointCloud2(*cloud_near,*cloud_blob);

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud_near);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*cloud_filtered);

    // Convert to the templated PointCloud
//    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

//	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.1);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    pts_obj_out->clear();
    int nr_points = (int) cloud_filtered->points.size ();
    for (int i = 0; i < 1; i++)
    {
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            break;
        }

        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);

        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];
        double dRoll = -1.0*atan(-1.0*a/c)/CV_PI*180.0;
        double dPitch = atan(-1.0*b/c)/CV_PI*180.0;
//        g_dPitch = dPitch;
//        g_dRoll = dRoll;
        double dHeight = d/c;
//        if (fabs(dRoll)<=5.0 && fabs(dPitch)<=5.0 && dHeight>=1.6 && dHeight<=1.9)
        {
            pts_out->clear();
            (*pts_out) += (*cloud_p);
            printf("plane%d :dRoll:%.2f, dPitch:%.2f, dHeight:%.2f\n",i,dRoll,dPitch,dHeight);
        }
//        else
//        {
//            (*pts_obj_out) += (*cloud_p);
//        }

        extract.setNegative (true);
        extract.filter (*cloud_f);

        cloud_filtered.swap(cloud_f);

    }
    (*pts_obj_out) += (*cloud_filtered);
}


