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

#include "ICPOper.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

#include "OpenCVInc.h"

#include <math.h>

#include "VLP16Graber.h"
#include "HDL32Grab.h"
#include "EuclideanClusterExtraction.h"
#include "LidarDataProcess.h"

#include "LcmReceiver4L.h"
#include "LCM_SENSOR_FUSION_PACKAGE.hpp"

#include "Param.h"

typedef struct GEO_POINT_2D_
{
    double x;
    double y;
    GEO_POINT_2D_()
    {
        x = 0.0;
        y = 0.0;
    }
}GEO_POINT_2D;

typedef struct
{
    int count;
    GEO_POINT_2D trace[200];
}MOTION_PLAN_TRACE;

int ReOrganizationPointsVLP(PointCloud<PointXYZI>::Ptr pts_in,
                             PointCloud<PointXYZI>::Ptr pts_out);

VelodyneParam g_Param;

string g_szChannelName("LCM_SENSOR_FUSION_PACKAGE");
CLcmReceive<LCM_SENSOR_FUSION_PACKAGE> g_LcmSend(g_szChannelName);
string g_szChannelName_Trace("MOTION_PLAN_TRACE");
CLcmReceive<LCM_SENSOR_FUSION_PACKAGE> g_LcmSend_Trace(g_szChannelName_Trace);

// 1 /home/zix/zix/mywork/2014-11-10-10-36-54_Velodyne-VLP_10Hz-County-Fair.pcap 10 180.0 1.6 2.4
// 1 /home/zix/zix/mywork/VLP16.pcap 5 2.5 0.75 -2.26
// 1 /home/zix/zix/mywork/0.pcap 5 0 0.69 -8.79
int main(int argc, char ** argv)
{
    CICPOpr icp;

    CLidarDataProcess::CLidarDataProcessParam Param;
    Param.dPathChangeDist = g_Param.dPathChangeDist;
    Param.dPathChangeToleratePer = g_Param.dPathChangeToleratePer;
    Param.dVehicleLen = g_Param.dVehicleLen;
    Param.dVehicleWidthHalf = g_Param.dVehicleWidthHalf;
    Param.nStableCont = g_Param.nStableCont;
    Param.nStableValid = g_Param.nStableValid;
    CLidarDataProcess LidarOpr;
    LidarOpr.Init(Param);

    int nSourceType = atoi(argv[1]);
    char* pszSourceAddr = argv[2];
    g_Param.dTreadSlope = atof(argv[3]);
    g_Param.CalibR.z = atof(argv[4]);
    g_Param.CalibR.y = atof(argv[5]);
    g_Param.CalibR.x = atof(argv[6]);

    Eigen::Affine3f pose =
            Eigen::Translation3f (Eigen::Vector3f (g_Param.CalibT.x, g_Param.CalibT.y, g_Param.CalibT.z)) *
            Eigen::AngleAxisf (g_Param.CalibR.z/180.0*CV_PI,   Eigen::Vector3f::UnitZ ()) *
            Eigen::AngleAxisf (g_Param.CalibR.y/180.0*CV_PI, Eigen::Vector3f::UnitY ()) *
            Eigen::AngleAxisf (g_Param.CalibR.x/180.0*CV_PI,  Eigen::Vector3f::UnitX ());

    VLPGrabber* pGrabber = NULL;
    if (nSourceType == 0)
    {
        pGrabber = new VLPGrabber(/*boost::asio::ip::address_v4::from_string(szAddr),2368*/);
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
//    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
//    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
    p->createViewPort (0.0, 0.0, 1.0, 1.0, vp_2);
    p->createViewPort (0.0, 0.0, 0.3, 0.6, vp_1);
    p->setBackgroundColor(0.05, 0.05, 0.05 ,vp_1);
    p->addCoordinateSystem(20.0, "Ori", vp_1);
    p->addCoordinateSystem(2.0, "Edge", vp_2);
    p->createViewPortCamera(vp_1);
    p->createViewPortCamera(vp_2);
    p->initCameraParameters();
    p->setCameraPosition(0.0, 0.0, 50.0, 0.0, 0.0, 0.0, vp_1);
    p->setCameraPosition(0.0, 0.0, 50.0, 0.0, 0.0, 0.0, vp_2);
    p->addText("", 0, 300, 30, 1, 1, 1, "szPclDisp");

    int nCont = 0;
    double dWheelAngle;
    std::vector<cv::Point2d> Track;
    cv::Mat TrackImg;
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
//        if(nCont != 10)
//        {
//            continue;
//        }
        boost::timer ter;

        PointCloud<PointXYZI>::Ptr CloudOri(new PointCloud<PointXYZI>);
        pcl::transformPointCloud (*ptCloudPt, *CloudOri, pose);
//        (*CloudOri) += (*ptCloudPt);
//        PointCloud<PointXYZI>::Ptr CloudOrgan(new PointCloud<PointXYZI>);
//        nRt = ReOrganizationPointsVLP(CloudOri, CloudOrgan);

        PointCloud<PointXYZI>::Ptr PtEdge(new PointCloud<PointXYZI>);
        PtEdge->reserve(CloudOri->size());
        PointCloud<PointXYZI>::Ptr PtContinue(new PointCloud<PointXYZI>);
        PtContinue->reserve(CloudOri->size());
        int nLayer = 16;
        int nTickCont = CloudOri->size()/nLayer;
        float fSlope = tan(g_Param.dTreadSlope/180.0*CV_PI);
        for (int i = 0; i < nTickCont; i++)
        {
            for (int j = 2; j < /*nLayer/2 + 1*/nLayer - 1; j++)
            {
                PointXYZI pt0 = CloudOri->at(i*16+j-1);
                PointXYZI pt1 = CloudOri->at(i*16+j+1);
                if (boost::math::isnan(pt0.x) || boost::math::isnan(pt1.x) ||
                    boost::math::isnan(pt0.y) || boost::math::isnan(pt1.y) ||
                    boost::math::isnan(pt0.z) || boost::math::isnan(pt1.z))
                {
                    continue;
                }
                if (pt1.z >= 1.5)
                {
                    continue;
                }
                if (pt1.x >= -2.0 && pt1.x <= 2.0 && pt1.y >= -3.0 && pt1.y <= 3.0)
                {
                    continue;
                }
                float dz = /*fabs*/(pt1.z-pt0.z);
                float dxy = sqrt(pow((pt0.x-pt1.x),2)+pow((pt0.y-pt1.y),2));
                if (dz/dxy >= fSlope)
                {
                    PtEdge->push_back(pt1);
//                    break;
                }
                else
                {
                    PtContinue->push_back(pt1);
                }
            }
        }

        PointCloud<PointXYZI>::Ptr IcpTrack(new PointCloud<PointXYZI>);
        double dScore = 0.0;
//        icp.NDTRegistrate(PtEdge, dScore);
        if(icp.m_VecPose.size() > 0)
        {
            IcpTrack->resize(icp.m_VecPose.size());
            cv::Mat CurPose = icp.m_VecPose.back().inv();
            for(unsigned int i = 0; i < icp.m_VecPose.size(); i++)
            {
                cv::Mat xx = CurPose * icp.m_VecPose[i];
                IcpTrack->at(i).x = xx.at<float>(0,3);
                IcpTrack->at(i).y = xx.at<float>(1,3);
                IcpTrack->at(i).z = 0.0;
                IcpTrack->at(i).intensity = 1.0;
            }
        }
        PointCloudColorHandler<pcl::PointXYZI>* handler_4 = new PointCloudColorHandlerCustom<PointXYZI> (IcpTrack, 0.0, 0.0, 255.0);
        p->removePointCloud("IcpTrack",vp_2);
        p->addPointCloud(IcpTrack,*handler_4, "IcpTrack", vp_2);
        delete handler_4;

        cv::Mat matCloudPt = cv::Mat::zeros(PtEdge->size(), 3, CV_32FC1);
        for(unsigned int i = 0; i < PtEdge->size(); i++)
        {
            matCloudPt.at<float>(i,0) = PtEdge->at(i).x * 1000.0;
            matCloudPt.at<float>(i,1) = PtEdge->at(i).y * 1000.0;
            matCloudPt.at<float>(i,2) = PtEdge->at(i).z * 1000.0;
        }
        cv::Mat imgCloudPt = LidarOpr.Grid(matCloudPt);
        LidarOpr.GetBestTrackEx(imgCloudPt,dWheelAngle,Track,TrackImg);
        char szPclDisp[256] = {0};
        sprintf(szPclDisp, "%.2f\n%.2f", dWheelAngle, Track.size()/2.0);
        int* pnWindSize = p->getRenderWindow()->GetSize();
        p->updateText(szPclDisp, 0, pnWindSize[1]-50, 20, 1, 1, 1, "szPclDisp");
        cv::namedWindow("imgCloudPt");
        cv::imshow("imgCloudPt",imgCloudPt);
        cv::waitKey(1);
        cv::namedWindow("TrackImg");
        cv::imshow("TrackImg",TrackImg);
        cv::waitKey(1);
        std::cout << "wheel angel:" << dWheelAngle << std::endl;
        PointCloud<PointXYZI>::Ptr PtTrack(new PointCloud<PointXYZI>);
        for(unsigned int i = 0; i < Track.size(); i++)
        {
            PointXYZI pt;
            pt.x = Track[i].x/1000.0;
            pt.y = Track[i].y/1000.0;
            pt.z = 0.0;
            pt.intensity = 1.0;
            PtTrack->push_back(pt);
        }
        PointCloudColorHandler<pcl::PointXYZI>* handler_3 = new PointCloudColorHandlerCustom<PointXYZI> (PtTrack, 0.0, 255.0, 0.0);
        if (Track.size() < g_Param.dPathChangeDist*2)
        {
            handler_3 = new PointCloudColorHandlerCustom<PointXYZI> (PtTrack, 255.0, 0.0, 0.0);
        }
        p->removePointCloud("PtTrack",vp_2);
        p->addPointCloud(PtTrack,*handler_3, "PtTrack", vp_2);
        delete handler_3;

//        PointCloudColorHandler<pcl::PointXYZI>* handler_1 = new PointCloudColorHandlerGenericField<PointXYZI> (CloudOri,"z");
//        p->removePointCloud("CloudOri",vp_1);
//        p->addPointCloud(CloudOri,*handler_1, "CloudOri", vp_1);
//        delete handler_1;

        PointCloudColorHandler<pcl::PointXYZI>* handler_2 = new PointCloudColorHandlerGenericField<PointXYZI> (PtEdge,"z");
        p->removePointCloud("PtEdge",vp_2);
        p->addPointCloud(PtEdge,*handler_2,"PtEdge",vp_2);
        delete handler_2;

        LCM_SENSOR_FUSION_PACKAGE Pack;
        pcl::PointCloud<pcl::PointXYZI>::Ptr CloudCluster(new pcl::PointCloud<pcl::PointXYZI>);
        int nCont = EuclideanClusterExtraction_(PtEdge, CloudCluster, Pack.IbeoObjList);
        PointCloudColorHandler<pcl::PointXYZI>* handler_CloudCluster = new PointCloudColorHandlerGenericField<PointXYZI> (CloudCluster,"intensity");
        p->removePointCloud("CloudCluster",vp_1);
        p->addPointCloud(CloudCluster,*handler_CloudCluster,"CloudCluster",vp_1);
        delete handler_CloudCluster;


//        std::cout<< "Time cost: " <<ter.elapsed()<<std::endl;

        p->spinOnce(1);


        LCM_NAVI_PATH Path;
        Path.PathId = 0;
        Path.PathType = 0;
        for(unsigned int i = 0; i < Track.size(); i++)
        {
            LCM_POINT2D_F pt_;
            pt_.x = Track[i].x / 1000.0;
            pt_.y = Track[i].y / 1000.0;
            Path.Path.push_back(pt_);
        }
        Path.ContPathPoint = Path.Path.size();
        Pack.NaviInfo.Paths.push_back(Path);
        Pack.NaviInfo.ContPath = Pack.NaviInfo.Paths.size();

        g_LcmSend.Send(g_szChannelName, Pack);

        MOTION_PLAN_TRACE Track_;
        int nContTrack = 0;
        if(nContTrack >= 200)
        {
            nContTrack = 200;
        }
        for(unsigned int i = 0; i < nContTrack; i++)
        {
            Track_.trace[i].x = 0/*Track[i].x / 1000.0*/;
            Track_.trace[i].y = i/*Track[i].y / 1000.0*/;
        }
        Track_.count = nContTrack;
//        g_LcmSend_Trace.m_plcmIpc->publish(g_szChannelName_Trace,&Track_,sizeof(MOTION_PLAN_TRACE ));
    }

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


