TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    main.cpp \
    Param.cpp \
    ../include/LidarDataProcess.cpp

INCLUDEPATH += /home/zix/zix/WInSvn/LinuxBackUp/include
INCLUDEPATH += /home/zix/zix/WInSvn/3rdparty/LcmSDK/include
INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/local/include/pcl-1.8
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/local/include/vtk-7.0


LIBS += /usr/lib/x86_64-linux-gnu/libpthread.so

LIBS += /usr/local/lib/libvtkRenderingCore-7.0.so
LIBS += /usr/local/lib/libvtkRenderingFreeType-7.0.so
LIBS += /usr/local/lib/libvtkCommonDataModel-7.0.so
LIBS += /usr/local/lib/libvtkCommonMath-7.0.so
LIBS += /usr/local/lib/libvtkCommonCore-7.0.so
LIBS += /usr/local/lib/libvtkCommonExecutionModel-7.0.so
LIBS += /usr/local/lib/libvtkFiltersGeometry-7.0.so
LIBS += /usr/local/lib/libvtkFiltersCore-7.0.so
LIBS += /usr/local/lib/libvtkFiltersSources-7.0.so
LIBS += /usr/local/lib/libvtkInteractionStyle-7.0.so
LIBS += /usr/local/lib/libvtkCommonTransforms-7.0.so
LIBS += /usr/local/lib/libvtkInteractionWidgets-7.0.so
LIBS += /usr/local/lib/libvtkFiltersGeneral-7.0.so
LIBS += /usr/local/lib/libvtkRenderingLOD-7.0.so

LIBS += /usr/lib/x86_64-linux-gnu/libboost_system.so
LIBS += /usr/lib/x86_64-linux-gnu/libboost_thread.so

LIBS += /usr/local/lib/libopencv_calib3d.so.2.4.11
LIBS += /usr/local/lib/libopencv_contrib.so.2.4.11
LIBS += /usr/local/lib/libopencv_core.so.2.4.11
LIBS += /usr/local/lib/libopencv_features2d.so.2.4.11
LIBS += /usr/local/lib/libopencv_flann.so.2.4.11
LIBS += /usr/local/lib/libopencv_gpu.so.2.4.11
LIBS += /usr/local/lib/libopencv_highgui.so.2.4.11
LIBS += /usr/local/lib/libopencv_imgproc.so.2.4.11
LIBS += /usr/local/lib/libopencv_legacy.so.2.4.11
LIBS += /usr/local/lib/libopencv_ml.so.2.4.11
LIBS += /usr/local/lib/libopencv_nonfree.so.2.4.11
LIBS += /usr/local/lib/libopencv_objdetect.so.2.4.11
LIBS += /usr/local/lib/libopencv_ocl.so.2.4.11
LIBS += /usr/local/lib/libopencv_photo.so.2.4.11
LIBS += /usr/local/lib/libopencv_stitching.so.2.4.11
LIBS += /usr/local/lib/libopencv_superres.so.2.4.11
LIBS += /usr/local/lib/libopencv_video.so.2.4.11
LIBS += /usr/local/lib/libopencv_videostab.so.2.4.11




LIBS += /usr/local/lib/libpcl_2d.so
LIBS += /usr/local/lib/libpcl_apps.so
LIBS += /usr/local/lib/libpcl_common.so
LIBS += /usr/local/lib/libpcl_features.so
LIBS += /usr/local/lib/libpcl_filters.so
LIBS += /usr/local/lib/libpcl_io.so
LIBS += /usr/local/lib/libpcl_io_ply.so
LIBS += /usr/local/lib/libpcl_kdtree.so
LIBS += /usr/local/lib/libpcl_keypoints.so
LIBS += /usr/local/lib/libpcl_ml.so
LIBS += /usr/local/lib/libpcl_octree.so
LIBS += /usr/local/lib/libpcl_outofcore.so
LIBS += /usr/local/lib/libpcl_people.so
LIBS += /usr/local/lib/libpcl_recognition.so
LIBS += /usr/local/lib/libpcl_registration.so
LIBS += /usr/local/lib/libpcl_sample_consensus.so
LIBS += /usr/local/lib/libpcl_search.so
LIBS += /usr/local/lib/libpcl_segmentation.so
LIBS += /usr/local/lib/libpcl_stereo.so
LIBS += /usr/local/lib/libpcl_surface.so
LIBS += /usr/local/lib/libpcl_tracking.so
LIBS += /usr/local/lib/libpcl_visualization.so

LIBS += /usr/local/lib/liblcm.so


HEADERS += \
    Param.h \
    ../include/EuclideanClusterExtraction.h \
    ../include/HDL32Grab.h \
    ../include/ICPOper.h \
    ../include/LidarDataProcess.h \
    ../include/linux.h \
    ../include/OpenCVInc.h \
    ../include/VLP16Graber.h
