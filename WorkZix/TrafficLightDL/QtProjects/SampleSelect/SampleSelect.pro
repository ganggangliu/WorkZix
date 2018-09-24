TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    ItemSelect.cpp

INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /home/zix-kotei/zix/tensorflow-master/bazel-tensorflow-master
INCLUDEPATH += /home/zix-kotei/zix/tensorflow-master
INCLUDEPATH += /home/zix-kotei/zix/tensorflow-master/bazel-genfiles
INCLUDEPATH += /home/zix-kotei/zix/tensorflow-master/bazel-tensorflow-master/external/eigen_archive

LIBS += /home/zix-kotei/zix/tensorflow-master/bazel-bin/tensorflow/libtensorflow_cc.so

LIBS += /usr/local/lib/libopencv_calib3d.so.2.4.13
LIBS += /usr/local/lib/libopencv_contrib.so.2.4.13
LIBS += /usr/local/lib/libopencv_core.so.2.4.13
LIBS += /usr/local/lib/libopencv_features2d.so.2.4.13
LIBS += /usr/local/lib/libopencv_flann.so.2.4.13
LIBS += /usr/local/lib/libopencv_gpu.so.2.4.13
LIBS += /usr/local/lib/libopencv_highgui.so.2.4.13
LIBS += /usr/local/lib/libopencv_imgproc.so.2.4.13
LIBS += /usr/local/lib/libopencv_legacy.so.2.4.13
LIBS += /usr/local/lib/libopencv_ml.so.2.4.13
LIBS += /usr/local/lib/libopencv_nonfree.so.2.4.13
LIBS += /usr/local/lib/libopencv_objdetect.so.2.4.13
LIBS += /usr/local/lib/libopencv_ocl.so.2.4.13
LIBS += /usr/local/lib/libopencv_photo.so.2.4.13
LIBS += /usr/local/lib/libopencv_stitching.so.2.4.13
LIBS += /usr/local/lib/libopencv_superres.so.2.4.13
LIBS += /usr/local/lib/libopencv_video.so.2.4.13
LIBS += /usr/local/lib/libopencv_videostab.so.2.4.13

LIBS += /usr/lib/x86_64-linux-gnu/libboost_system.so.1.58.0
LIBS += /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.58.0


HEADERS +=\
    OpenCVInc.h \
    ItemSelect.h
