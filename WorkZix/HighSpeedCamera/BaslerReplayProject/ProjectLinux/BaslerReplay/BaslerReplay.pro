TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    ../../src/DataFusion.cpp \
    ../../src/FileOpr.cpp \
    ../../src/LogDisplay.cpp \
    ../../src/Other.cpp \
    ../../src/RawDataReader.cpp \
    ../../src/UbloxReader.cpp \
    ../../src/BaslerReplay.cpp

INCLUDEPATH += /usr/local/include/opencv

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

HEADERS += \
    ../../src/DataFusion.h \
    ../../src/FileOpr.h \
    ../../src/LogDisplay.h \
    ../../src/OpenCVInc.h \
    ../../src/RawDataReader.h \
    ../../src/UbloxReader.h \
    ../../src/Other.h

