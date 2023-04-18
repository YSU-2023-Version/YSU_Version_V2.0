QT -= core
QT -= gui

#TARGET = YSU_2021_Infantry
TARGET =YSU_2022_Sentry
CONFIG += console
CONFIG -= app_bundle
CONFIG += c++14
TEMPLATE = app


INCLUDEPATH += /usr/local/include \
/usr/local/include/opencv \
/usr/local/include/opencv2 \
#/home/nano/linuxSDK_V2.1.0.2/include \
/usr/local/opencv/linuxsdk/include \
/usr/local/include/gsl \
/home/zjw/intel/openvino_2022.1.0.643/runtime/include \
/home/zjw/intel/openvino_2022.1.0.643/runtime/include/ie \
/home/zjw/intel/openvino_2022.1.0.643/runtime/include/ngraph \
/home/zjw/intel/openvino_2022.1.0.643/runtime/include/openvino

LIBS += /usr/local/lib/libopencv_calib3d.so \
/usr/local/lib/libopencv_core.so \
/usr/local/lib/libopencv_features2d.so \
/usr/local/lib/libopencv_flann.so \
/usr/local/lib/libopencv_highgui.so \
/usr/local/lib/libopencv_imgcodecs.so \
/usr/local/lib/libopencv_imgproc.so \
/usr/local/lib/libopencv_ml.so \
/usr/local/lib/libopencv_objdetect.so \
/usr/local/lib/libopencv_photo.so \
/usr/local/lib/libopencv_shape.so \
/usr/local/lib/libopencv_stitching.so \
/usr/local/lib/libopencv_superres.so \
/usr/local/lib/libopencv_videoio.so \
/usr/local/lib/libopencv_video.so \
/usr/local/lib/libopencv_videostab.so\
/home/zjw/1/lib/x64/libMVSDK.so \
/usr/local/lib/libgsl.so \
/usr/local/lib/libgslcblas.so \
/home/zjw/intel/openvino_2022.1.0.643/runtime/lib/intel64/libopenvino.so \
-L/home/zjw/intel/openvino_2022.1.0.643/runtime/lib/intel64 -lopenvino
#                                                -lngraph \
#                                                -ltbb \


HEADERS += \
    Communication/SerialPort.hpp \
    DeepLearning/yolov5.h \
    ThreadManager/thread_manager.h \
    ArmorDetector/armor_detector.h \
    RuneDetector/rune_detector.h \
    CameraManager/camera_manager.h \
    Classify/classify.h \
    Communication/communication.h \
    Pose/angle_solver.h \
    Main/headfiles.h \
    Timer/Timer.h \
    Timer/Log.h \
    Forecast/forecast.h \
    Kalman/kalman.h \
    SVM/svm.h \
    Forecast/ysu_gsl.h
 #   Forecast/PID.h \




SOURCES += \
    Communication/SerialPort.cpp \
    DeepLearning/yolov5.cpp \
    ThreadManager/thread_manager.cpp \
    ArmorDetector/armor_detector.cpp \
    RuneDetector/rune_detector.cpp \
    CameraManager/camera_manager.cpp \
    Classify/classify.cpp \
    Communication/communication.cpp \
    Main/main.cpp \
    Pose/angle_solver.cpp \
    Timer/Timer.cpp \
    Forecast/forecast.cpp \
    Kalman/kalman.cpp \
    SVM/svm.cpp \
    Forecast/ysu_gsl.cpp
 #   Forecast/PID.cpp \





