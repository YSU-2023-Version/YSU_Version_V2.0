# 燕山大学Robomaster燕鹰战队源代码

*TrackDemo分支用于优化代码结构和开发预测最新的模块*
*Master分支已经是可以正常使用的版本了*

``` shell
.
├── model
│   ├── best.bin
│   ├── best.mapping
│   ├── best.onnx
│   ├── best.xml
│   ├── model
│   │   ├── opt-0527-001.bin
│   │   ├── opt-0527-001.onnx
│   │   ├── opt-0527-001.xml
│   │   └── opt-0625-001.onnx
├── README.md
├── temp
├── tools
│   └── watchdog.sh
├── xml_path
│   ├── armor_limited.xml
│   ├── camera.xml
│   ├── forecast.xml
│   ├── run
│   └── svm.xml
└── YSU_2022_stentry
    ├── ArmorDetector
    │   ├── armor_detector.cpp
    │   └── armor_detector.h
    ├── ArmorTrack
    │   ├── armor_track.cpp
    │   ├── armor_track.h
    │   ├── Detector
    │   │   ├── yolov5.cpp
    │   │   └── yolov5.h
    │   └── Predictor
    │       ├── armor_predictor.cpp
    │       └── armor_predictor.h
    ├── Camera
    │   ├── Data
    │   └── log
    ├── CameraManager
    │   ├── camera_manager.cpp
    │   └── camera_manager.h
    ├── Classify
    │   ├── classify.cpp
    │   └── classify.h
    ├── CMakeList.txt
    ├── Communication
    │   ├── communication.cpp
    │   ├── communication.h
    │   ├── Exception.hpp
    │   ├── SerialPort.cpp
    │   └── SerialPort.hpp
    ├── DeepLearning
    │   ├── yolov5.cpp
    │   └── yolov5.h
    ├── Forecast
    │   ├── forecast.cpp
    │   ├── forecast.h
    │   ├── ysu_gsl.cpp
    │   └── ysu_gsl.h
    ├── infantry.h
    ├── Kalman
    │   ├── kalman.cpp
    │   └── kalman.h
    ├── Main
    │   ├── headfiles.h
    │   └── main.cpp
    ├── Makefile
    ├── Pose
    │   ├── angle_solver.cpp
    │   ├── angle_solver.h
    │   └── camera.xml
    ├── RuneDetector
    │   ├── buff_.xml
    │   ├── double_camera.xml
    │   ├── rune_detector.cpp
    │   ├── rune_detector.h
    ├── SVM
    │   ├── svm.cpp
    │   ├── svm.h
    ├── ThreadManager
    │   ├── thread_manager.cpp
    │   └── thread_manager.h
    ├── Timer
    │   ├── Log.h
    │   ├── Timer.cpp
    │   └── Timer.h
    ├── YSU_2022_Sentry.pro
```
