# Configs
TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++14

#Libraries
unix: CONFIG += link_pkgconfig

#OpenCV
unix: PKGCONFIG += opencv

#CUDA
unix:!macx: LIBS += -L$$PWD/../../../usr/local/cuda/lib64/-lcudart
unix:!macx: LIBS += -L$$PWD/../../../usr/local/cuda/lib64/-lcuda
unix:!macx: LIBS += -L$$PWD/../../../usr/local/cuda/lib64/-lcublas
unix:!macx: LIBS += -L$$PWD/../../../usr/local/cuda/lib64/-lcurand
INCLUDEPATH += $$PWD/../../../usr/local/cuda/include
DEPENDPATH += $$PWD/../../../usr/local/cuda/include

#V4L2
unix:!macx: LIBS += -lpthread
unix:!macx: LIBS += -lv4l2

#Darknet
unix:!macx: LIBS += -L$$PWD/Darknet/ -ldarknet
INCLUDEPATH += $$PWD/Darknet
DEPENDPATH += $$PWD/Darknet

#Source and header files
HEADERS += \
    General/singleton.hpp \
    General/opencv_extended.h \
    General/numeric_rm.h \
    General/General.h \
    Driver/RMVideoCapture.hpp \
    Serials/Serial.h \
    Main/ImgProdCons.h \
    Pose/Predictor.hpp \
    Pose/AngleSolver.hpp \
    Armor/ArmorDetector.h \
    Rune/Rune.h \

SOURCES += \
    General/opencv_extended.cpp \
    General/numeric_rm.cpp \
    Driver/RMVideoCapture.cpp \
    Serials/Serial.cpp \
    Main/ImgProdCons.cpp \
    Pose/Predictor.cpp \
    Pose/AngleSolver.cpp \
    Armor/ArmorDetector.cpp \
    Rune/Rune.cpp \
#   To test a sigular module, uncomment only one of the following
#    Armor/test_armor.cpp
#    Driver/test_camera.cpp
#    Serials/test_serials.cpp
#    Pose/test_angle_2.cpp
#    Pose/test_angle_1.cpp
#   To test a singular module with multi-thread, uncomment main.cpp
#   and one of the others
    Main/main.cpp \
#    Main/test_armor_temp.cpp
#    Main/test_producer.cpp
#    Main/test_armor.cpp
#    Main/test_armor_solver.cpp
#    Main/test_rune.cpp \
#    Main/test_armor_solver_serial.cpp
#     Main/test_recording_video.cpp
    Main/test_infantry.cpp

