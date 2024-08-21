QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
CONFIG += "lang-zh_CN"

RC_ICONS = res/SWR.ico

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    src/main.cpp \
    src/mainwindow.cpp \
    src/mypushbutton.cpp \
    src/point.cpp \
    src/robot.cpp

HEADERS += \
    inc/craft.h \
    inc/mainwindow.h \
    inc/mypushbutton.h \
    inc/point.h \
    inc/robot.h \
    lib/agp/include/AGP.h \
    lib/hans/include/HR_Pro.h \
    lib/duco/shared/include/DucoCobot.h \
    lib/jaka/inc_of_c++/JAKAZuRobot.h \
    lib/jaka/inc_of_c++/jkerr.h \
    lib/jaka/inc_of_c++/jktypes.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    res/logo.qrc

FORMS += \
    res/mainwindow.ui

INCLUDEPATH += \
    $$PWD/inc \
    $$PWD/lib/agp/include

# Hans
win32: LIBS += -L$$PWD/lib/hans/x64/Release/ -lHR_Pro

INCLUDEPATH += $$PWD/lib/hans/include
DEPENDPATH += $$PWD/lib/hans/x64/Release

# Duco
win32: LIBS += -L$$PWD/lib/duco/shared/VS2019/win64/release/ -lDucoCobotAPI

INCLUDEPATH += $$PWD/lib/duco/shared/include
DEPENDPATH += $$PWD/lib/duco/shared/VS2019/win64/release

# Jaka
win32: LIBS += -L$$PWD/lib/jaka/x64/ -ljakaAPI

INCLUDEPATH += $$PWD/lib/jaka/inc_of_c++
DEPENDPATH += $$PWD/lib/jaka/x64
