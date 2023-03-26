QT -= gui
QT += core serialport network

CONFIG += c++11 console
CONFIG -= app_bundle
CONFIG(release, debug|release):DEFINES += QT_NO_DEBUG_OUTPUT
TARGET = QT_Server_Serial2Arm3
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
DEFINES += QT_DEPRECATED_WARNINGS

HEADERS += \
    ../sharedfiles/mkglobalclass.h \
    ../sharedfiles/mkRobotKin.h \
    ../sharedfiles/mkringbuffer.h \
    helper.h \
    mkredis.h \
    mkserialport.h \
    mkserver_serial2arm3.h \
    mksocketserver.h \
    rediqtadapter.h

INCLUDEPATH += /usr/local/include/hiredis/
INCLUDEPATH += /usr/local/include/hiredis/adapters/
INCLUDEPATH += /usr/local/include
LIBS += -L/usr/local/Cellar/hiredis/1.1.0/lib -lhiredis
LIBS += -L/usr/local/Cellar/libevent/2.1.12/lib -levent

SOURCES += \
    ../sharedfiles/mkmatrix.cpp \
    helper.cpp \
    main.cpp \
    mkredis.cpp \
    mkserialport.cpp \
    mkserver_serial2arm3.cpp \
    mksocketserver.cpp


TRANSLATIONS += \
    QT_TcpClient_Serial2Arm3_en_US.ts

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
#qnx: target.path = /home/pi/QT
#else: unix:!android: target.path = /home/pi/QT
!isEmpty(target.path): INSTALLS += target
