#include "mkserver_serial2arm3.h"
#include<iostream>
#include<cstring>
#include <QFile>
#include <QStringList>
#include <QThread>
#include <QThread>
#include <QTime>
#include <QLibraryInfo>
#include "helper.h"
#include "../sharedfiles/mkringbuffer.h"
#include "../sharedfiles/mkglobalclass.h"
#include "mkSocketThread.h"


//#define BAUDRATE 250000 //115200
//#define BAUDRATE 9600
#define BAUDRATE 115200
#define DEG2RAD  (M_PI/180.0)
#define RAD2DEG  (180.0/M_PI)

double Z_STEP2DIST = double(Z_DIST_PER_REV)/double(Z_MICROSTEPPING);
double Z_DIST2STEP =double( Z_MICROSTEPPING)/double(Z_DIST_PER_REV);

static const int WRITE_DELAY=50;
//#define MK_BUFSIZE 16
//#define MK_MAX_CMD_SIZE 96


#ifdef Q_OS_LINUX
const char *port[]={ "/dev/ttyACM0"};
#define CONFIG_FILE "/home/mike/Developer/serial2arm_config"
#elif defined(Q_OS_MAC)
//const char *port[]={"/dev/cu.usbmodem143401"};
const char *port[]={"/dev/cu.usbmodem14101"};
//const char *port[]={"/dev/cu.usbmodem14301"};
#define CONFIG_FILE "/Users/mikekim/Developer/serial2arm_config"
//#define CONFIG_FILE "/Users/mikekim/Developer/serial2arm_config_test"
#endif

//mkRingBuffer ringBuffer;
extern  Helper con;


void display(redisReply *reply){
    if(reply->type == REDIS_REPLY_STRING){
        qDebug() <<reply->str<<Qt::endl;
    }
    else if(reply->type == REDIS_REPLY_ARRAY){
        for (size_t i = 0; i < reply->elements; i++ ) {
            qDebug()<<  reply->element[i]->str;//<<" = "<<reply->element[i + 1]->str ;
          }
    }
    else {
        qDebug()<< "type: " << reply->type<<Qt::endl;
    }
    freeReplyObject(reply);

}

mkServer_Serial2Arm3::mkServer_Serial2Arm3(QObject *parent): QObject(parent)
{
    app = QCoreApplication::instance();
    ////////////////////////////////////////////
    // *********** DEFINE MOTOR PARAMETERS *****************
    // TRANSLATIONAL JOINT
    int CH=0;

    CH=0; // X-AXIS
    robotProperty[CH].ID = CH;
    robotProperty[CH].jointType = JT_LIN;
    robotProperty[CH].maxDistance = 1650.0;
    robotProperty[CH].maxVel=400;//600.0;//[mm/sec]
    robotProperty[CH].maxAccel=450;//800.0; //[mm/sec^2]
    robotProperty[CH].LEADPITCH = 100.0;//X_DIST_PER_REV;//  100mm/rev (=20TH*5MM)
    robotProperty[CH].MICROSTEPPING = 1600;//1600*2;//X_MICROSTEPPING;
    robotProperty[CH].STEP2DIST = robotProperty[CH].LEADPITCH /robotProperty[CH].MICROSTEPPING;//[mm];
    robotProperty[CH].DIST2STEP = 1.0/robotProperty[CH].STEP2DIST;
    robotProperty[CH].homingPosStage[0]=-1700;//mm
    robotProperty[CH].homingPosStage[1]=100;  //mm
    robotProperty[CH].homingPosStage[2]=5;    //mm
    robotProperty[CH].homingVelStage[0]=40;   //mm/s
    robotProperty[CH].homingVelStage[1]=20;  //mm/s
    robotProperty[CH].homingVelStage[2]=20;    //mm/s
    robotProperty[CH].homingAccelStage[0]=40;   // [mm/s^2]
    robotProperty[CH].homingAccelStage[1]=20;   // [mm/s^2]
    robotProperty[CH].homingAccelStage[2]=10;   // [mm/s^2]
    robotProperty[CH].centerPos = 0;
    robotProperty[CH].print();
    //////////////////////////////////////////////////
    // REVOLUTE JOINT
    CH=1; // R1-AXIS
    robotProperty[CH].ID = CH;
    robotProperty[CH].jointType = JT_ROT;
    robotProperty[CH].gearRatio = 3*3;       //R1_GEAR_RATIO;
    robotProperty[CH].maxDistance = 228.252*DEG2RAD;//232.207*DEG2RAD; //[rad] on output gear
    robotProperty[CH].maxVel=200.0*DEG2RAD;//120.0*DEG2RAD;        //[rad/sec]
    robotProperty[CH].maxAccel=400.0*DEG2RAD;//100.0*DEG2RAD;      //[rad/sec^2]
    robotProperty[CH].LEADPITCH = 2.0*M_PI/robotProperty[CH].gearRatio; //[rad/rev] R1_ANGLE_PER_REV;
    robotProperty[CH].MICROSTEPPING = 1600;//1600*2;//R1_MICROSTEPPING;
    robotProperty[CH].STEP2DIST = robotProperty[CH].LEADPITCH /robotProperty[CH].MICROSTEPPING;//R1_STEP2ANGLE [rad];
    robotProperty[CH].DIST2STEP = 1.0/robotProperty[CH].STEP2DIST; //[R1_ANGLET2STEP;
    robotProperty[CH].homingPosStage[0]=-200;//deg [Approaching to the Home End Stop S-W until S-W is On.
    robotProperty[CH].homingPosStage[1]=10;  //deg [When H-ES was hitted, then move until S-W is Off.
    robotProperty[CH].homingPosStage[2]=2;   //deg [Move 2 degrees off from S-W.
    robotProperty[CH].homingVelStage[0]=25;   // [deg/s]
    robotProperty[CH].homingVelStage[1]=15;   // [deg/s]
    robotProperty[CH].homingVelStage[2]=15;   // [deg/s]
    robotProperty[CH].homingAccelStage[0]=25; // [deg/s^2]
    robotProperty[CH].homingAccelStage[1]=15; // [deg/s^2]
    robotProperty[CH].homingAccelStage[2]=15;  // [deg/s^2]
    robotProperty[CH].centerPos = 120.8490;//122.256;//95.0; //[deg] from Home->End S-W
    robotProperty[CH].print();
    //////////////////////////////////////////////////
    // REVOLUTE JOINT
    CH=2; // R2-AXIS
    robotProperty[CH].ID = CH;
    robotProperty[CH].jointType = JT_ROT;
    robotProperty[CH].gearRatio = 3;//R2_GEAR_RATIO;
    robotProperty[CH].maxDistance = 290.654*DEG2RAD;// [rad] on output gear
    robotProperty[CH].maxVel=200.0*DEG2RAD;//120.0*DEG2RAD;  //[rad/sec]
    robotProperty[CH].maxAccel=400.0*DEG2RAD;//100.0*DEG2RAD;//[rad/sec^2]
    robotProperty[CH].LEADPITCH = 2.0*M_PI/robotProperty[CH].gearRatio;//[rad/rev] R2_ANGLE_PER_REV;
    robotProperty[CH].MICROSTEPPING = 1600*4; //R2_MICROSTEPPING;
    robotProperty[CH].STEP2DIST = robotProperty[CH].LEADPITCH /robotProperty[CH].MICROSTEPPING;//R2_STEP2ANGLE[step->rad];
    robotProperty[CH].DIST2STEP = 1.0/robotProperty[CH].STEP2DIST; //R2_ANGLET2STEP [rad->step];
    robotProperty[CH].homingPosStage[0]=-360;//deg [Approaching to the Home End Stop S-W until S-W is On.
    robotProperty[CH].homingPosStage[1]=10;  //deg [When H-ES was hitted, then move until S-W is Off.
    robotProperty[CH].homingPosStage[2]=2;   //deg [Move 2 degrees off from S-W.
    robotProperty[CH].homingVelStage[0]=50;   // [deg/s]
    robotProperty[CH].homingVelStage[1]=25;   // [deg/s]
    robotProperty[CH].homingVelStage[2]=25;   // [deg/s]
    robotProperty[CH].homingAccelStage[0]=50; // [deg/s^2]
    robotProperty[CH].homingAccelStage[1]=25; // [deg/s^2]
    robotProperty[CH].homingAccelStage[2]=25;  // [deg/s^2]
    robotProperty[CH].centerPos = 150.8230;//153.0; //[deg] from Home->End S-W
    robotProperty[CH].print();
    ////////////////////////////////////////////
    // TRANSLATIONAL JOINT
    CH=3;// Z-AXIS
    robotProperty[CH].ID = CH;
    robotProperty[CH].jointType = JT_LIN;
    robotProperty[CH].maxDistance = 1000.0;
    robotProperty[CH].maxVel=400;//170.0;//[mm/sec]
    robotProperty[CH].maxAccel=450;//170.0; //[mm/sec^2]
    robotProperty[CH].LEADPITCH = 100.0;//X_DIST_PER_REV;//  100mm/rev (=20TH*5MM)
    robotProperty[CH].MICROSTEPPING = 1600;//1600*2;//X_MICROSTEPPING;
    robotProperty[CH].STEP2DIST = robotProperty[CH].LEADPITCH /robotProperty[CH].MICROSTEPPING;//[mm];
    robotProperty[CH].DIST2STEP = 1.0/robotProperty[CH].STEP2DIST;
    robotProperty[CH].homingPosStage[0]=-1700;//mm
    robotProperty[CH].homingPosStage[1]=100;  //mm
    robotProperty[CH].homingPosStage[2]=5;    //mm
    robotProperty[CH].homingVelStage[0]=50;   //mm/s
    robotProperty[CH].homingVelStage[1]=50;  //mm/s
    robotProperty[CH].homingVelStage[2]=50;    //mm/s
    robotProperty[CH].homingAccelStage[0]=50;   // [mm/s^2]
    robotProperty[CH].homingAccelStage[1]=25;   // [mm/s^2]
    robotProperty[CH].homingAccelStage[2]=25;   // [mm/s^2]
    robotProperty[CH].centerPos = 0;
    robotProperty[CH].print();
    /*
    // Old setup for ball screw for z-axis
    CH=3;// Z-AXIS
    robotProperty[CH].ID = CH;
    robotProperty[CH].jointType = JT_LIN;
    robotProperty[CH].maxDistance = 1000.0; //[mm]
    robotProperty[CH].maxVel=50;//110.0;//60.0; //[mm/sec]
    robotProperty[CH].maxAccel=robotProperty[CH].maxVel*1.0; //[mm/sec^2]
    robotProperty[CH].LEADPITCH = 5;//10.0;//5.0; //Z_DIST_PER_REV;//5mm/rev (Ball Screw Lead pitch)
    robotProperty[CH].MICROSTEPPING = 400;//1600;// 800;//400; //Z_MICROSTEPPING;
    robotProperty[CH].STEP2DIST = robotProperty[CH].LEADPITCH /robotProperty[CH].MICROSTEPPING;//Z_STEP2DIST;
    robotProperty[CH].DIST2STEP = 1.0/robotProperty[CH].STEP2DIST; //Z_DIST2STEP;
    robotProperty[CH].homingPosStage[0]=-1100;//mm
    robotProperty[CH].homingPosStage[1]=100;  //mm
    robotProperty[CH].homingPosStage[2]=5;    //mm
    robotProperty[CH].homingVelStage[0]=25;   //mm/s
    robotProperty[CH].homingVelStage[1]=25;  //mm/s
    robotProperty[CH].homingVelStage[2]=25;    //mm/s
    robotProperty[CH].homingAccelStage[0]=25;   // [mm/s^2]
    robotProperty[CH].homingAccelStage[1]=25;   // [mm/s^2]
    robotProperty[CH].homingAccelStage[2]=25;   // [mm/s^2]
    robotProperty[CH].centerPos = 0;
    robotProperty[CH].print();
*/
}
mkServer_Serial2Arm3::~mkServer_Serial2Arm3()
{
    //    action_controlZBrake(false);// Turn off the power automatically
    //     delay(1000);
    if(tcpServerForOrder) delete tcpServerForOrder;
}
// --------------------------------------------------------------------------------------
void mkServer_Serial2Arm3::aboutToQuitApp()
{
    action_controlPower(false);// Turn off the power automatically
    savePos(".txt", robotProperty);
    delay(2000);

    cout<<"mkServer_Serial2Arm3  is killed"<<endl<<flush;
    qDebug()<<"aboutToQuitApp";
    // stop threads
    //         sleep(1);   // wait for threads to stop.
    // delete any objects
}
void mkServer_Serial2Arm3::startRun()
{
    qDebug() << "mkServer_Serial2Arm3.Run is executing";
    loadConfigFile(".txt",robotProperty);
    startSerialPort();
    connect(&socketOrderResponseTimer, &QTimer::timeout, this, &mkServer_Serial2Arm3::processSocketOrderTimer);
    connect(&socketCameraResponseTimer, &QTimer::timeout, this, &mkServer_Serial2Arm3::processSocketCameraTimer);
    connect(&socketExecuteTimer,  &QTimer::timeout, this, &mkServer_Serial2Arm3::processSocketExecuteTimer);
    connect(&serialSendTimer, &QTimer::timeout, this, &mkServer_Serial2Arm3::processSerialPortSendTimer);
    connect(&serialInitTimer, &QTimer::timeout, this, &mkServer_Serial2Arm3::processSerialPortInitTimer);
    connect(&serialTimeOut, &QTimer::timeout, this, &mkServer_Serial2Arm3::processSerialPortTimeOut);
    connect(qApp, SIGNAL(aboutToQuit()), this, SLOT(aboutToQuitApp()));

    //////////////////////////////////////////////////
    // Redis client
    mkRedis.run();
    //////////////////////////////////////////////////
    // Server for Ordering client
    tcpServerForOrder = new mkSocketServer(this);
    tcpServerForOrder->startServer(8888);
    connect(tcpServerForOrder, SIGNAL(connected()), this, SLOT(socketOrderConnected_slot()));

    /////////////////////////////////////////////////
    // Server for Position and Orientation of  tag w.r.t camera client
    tcpServerForCamera = new mkSocketServer(this);
    tcpServerForCamera->startServer(8880);
    connect(tcpServerForCamera, SIGNAL(connected()), this, SLOT(socketCameraConnected_slot()));

    ////////////////////////////////////
    socketOrderResponseTimer.start(20);
    socketExecuteTimer.start(20);

    socketCameraResponseTimer.start(20);
    //


}

void mkServer_Serial2Arm3::socketOrderConnected_slot()
{
    tcpServerForOrder->isConnected=true;
    cout<<"socket_Order is connected:"<<endl<<flush;
}
void mkServer_Serial2Arm3::socketCameraConnected_slot()
{
    tcpServerForCamera->isConnected=true;
    cout<<"socket_Camera is connected:"<<endl<<flush;
}

void mkServer_Serial2Arm3::startSerialPort()
{
    bSerialProcessDone=false;
    serialResponseTimer.stop();
    for(int i=0; i<NUM_PORT;i++)
    {
        if(serialPort[i].isOpen()) serialPort[i].close();
        if(serialHandler[i]) delete serialHandler[i];
        serialHandler[i] = new mkSerialPort(&serialPort[i]);
        serialPort[i].setPortName(port[i]);
        serialPort[i].setBaudRate(BAUDRATE);
        serialPort[i].setDataBits(QSerialPort::Data8);
        serialPort[i].setFlowControl(QSerialPort::NoFlowControl);
        serialPort[i].setStopBits(QSerialPort::OneStop);
        serialPort[i].setParity(QSerialPort::NoParity);
        serialPort[i].setReadBufferSize (400);

        bool rec = serialPort[i].open(QIODevice::ReadWrite);
        if( !rec) {
            qDebug()<<"Serial is not open: "<< port[i];//"/dev/ttyACM0"
            serialPort[i].setPortName("/dev/ttyACM0");
        }
        else {
            qDebug()<<"Serial is open: "<< port[i];//"/dev/ttyACM0"
        }
        serialData[i].clear();
    }

    connect(&serialPort[0], &QSerialPort::aboutToClose, this, &mkServer_Serial2Arm3::serialAboutClose);
    connect(&serialResponseTimer, &QTimer::timeout, this, &mkServer_Serial2Arm3::processSerialPortResponseTimer);
    serialResponseTimer.start(1);
    qDebug()<<"startSerialPort()";
    bSerialProcessDone=true;




}




void mkServer_Serial2Arm3::serialAboutClose()
{
    //    action_controlPower(false);// Turn off the power automatically
    //    savePos(".txt", robotProperty);
    //    delay();
    qDebug()<<"Serial is about to close:: ="<<robotProperty[0].absSteps;
}

// ****************************************************************************** //
// *------------- Called processSocketTimer() every 1msec ----------------------* //
// * Processing ringbuffer command from IStacker Planner TCP Socket Client...   * //
void mkServer_Serial2Arm3::processSocketOrderTimer()
{
    if(tcpServerForOrder->socket==0) return;
    if(tcpServerForOrder->ringBuffer.isEmpty()) return;
    if(!bSocketOrderProcessDone) return; // if the socket process is not finished, then skip 1msec timeout...
    bSocketOrderProcessDone=false;

    qDebug()<<"---->Received data from Order Client: "<<tcpServerForOrder->ringBuffer.getCmd()<<endl<<flush;
    if(tcpServerForOrder->ringBuffer.codeSeen('J')) {
        //////////////////////////////////////////////
        // If we got "SC_STOP", then cancel all previous job and send stop to robot server...
        if(tcpServerForOrder->ringBuffer.codeSeen('G')) {
            int codeValue = (int)tcpServerForOrder->ringBuffer.codeValue();

            if(SC_STOP==codeValue) {
                tcpServerForOrder->ringBuffer.codeSeen('J');
                int jobID = (int)tcpServerForOrder->ringBuffer.codeValue();
                action_stop(codeValue,jobID);
                bRobotMoving=false;
                return;
            }
            else if(SC_SAVE_POS==codeValue) {
                cout <<"SC_SAVE_POS==codeValue"<<endl<<flush;
                tcpServerForOrder->ringBuffer.readDone();
                savePos(".txt", robotProperty);
                bSocketOrderProcessDone=true;
                return;
            }
        }
        // Stop is highest priority...
        //////////////////////////////////////////////////////////////////////////
        PacketJobs *newJob = new PacketJobs;
        newJob->unpack(tcpServerForOrder->ringBuffer.getCmd());
        queuePacketJobs.push(newJob);
        if( QLibraryInfo::isDebugBuild()) newJob->print();
    }
    /////////////////////////////////////////////////
    tcpServerForOrder->ringBuffer.readDone();
    bSocketOrderProcessDone=true;
}

void mkServer_Serial2Arm3::processSocketCameraTimer()
{
    if(tcpServerForCamera->socket==0) return;
    if(tcpServerForCamera->ringBuffer.isEmpty()) return;
    if(!bSocketCameraProcessDone) return; // if the socket process is not finished, then skip 1msec timeout...
    bSocketCameraProcessDone=false;

    qDebug()<<"Received data from Camera Client: "<<tcpServerForCamera->ringBuffer.getCmd()<<endl<<flush;

    if(tcpServerForCamera->ringBuffer.codeSeen('J')) {
        //ID = (int)tcpServerForOrder->ringBuffer.codeValue();
        PacketCam *newJob = new PacketCam;
        newJob->unpack(tcpServerForCamera->ringBuffer.getCmd());
        newJob->print();
    }
    /////////////////////////////////////////////////
    tcpServerForCamera->ringBuffer.readDone();
    bSocketCameraProcessDone=true;
}
////////////////////////////////////////////
/// \brief mkServer_Serial2Arm3::processSocketExecuteTimer
/// Sending Commands to the robot...
void mkServer_Serial2Arm3::processSocketExecuteTimer()
{
    // return;
    // if(!bIsStartedJobForRobot) return;

    if(jobIDDone==-1 && !queuePacketJobs.empty() ) {
        //isStartedJobForRobot=true;
//        serialSendTimer.stop();
        qDebug()<<"queue size: "<<queuePacketJobs.size();
        PacketJobs *job = queuePacketJobs.front();
        currentPacketJob = *job;
        //currentSequence = job->getSequenceNumber();
        switch(job->cmdCode) {
        case SC_REBOOT:
            bRobotMoving=false;
            jobIDDone = job->getJobID();
            action_reboot(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_TIME_DELAY:
            jobIDDone = job->getJobID();
            bDelayOp=true;
            delay(job->mode);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_TIME_DELAY_MC:
            jobIDDone = job->getJobID();
            action_delayMicroController(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_POWER:
            jobIDDone = job->getJobID();
            action_controlPower(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_Z_BRAKE:
            jobIDDone = job->getJobID();
            action_controlZBrake(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_SETPOS:
            jobIDDone = job->getJobID();
            action_setPosition(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_SET_SPEED: {
            serialSendTimer.stop();
            jobIDDone = job->getJobID();
            bool ret = action_genSeedProfile(job);
            if(ret) {
                queuePacketJobs.pop();
//                if(job) delete job;
            }
            return;
        }
        case SC_MOVE:
            jobIDDone = job->getJobID();
            bRobotMoving=true;
            qDebug()<<"+++ case SC_MOVE: +++ jobID="<<jobIDDone<<", N="<<job->getSequenceNumber()<<", selected Motor"<<statusAllSpeedReady.selectMotors;
            action_moveRobotStation(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_GEN_EELINEAR:
            serialSendTimer.stop();
            jobIDDone = job->getJobID();
            action_genLinearMotion(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_GEN_EEROTATION:
            serialSendTimer.stop();
            jobIDDone = job->getJobID();
            action_genRotateEEMotion(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_GEN_CIRCLE:
            serialSendTimer.stop();
            jobIDDone = job->getJobID();
            action_genCircularMotion(job);
            //
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_GEN_SPIRAL:
            serialSendTimer.stop();
            jobIDDone = job->getJobID();
            action_genSpiralMotion(job);
            //jobIDDone = -1;
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_STATUS:
            serialSendTimer.stop();
            jobIDDone = job->getJobID();
            action_requestStatus(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_STATUS_ALL_POS:
            serialSendTimer.stop();
            jobIDDone = job->getJobID();
            action_requestAllPosition(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_GET_ENCODER:
            //return;// for test
            jobIDDone = job->getJobID();
            action_requestEncoderValue(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_SET_ZERO_ENCODER:
            jobIDDone = job->getJobID();
            action_setZeroEncoder(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_DROP_CUP:
            jobIDDone = job->getJobID();
            action_dropCup(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_HOMING:
            // By Using Status Info which SC_STATUS is issued we can check if
            // Robot already contacted to HomeS/W and statusS/W is 0 or 1
            // Skip stage 1: approach to homeS/W...
            if( job->getSequenceNumber()==2 &&
                    packetStatusRec.switchHome==1 &&
                    packetStatusRec.statusHoming<2
                    ) {
                jobIDDone = -1;
            }
            else {
                jobIDDone = job->getJobID();
                action_homing(job);
            }
            queuePacketJobs.pop();
//            if(job) delete job;
            return;
        case SC_ORDER:
            jobIDDone = job->getJobID();
            action_order(job);
            queuePacketJobs.pop();
//            if(job) delete job;
            return;

        }

    }

}



// ********************************************************************************** //

// *------------- Called processSerialPortTimer() every 1msec ----------------------* //
// *Processing ringbuffer command from the ARM micro controller                     * //
void mkServer_Serial2Arm3::processSerialPortResponseTimer()
{


    if(serialHandler[0]->readBuffer.isEmpty()) return;
//     qDebug()<<"tData";
    if(!bSerialProcessDone) return; // if the serial process is not finished, then skip 1msec timeout...

    bSerialProcessDone=false;
    char tData[128];
    int responseCode=-1, commanedCode=-1;

    //robotProperty[selectedMotorID].processStatus=0;
    strcpy(tData, serialHandler[0]->readBuffer.getCmd());

    bool found13=false;
    if(serialHandler[0]->readBuffer.codeSeen('G')){
        if(serialHandler[0]->readBuffer.codeValue()==13) found13=true;
    }
    if(!found13 && strlen(tData)>1)cout<<"    >>read: "<<tData<<endl;

/*
// ........ CRC TEST ............
//    QByteArray foo("\xff\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xaa");
 const char pattern[] = "G12M2S9112A2666C6446D2666T4295V0.381O1J106N1";

 size_t len = std::strlen(pattern);

 // .... Beginning ...
    char frame[128] ;

    frame[0]=char(DEC2HEX[255]);
    frame[1]=char(DEC2HEX[len+4]);
    int N=0;
    // ... Data ...
    for(N=0;N<len; N++){
        frame[2+N]=DEC2HEX[(unsigned char)(pattern[N])];
    }
    // ... add CRC to the end ...
    unsigned short crc = calculateCRC16Xmodem(frame, N+2);
    frame[N+2]=char(DEC2HEX[crc>>8]);//0x06;
    frame[N+3]=char(DEC2HEX[(crc<<8 & 0xFF00)>>8]);//0xCC;
    frame[N+4]='\0';

    // test to make error
    // frame[5]=data1[100];
     QByteArray hex1=QByteArray::fromRawData((char*)frame,std::strlen(frame));
    serialHandler[0]->write(hex1);
*/

//if(serialHandler[0]->readBuffer.codeSeen('!')){
//    cout<<"Received Cmd from Robot: "<<tData<<endl<<flush;
//    return;
//}
//        cout<<"Received Cmd from Robot: "<<tData<<endl<<flush;
//    if(tcpServerForOrder->isConnected && tcpServerForOrder->socket->isOpen())
//        tcpServerForOrder->socket->write(tData);
//FROM ZOEROBOTICS CONTROLLER!
    /////////////////////////////////////////////////////////////
    // Initializing Motion Controller...
    if(serialHandler[0]->readBuffer.codeSeen('!')==1) { // !!!! REMOVE "==1" IN REAL OPERATION !!!!
        int endIndex = serialHandler[0]->readBuffer.codeSeen('!');
        char initWords[128];
        strcpy(initWords,serialHandler[0]->readBuffer.curBuffer);
        initWords[endIndex]='\0';
        if(strcmp(initWords,"FROM ZOEROBOTICS CONTROLLER!")==0) {
            //tcpServer->socket->write(serialHandler[0]->readBuffer.getCmd());
            qDebug()<<"I got it: FROM ZOEROBOTICS CONTROLLER!";
            cout<<"---------- Motion Controller Started --------------"<<endl<<flush;
            serialHandler[0]->readBuffer.igotFirstMessage=true;
            serialHandler[0]->writeBuffer.bReadyToWrite=true;
            //goto KEEPS_JOB_NO_EXIT;
            //            action_controlPower(true);// Turn on the power automatically after received "FROM ZOEROBOTICS CONTROLLER!"
            action_initRobot();
            //delay(1000);
            //            action_controlZBrake(true);
            serialHandler[0]->readBuffer.readDone();
            bSerialProcessDone=true;

            return;
        }
    }

//    if(serialHandler[0]->writeBuffer.bReadyToWrite) return;

    ///////////////////////////////////////////////////////////////////////////
    // Parsing all responses from Motion Controller...
    // ... if R comes first it is a response otherwise it is garbage ...
    if(serialHandler[0]->readBuffer.codeSeen('R')==1)
    {


        responseCode=(int)serialHandler[0]->readBuffer.codeValue();

        int jobID = 0, seqNumber=0;
        if(serialHandler[0]->readBuffer.codeSeen('J'))
            jobID = (int)serialHandler[0]->readBuffer.codeValue();
        if(serialHandler[0]->readBuffer.codeSeen('N'))
            seqNumber = (int)serialHandler[0]->readBuffer.codeValue();
        switch (responseCode) {
        // ------------------------------------------------- case RC_ACK
        case RC_ACK:
            {
                packetAckRec.unpack(serialHandler[0]->readBuffer.getCmd());
                packetBase = &packetAckRec;
                if( QLibraryInfo::isDebugBuild()) packetAckRec.print();
                commanedCode = packetBase->cmdCode;
                /////////////////////////////////////////
                // Handling Errors...
                if(packetAckRec.errorCode>=100) {
                    action_cancelAllJobs(commanedCode, packetAckRec.errorCode, jobID, seqNumber);
                    return;
                }
                if(packetAckRec.cmdCode==SC_STOP) {

                }
                else if(packetAckRec.cmdCode==SC_SET_SPEED) {
                    // If command is set_speed,
                    // then the job process in socket will wait until all joints are sent...
                    bool rev = statusAllSpeedReady.check(packetAckRec.axisID);
//                    cout<<"Set Speed"<<statusAllSpeedReady.bAxis[0]<<statusAllSpeedReady.bAxis[1]<<statusAllSpeedReady.bAxis[2]<<statusAllSpeedReady.bAxis[3]<<endl;
                    if(!rev) {
                        // holding job process not by sending this:[jobIDDone = -1]
                        // Doesn't effect job sequence...
                        goto KEEPS_JOB_NO_EXIT;
                    }

                    if(statusAllSpeedReady.done) cout<<"Set Speed is Done!!!"<<endl;
                }
                else if(packetAckRec.cmdCode==SC_MOVE) {
                    // holding job process not by sending this:[jobIDDone = -1]
                    // Doesn't effect job sequence...
                    goto KEEPS_JOB_NO_EXIT;
                }
                break;
            }
        case RC_STATUS_LINEAR:{
            {
                packetLinearRec.unpack(serialHandler[0]->readBuffer.getCmd());
                packetBase = &packetLinearRec;//
                commanedCode = packetBase->cmdCode;
                packetLinearRec.print();
                if(packetLinearRec.errorCode>=100) {
                    action_cancelAllJobs(commanedCode, packetLinearRec.errorCode, jobID, seqNumber);
                    return;
                }
                //bool rev = statusAllSpeedReady.check(SM_KIN);
                statusAllSpeedReady.done=true;
                cout<<"Set LinearMotion Profile is Done!!!: ";
                break;
            }
        }
        case RC_STATUS_EEROTATION:{
            {
                packetEERotateRec.unpack(serialHandler[0]->readBuffer.getCmd());
                packetBase = &packetEERotateRec;//
                commanedCode = packetBase->cmdCode;
                packetEERotateRec.print();
                if(packetEERotateRec.errorCode>=100) {
                    action_cancelAllJobs(commanedCode, packetEERotateRec.errorCode, jobID, seqNumber);
                    return;
                }
                statusAllSpeedReady.done=true;
                cout<<"Set EE Rotation Profile is Done!!!: ";
                break;
            }
        }
        case RC_STATUS_CIRCLE:{
            {
                packetCircleRec.unpack(serialHandler[0]->readBuffer.getCmd());
                packetBase = &packetCircleRec;//
                commanedCode = packetBase->cmdCode;
                packetCircleRec.print();
                if(packetCircleRec.errorCode>=100) {
                    action_cancelAllJobs(commanedCode, packetCircleRec.errorCode, jobID, seqNumber);
                    return;
                }
                //bool rev = statusAllSpeedReady.check(SM_KIN);
                statusAllSpeedReady.done=true;
                cout<<"Set RC_STATUS_CIRCLE Profile is Done!!!: ";
                break;
            }
        }
        case RC_STATUS_SPRIAL:{
            {
                packetSpiralRec.unpack(serialHandler[0]->readBuffer.getCmd());
                packetBase = &packetSpiralRec;//
                commanedCode = packetBase->cmdCode;
                packetSpiralRec.print();
                if(packetSpiralRec.errorCode>=100) {
                    action_cancelAllJobs(commanedCode, packetSpiralRec.errorCode, jobID, seqNumber);
                    return;
                }
                statusAllSpeedReady.done=true;
                // bool rev = statusAllSpeedReady.check(SM_KIN);
                cout<<"Set RC_STATUS_SPRIAL Profile is Done!!!: ";
                break;
            }
        }
            // ------------------------------------------------- case RC_STATUS_MOTION
        case RC_UPDATE_MOTION:
            // Report current positions to the Client only when robot has motion...
            // Doesn't effect job sequence...
            {
                packetAllPosRec.unpack(serialHandler[0]->readBuffer.getCmd());
                for(int i=0; i<NUM_AXIS; i++) {
                    robotProperty[i].absSteps = packetAllPosRec.absSteps[i];
                }
                packetBase = &packetAllPosRec;//
                commanedCode = packetBase->cmdCode;
                response2SocketupdateAxisPosition(RC_UPDATE_MOTION, packetAllPosRec.cmdCode);
                goto KEEPS_JOB_NO_EXIT;
                break;
            }

            // ------------------------------------------------- case RC_STATUS
        case RC_STATUS:
            {
                packetStatusRec.unpack(serialHandler[0]->readBuffer.getCmd());
                // Update position
                robotProperty[packetStatusRec.axisID].absSteps = packetStatusRec.absSteps;

                packetBase = &packetStatusRec;//
                commanedCode = packetBase->cmdCode;
                packetStatusRec.print();

                if(packetStatusRec.cmdCode==SC_MOVE) {
                    // If command is SC_MOVE,
                    // then the job process in socket will wait until all joints are moved...
                    bool rev = statusAllMotionDone.check(packetStatusRec.axisID);
                    if(!rev) { // holding job process not by sending this:[jobIDDone = -1]
                        goto KEEPS_JOB_NO_EXIT;
                    }
                    serialSendTimer.stop();
                     bRobotMoving=false;
                    if(rev) cout<<"All Motion are Done!!!"<<endl<<flush;
                    response2SocketupdateAxisPosition(RC_UPDATE_MOTION, packetStatusRec.cmdCode);
                }
                break;
            }
            // ----------------------------------------------- case RC_STATUS_POS
        case RC_STATUS_ALL_POS:
            {
                packetAllPosRec.unpack(serialHandler[0]->readBuffer.getCmd());
                for(int i=0; i<NUM_AXIS; i++) {
                    robotProperty[i].absSteps = packetAllPosRec.absSteps[i];
                }
                packetAllPosRec.print();
                packetBase = &packetAllPosRec;//
                commanedCode = packetBase->cmdCode;
                response2SocketupdateAxisPosition(RC_STATUS_ALL_POS, packetAllPosRec.cmdCode);
                break;
            }
        case RC_ENCODER_VALUE:
            {
                serialTimeOut.stop();
                packetEncoderRec.unpack(serialHandler[0]->readBuffer.getCmd());
                cout<<"M:"<<packetEncoderRec.axisID<<", EncoderValue:"<<packetEncoderRec.encoderValue<<endl;
                packetBase = &packetEncoderRec;//
                commanedCode = packetBase->cmdCode;
                packetEncoderRec.print();
                break;
            }
            // ----------------------------------------------- case RC_STATUS_CIRCLE
            //        case RC_STATUS_CIRCLE:
            //            // DO Analyze received packet...
            //            packetCircleRec.unpack(serialHandler[0]->readBuffer.getCmd());
            //            packetBase = &packetCircleRec;//
            //            commanedCode = packetBase->cmdCode;
            //            break;
            // -----------------------------------------------
         default:
            cout<<"\n response default: "<<serialHandler[0]->readBuffer.getCmd()<<"\n";
            break;
        }
    }
    else {
        qDebug()<<" ---------- Garbage: "<<tData<<"\n";
        serialHandler[0]->readBuffer.readDone();
        bSerialProcessDone=true;
        return;
    }

    ////////////////////////////////////////////////////////////
    // Read off the command from ring buffer(Don't forget it!!!)...
    qDebug()<<"Job Done: "<<commanedCode;
//    cout<<"-------------Job Done: "<<commanedCode<<endl;
    if(currentPacketJob.nSequence==0) response2SocketupdateAxisPosition(RC_ORDER_DONE,commanedCode);
    handlingJobResponse();
    serialHandler[0]->readBuffer.readDone();
    jobIDDone = -1;// JOB DONE AND GO TO NEXT JOB TASK...

    // ...When it has a response, let it to write the next command...
    serialHandler[0]->writeBuffer.bReadyToWrite=true;

    bSerialProcessDone=true;
    if(commanedCode<1) return;
    if(commanedCode!=SC_UPDATE_MOTION) ;//cout<<"SC_UPDATE_MOTION: currentSequence="<<packetBase->nSequence;
    //    if(packetBase->nSequence==0 && commanedCode!=SC_UPDATE_MOTION) {
    //        // CHECK if current job is done or not...
    //        // If so, send RC_STATUS_JOB_DONE to client...
    //        response2Socket_jobDone(commanedCode);
    //        qDebug()<<"response2Socket_jobDone";
    //        //serialSendTimer.start(50);// test
    //    }
    return;
    // JOB IS NOT DONE AND WAITING UNTIL ALL MOTOR FINISH THEIR WORKS...
KEEPS_JOB_NO_EXIT:
    qDebug()<<"KEEPS_JOB_NO_EXIT: currentSequence="<<currentPacketJob.nSequence<<", cmd="<<commanedCode;
    // serialHandler[0]->readBuffer.print();
    serialHandler[0]->readBuffer.readDone();
    bSerialProcessDone=true;// Keep Accepting Message in from MC...

    if(commanedCode!=SC_UPDATE_MOTION)
        ;//qDebug()<<"KEEPS_JOB_NO_EXIT-SC_UPDATE_MOTION: currentSequence="<<currentPacketJob.nSequence;
    return;
}
void mkServer_Serial2Arm3::handlingJobResponse()
{
    if(packetBase==nullptr) return;
    // ----------------------- Init Robot -------------------
    if(packetBase->jobID == 0) {
        if(packetBase->nSequence==encoderJobSequence[0]) {
            // SC_GET_ENCODER for JR1
            // Home Position of RJ1 starts from Link1 touching the arm base in CW direction...
            // packetEncoderRec.encoderValue[deg]
            // Convert packetEncoderRec.encoderValue to steps...
            double stepJR1 =robotProperty[1].setNextStepsFromDistance(packetEncoderRec.encoderValue);
            robotProperty[1].absSteps = floor(stepJR1+0.5);
            cout<<"robotProperty[1].absSteps: "<<robotProperty[1].absSteps<<endl;

        }
        else if(packetBase->nSequence==encoderJobSequence[1]) { // SC_GET_ENCODER for JR2
            // Home Position of RJ1 starts from Link1 touching the arm base in CW direction...
            // packetEncoderRec.encoderValue[deg]
            // Convert packetEncoderRec.encoderValue to steps...
            double stepJR2 =robotProperty[2].setNextStepsFromDistance(packetEncoderRec.encoderValue);
            robotProperty[2].absSteps = floor(stepJR2+0.5);
            cout<<"robotProperty[2].absSteps: "<<robotProperty[2].absSteps<<endl;

        }
    }
    // ... check if this is the last sequence ...
    else {
        if(packetBase->nSequence==0)
        {
            // Send axis position to client through TCP/IP...
//                if(tcpServerForOrder->isConnected && tcpServerForOrder->socket->isOpen())
//                    tcpServerForOrder->socket->write();
            qDebug()<<"Job["<<packetBase->jobID<<"] ended.\n";
        }
    }
    //    qDebug()<<"Job["<<packetBase->jobID<<"] seq="<<packetBase->nSequence<<"\n";
}


void mkServer_Serial2Arm3::processSerialPortSendTimer()
{
    // cout<<"processSerialPortSendTimer--"<<endl<<flush;
    if(!tcpServerForOrder->isConnected) serialSendTimer.stop();// When the client is disconnected, kill timer(not sending)...
    if(bRobotMoving)
        action_requestMotionPosition(&currentPacketJob);
}

void mkServer_Serial2Arm3::processSerialPortInitTimer()
{

}

void mkServer_Serial2Arm3::processSerialPortTimeOut()
{
    serialTimeOut.stop();
    action_cancelAllJobs(111, ERROR_SERIAL_TIMEOUT,0,0);
}



void mkServer_Serial2Arm3::serialRequestStatus(int ch)
{
    char packet[68];
    sprintf(packet, "G%d M%d", SC_STATUS, ch);
    qDebug()<<"sending(serialRequestStatus)="<<packet<<endl<<flush;
    serialHandler[0]->write_crc(packet, strlen(packet));
//    QByteArray data = QByteArray((char*)packet, strlen(packet));
//    serialHandler[0]->write(data);
}

void mkServer_Serial2Arm3::serialRequestAllPosition()
{
    char packet[68];
    sprintf(packet, "G%d", SC_STATUS_ALL_POS);
    qDebug()<<"sending(requestAllPosition)="<<packet<<endl<<flush;
    serialHandler[0]->write_crc(packet, strlen(packet));
//    QByteArray data = QByteArray((char*)packet, strlen(packet));
//    serialHandler[0]->write(data);
}

void mkServer_Serial2Arm3::startHomingProcess(RobotProperty &robotStatus)
{
    robotStatus.print();
    if(robotStatus.cmdCode==SC_STATUS && robotStatus.opMode==HOMING)
    {
        qDebug()<< "Already Homing mode is operating, please wait until it is finished.";
        return;
    }
    char packet[68];
    sprintf(packet, "G%d M%d",SC_HOMING,selectedMotorID);
    serialHandler[0]->write_crc(packet, strlen(packet));
//    QByteArray data = QByteArray((char*)packet, strlen(packet));
//    serialHandler[0]->write(data);
    qDebug()<<"Sending (startHomingProcess) CMD:"<<packet;
}

void mkServer_Serial2Arm3::removeJobFromQueque()
{
    jobIDDone = -1;// allow to processs the next command...
}

void mkServer_Serial2Arm3::startMove(int select_motors)
{
    issuedXCMD  = SC_MOVE;
    char packet[68];
    sprintf(packet, "G%d A%d",SC_MOVE, select_motors);
//    QByteArray data = QByteArray((char*)packet, strlen(packet));
//    serialHandler[0]->write(data);
     serialHandler[0]->write_crc(packet, strlen(packet));

    qDebug()<<"Sending (startMove) CMD:"<<packet;
}


void mkServer_Serial2Arm3::loadConfigFile(QString name, RobotProperty *robotStatus)
{
    QFile file(CONFIG_FILE+name);

    if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        cout<<"\n\033[1;32m ----------------- Loading File: \033[0m["<<CONFIG_FILE+name.toStdString()<<"\033[1;32m -----------------\033[0m\n"<<flush;
        char name[128];
        while (!file.atEnd()) {
            QByteArray line = file.readLine();

            int step=0;
            int motorID=0;
            sscanf(line.toStdString().c_str(),"%s %d %d", name, &motorID, &step );
            //            qDebug()<<"read pos:"<<line<<name;
            if(std::string(name)=="abspos:")
            {
                robotStatus[motorID].ID =motorID;
                robotStatus[motorID].absSteps = step;
                double distMM = robotStatus[motorID].getDistanceFromStep();//[mm or deg]//.absSteps*X_STEP2DIST;
                cout<<"\033[1;32mReading Robot Position\033[0m["<<robotStatus[motorID].ID<<"].absPos: "<<robotStatus[motorID].absSteps<<"[steps],"<<distMM<<robotStatus[motorID].getUnit().c_str()<<"\n"<<flush;
            }
        }
    }
}

void mkServer_Serial2Arm3::savePos(QString name, RobotProperty *robotStatus)
{
    cout <<CONFIG_FILE<<name.toStdString()<<endl<<flush;
    QFile file(CONFIG_FILE+name);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        cout<<"Couldn't save it..."<<endl<<flush;
        return;
    }
    else {
        cout<<"Opened to save file..."<<endl<<flush;
    }

    QTextStream out(&file);
    for(int i=0; i<NUM_AXIS; i++)
        out << "abspos: "<< robotStatus[i].ID <<" "<< robotStatus[i].absSteps << "\n";
}

void mkServer_Serial2Arm3::delay(unsigned long ms, bool bJobDone)
{
    //    socketExecuteTimer.stop();
    QTime dieTime= QTime::currentTime().addMSecs(ms);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    //socketExecuteTimer.start(1);
    bDelayOp=false;
    if(bJobDone) jobIDDone=-1;
    qDebug()<<"-------delay is done......."<<ms<<endl;

}



void mkServer_Serial2Arm3::action_initRobot(PacketJobs *job)
{
    const int nJ=11;
    int s=0;
    int numberCurrentJob=0;
    PacketJobs *newJob[nJ]={nullptr};

    for(int i=0; i<nJ; i++) {
        newJob[i] = new PacketJobs;
    }

    //    newJob[s]->packf("J%d N%d G%d M%d\n",numberCurrentJob, s+1, SC_TIME_DELAY, 1000);  newJob[s]->unpack();s++;
    newJob[s]->packf("J%d N%d G%d M%d\n",numberCurrentJob, s+1, SC_POWER, 1);          newJob[s]->unpack();s++;
    newJob[s]->packf("J%d N%d G%d M%d\n",numberCurrentJob, s+1, SC_REBOOT, 1);         newJob[s]->unpack(); s++;
    // First set posions info from saved file...
    newJob[s]->packf("J%d N%d G%d M%d\n",numberCurrentJob, s+1, SC_SETPOS, 1);         newJob[s]->unpack();s++;
    newJob[s]->packf("J%d N%d G%d M%d\n",numberCurrentJob, s+1, SC_GET_ENCODER, 1);    newJob[s]->unpack();s++; encoderJobSequence[0]=s;
    newJob[s]->packf("J%d N%d G%d M%d\n",numberCurrentJob, s+1, SC_GET_ENCODER, 2);    newJob[s]->unpack();s++; encoderJobSequence[1]=s;
    // After read encoder for rotary joint, update all joint info on the Micro Controller...

    newJob[s]->packf("J%d N%d G%d M%d\n",numberCurrentJob, s+1, SC_SETPOS, 1);         newJob[s]->unpack();s++;

    newJob[s]->packf("J%d N%d G%d M%d\n",numberCurrentJob, 0, SC_STATUS_ALL_POS, 1); newJob[s]->unpack();s++;
    for(int i=0; i<s; i++) {
        queuePacketJobs.push(newJob[i]);
    }

}


void mkServer_Serial2Arm3::action_requestStatus(PacketJobs *job)
{
    char packet[68];
    sprintf(packet, "G%dM%dJ%dN%d", SC_STATUS, job->mode, job->jobID, job->nSequence);
    qDebug()<<"Sending(requestStatus)="<<packet<<Qt::endl<<Qt::flush;
    serialHandler[0]->write_crc(packet, strlen(packet));
//    QByteArray data = QByteArray((char*)packet, strlen(packet));
//    serialHandler[0]->write(data);
}

void mkServer_Serial2Arm3::action_requestAllPosition(PacketJobs *job)
{
    if(bRobotMoving) {
        qDebug()<<"action_requestAllPosition is Canceled due to the robot movement.\n";
        return;
    }
    char packet[68];
    sprintf(packet, "G%dJ%dN%d", SC_STATUS_ALL_POS, job->jobID, job->nSequence);
    //cout<<"requestAllPosition::"<<packet<<endl;
    qDebug()<<"Sending(requestAllPosition)="<<packet;//<<endl<<flush;//
    serialHandler[0]->write_crc(packet, strlen(packet));
//    QByteArray data = QByteArray((char*)packet, strlen(packet));
//    serialHandler[0]->write(data);
}

void mkServer_Serial2Arm3::action_requestEncoderValue(PacketJobs *job)
{
    char packet[68];
    sprintf(packet, "G%dM%dJ%dN%d", SC_GET_ENCODER, job->mode, job->jobID, job->nSequence);
    //cout<<"requestAllPosition::"<<packet<<endl;
    qDebug()<<"Sending(action_requestEncoderValue)="<<packet;//<<endl<<flush;//
    serialHandler[0]->write_crc(packet, strlen(packet));
//    QByteArray data = QByteArray((char*)packet, strlen(packet));
//    serialHandler[0]->write(data);
    serialTimeOut.start(1000);// uncomment it !!!
}
void mkServer_Serial2Arm3::action_requestMotionPosition(PacketJobs *job)
{
    char packet[68];
    sprintf(packet, "G%dJ%dN%d", SC_UPDATE_MOTION,  job->jobID, job->nSequence);
    serialHandler[0]->write_crc(packet, strlen(packet));
    //cout<<"requestAllPosition::"<<packet<<endl;
     qDebug()<<"Sending(requestMotionPosition)="<<packet;//<<endl<<flush;//
//    QByteArray data = QByteArray((char*)packet, strlen(packet));
////    qDebug()<<"Sending(requestMotionPosition)="<<packet<<", "<<data<<Qt::endl<<Qt::flush;//
//    serialHandler[0]->write(data);
}

void mkServer_Serial2Arm3::action_reboot(PacketJobs *job)
{
    issuedXCMD= SC_REBOOT;
    char packet[68];
    sprintf(packet, "G%dJ%dN%d", SC_REBOOT, job->jobID, job->nSequence);
//    QByteArray data = QByteArray(packet, strlen(packet));
    serialHandler[0]->write_crc(packet, strlen(packet));
    qDebug()<<"Sending(action_reboot)="<<packet;//<<endl<<flush;//
//    serialHandler[0]->write(data);
}
void mkServer_Serial2Arm3::action_delayMicroController(PacketJobs *job)
{
    issuedXCMD= SC_TIME_DELAY_MC;
    char packet[68];
    sprintf(packet, "G%dM%dJ%dN%d", SC_TIME_DELAY_MC, job->mode, job->jobID, job->nSequence);
//    QByteArray data = QByteArray(packet, strlen(packet));
    qDebug()<<"Sending(action_delayMicroController)="<<packet;//<<endl<<flush;//
    serialHandler[0]->write_crc(packet, strlen(packet));
//    serialHandler[0]->write(data);
}
void mkServer_Serial2Arm3::action_controlPower(PacketJobs *job)
{
    issuedXCMD= SC_POWER;
    char packet[68];
    sprintf(packet, "G%dM%dJ%dN%d", SC_POWER, job->mode, job->jobID, job->nSequence);
//    QByteArray data = QByteArray(packet, strlen(packet));
    qDebug()<<"Sending(action_controlPower)="<<packet;//<<endl<<flush;//
    serialHandler[0]->write_crc(packet, strlen(packet));
//    serialHandler[0]->write(data);
}

void mkServer_Serial2Arm3::action_controlPower(bool bPowerOn)
{
    issuedXCMD= SC_POWER;
    char packet[68];
    sprintf(packet, "G%dM%dJ%dN%d", SC_POWER, bPowerOn, 1, 0);
//    QByteArray data = QByteArray(packet, strlen(packet));
    qDebug()<<"Sending(action_controlPower)="<<packet;//<<endl<<flush;//
//    serialHandler[0]->write(data);
    serialHandler[0]->write_crc(packet, strlen(packet));
}

void mkServer_Serial2Arm3::action_controlZBrake(PacketJobs *job)
{
    issuedXCMD= SC_Z_BRAKE;
    char packet[68];
    sprintf(packet, "G%dM%dJ%dN%d", SC_Z_BRAKE, job->mode, job->jobID, job->nSequence);
//    QByteArray data = QByteArray(packet, strlen(packet));
    qDebug()<<"Sending(action_controlZBrake)="<<packet;//<<endl<<flush;//
//    serialHandler[0]->write(data);
    serialHandler[0]->write_crc(packet, strlen(packet));
}

void mkServer_Serial2Arm3::action_controlZBrake(bool bPowerOn)
{
    issuedXCMD= SC_Z_BRAKE;
    char packet[68];
    sprintf(packet, "G%dM%dJ%dN%d", SC_Z_BRAKE, bPowerOn, 1, 0);
//    QByteArray data = QByteArray(packet, strlen(packet));
    qDebug()<<"Sending(action_controlZBrake)="<<packet;//<<endl<<flush;//
//    serialHandler[0]->write(data);
    serialHandler[0]->write_crc(packet, strlen(packet));
}

void mkServer_Serial2Arm3::action_homing(PacketJobs *job)
{
    issuedXCMD  = SC_HOMING;
    PacketSpeed targetVel;
    PacketSpeedProfile outVelProfile;
    targetVel.jobID = job->jobID;
    targetVel.nSequence = job->nSequence;
    targetVel.axisID = job->mode;
    targetVel.diffPos = job->data[job->mode];
    targetVel.vel = job->data[4];
    targetVel.accel= job->data[5];
    robotKin.genSpeedProfile(robotProperty[targetVel.axisID],targetVel,outVelProfile,issuedXCMD );
//    QByteArray data = QByteArray((char*)outVelProfile.get(), outVelProfile.length());
    qDebug()<<"Sending data (action_homing):"<<outVelProfile.get();
//    serialHandler[0]->write(data);
    serialHandler[0]->write_crc(outVelProfile.get(), outVelProfile.length());
}
////////////////////////////////////////////////////////////////
bool mkServer_Serial2Arm3::action_genSeedProfile(PacketJobs *job)
{

    //    qDebug()<<"------------------------------- action_genSeedProfile ------------------\n";
    //    job->print();
    //    qDebug()<<"-------------------------------------------------\n";
    if(job->mode>SINGLE_JNT_MODE){
        int ret  = robotKin.checkJointLimit(job->data[0], job->data[1], job->data[2], job->data[3]);
        if(ret>0) {
            cout<<"---- Joint Limit Error ---"<<endl;
            action_cancelAllJobs(job->cmdCode, ERROR_JOINT_LIMIT+ret, job->jobID, job->nSequence);
            return false;
        }
    }
    //////////////////////////////////////////
    // Handling 4-joints input...
    if(job->mode==MULTI_ALL_JNT_MODE) {
        double targetPos[6];
        memcpy(targetPos, job->data, sizeof(double)*6);

        targetPos[1]+= robotProperty[1].centerPos;// [deg]
        targetPos[2]+= robotProperty[2].centerPos;// [deg]

        statusAllSpeedReady.reset();// check if all speed setups done in ACK response for trigger SC_MOVE...
        statusAllMotionDone.reset();// check if all movement is done in PacketStatusRec for next job...
        issuedXCMD = SC_SET_SPEED;
//        bool anyMotion=false;
        for(int i=0; i<4; i++){
            statusAllSpeedReady.add(i);
            statusAllMotionDone.add(i);
        }
        for(int i=0; i<4; i++) {
            // if(i!=1) continue;//for a test
            PacketSpeed targetVel;
            PacketSpeedProfile outVelProfile;
            double nextStep =robotProperty[i].setNextStepsFromDistance(targetPos[i]);
            double diffPos =  (nextStep - robotProperty[i].absSteps)*robotProperty[i].STEP2DIST;
            if(robotProperty[i].jointType==JT_ROT){
                diffPos*=RAD2DEG;
            }
            //            if(abs(diffPos)<=0.001) continue;
//            anyMotion=true;
            ///////////////////////////////////////
            targetVel.jobID = job->jobID;
            targetVel.nSequence = job->nSequence;
            targetVel.axisID = i;
            targetVel.diffPos = diffPos;
            targetVel.vel = robotProperty[i].getMaxVel()*targetPos[4]*0.01;// [%]
            targetVel.accel= robotProperty[i].getMaxAccel()*targetPos[5]*0.01; // [%]
            targetVel.pack();
            targetVel.print();
            robotKin.genSpeedProfile(robotProperty[i],targetVel,outVelProfile);


//            QByteArray data = QByteArray((char*)outVelProfile.get(), outVelProfile.length());
            qDebug()<<"Sending (action_setMultiJointSpeed):"<<outVelProfile.get();
            serialHandler[0]->write_crc(outVelProfile.get(), outVelProfile.length());
//            serialHandler[0]->write(data);
//            statusAllSpeedReady.add(i);
//            statusAllMotionDone.add(i);
            delay(WRITE_DELAY, false);
        }
    }//-------- if(job->mode==MULTI_JNT_MODE) -----------------
    // Handling X-Z jointS at the same time...
    if(job->mode==MULTI_XZ_JNT_MODE) {
        double targetPos[6];
        int jntXZIndex[2]={0,3};
        memcpy(targetPos, job->data, sizeof(double)*6);

        targetPos[1]+= robotProperty[1].centerPos;// [deg]
        targetPos[2]+= robotProperty[2].centerPos;// [deg]

        statusAllSpeedReady.reset();// check if all speed setups done in ACK response for trigger SC_MOVE...
        statusAllMotionDone.reset();// check if all movement is done in PacketStatusRec for next job...
        issuedXCMD = SC_SET_SPEED;
        for(int i=0; i<2; i++){
            statusAllSpeedReady.add(i);
            statusAllMotionDone.add(i);
        }
        bool anyMotion=false;
        int i=0;
        for(int j=0; j<2; j++) {
            i = jntXZIndex[j];
            PacketSpeed targetVel;
            PacketSpeedProfile outVelProfile;
            double nextStep =robotProperty[i].setNextStepsFromDistance(targetPos[i]);
            double diffPos =  (nextStep - robotProperty[i].absSteps)*robotProperty[i].STEP2DIST;
            if(robotProperty[i].jointType==JT_ROT){
                diffPos*=RAD2DEG;
            }
            //            if(abs(diffPos)<=0.001) continue;
            anyMotion=true;
            ///////////////////////////////////////
            targetVel.jobID = job->jobID;
            targetVel.nSequence = job->nSequence;
            targetVel.axisID = i;
            targetVel.diffPos = diffPos;
            targetVel.vel = robotProperty[i].getMaxVel()*targetPos[4]*0.01;// Percentage of speed => 1/100
            targetVel.accel= robotProperty[i].getMaxAccel()*targetPos[5]*0.01;// Percentage of acceleration => 1/100
            targetVel.pack();
            targetVel.print();
            robotKin.genSpeedProfile(robotProperty[i],targetVel,outVelProfile);

            statusAllSpeedReady.add(i);
            statusAllMotionDone.add(i);

//            QByteArray data = QByteArray((char*)outVelProfile.get(), outVelProfile.length());
            qDebug()<<"Sending (action_setMultiJointSpeed):"<<outVelProfile.get();
//            serialHandler[0]->write(data);
            serialHandler[0]->write_crc(outVelProfile.get(), outVelProfile.length());
//            delay(WRITE_DELAY, false);
//
        }
    }
    // Handling ARM joints (JR1, JR2) at the same time...
    if(job->mode==MULTI_ARM_JNT_MODE) {
        double targetPos[6];
        memcpy(targetPos, job->data, sizeof(double)*6);

        targetPos[1]+= robotProperty[1].centerPos;// [deg]
        targetPos[2]+= robotProperty[2].centerPos;// [deg]

        statusAllSpeedReady.reset();// check if all speed setups done in ACK response for trigger SC_MOVE...
        statusAllMotionDone.reset();// check if all movement is done in PacketStatusRec for next job...
        issuedXCMD = SC_SET_SPEED;
        for(int i=0; i<3; i++){
            statusAllSpeedReady.add(i);
            statusAllMotionDone.add(i);
        }
        bool anyMotion=false;
        for(int i=1; i<3; i++) {
            // if(i!=1) continue;//for a test
            PacketSpeed targetVel;
            PacketSpeedProfile outVelProfile;
            double nextStep =robotProperty[i].setNextStepsFromDistance(targetPos[i]);
            double diffPos =  (nextStep - robotProperty[i].absSteps)*robotProperty[i].STEP2DIST;
            if(robotProperty[i].jointType==JT_ROT){
                diffPos*=RAD2DEG;
            }
            //            if(abs(diffPos)<=0.001) continue;
            anyMotion=true;
            ///////////////////////////////////////
            targetVel.jobID = job->jobID;
            targetVel.nSequence = job->nSequence;
            targetVel.axisID = i;
            targetVel.diffPos = diffPos;
            targetVel.vel = robotProperty[i].getMaxVel()*targetPos[4]*0.01;// Percentage of speed => 1/100
            targetVel.accel= robotProperty[i].getMaxAccel()*targetPos[5]*0.01;// Percentage of acceleration => 1/100
            targetVel.pack();
            targetVel.print();
            robotKin.genSpeedProfile(robotProperty[i],targetVel,outVelProfile);

//            statusAllSpeedReady.add(i);
//            statusAllMotionDone.add(i);
//            QByteArray data = QByteArray((char*)outVelProfile.get(), outVelProfile.length());
            qDebug()<<"Sending (action_setMultiJointSpeed):"<<outVelProfile.get();
//          //serialHandler[0]->write(data);
            serialHandler[0]->write_crc(outVelProfile.get(), outVelProfile.length());
//             delay(WRITE_DELAY, false);
        }
    }
    // Handingl Single Joint Operation...
    else if(job->mode<SINGLE_JNT_MODE ) {
        double targetPos=0;
        int axisID = job->mode;


        if(robotProperty[axisID].getJointType()==JT_ROT) {
            targetPos=job->data[axisID] + robotProperty[axisID].centerPos;
        }
        else {
            targetPos=job->data[axisID];
        }
        statusAllSpeedReady.reset();// check if all speed setups done in ACK response for trigger SC_MOVE...
        statusAllMotionDone.reset();// check if all movement is done in PacketStatusRec for next job...
        issuedXCMD = SC_SET_SPEED;
        PacketSpeed targetVel;
        PacketSpeedProfile outVelProfile;
        double nextStep =robotProperty[axisID].setNextStepsFromDistance(targetPos);
        double diffPos =  (nextStep - robotProperty[axisID].absSteps)*robotProperty[axisID].STEP2DIST;
        if(robotProperty[axisID].getJointType()==JT_ROT){
            diffPos*=RAD2DEG;
        }
        //        if(abs(diffPos)<=0.001) {
        //             removeJobFromQueque();
        //             return;
        //        }
        ///////////////////////////////////////
        targetVel.jobID = job->jobID;
        targetVel.nSequence = job->nSequence;
        targetVel.axisID = axisID;
        targetVel.diffPos = diffPos;
        targetVel.vel = robotProperty[axisID].getMaxVel()*job->data[4]*0.01;
        targetVel.accel= robotProperty[axisID].getMaxAccel()*job->data[5]*0.01;
        targetVel.pack();
        targetVel.print();
        robotKin.genSpeedProfile(robotProperty[axisID],targetVel,outVelProfile);

        statusAllSpeedReady.add(axisID);
        statusAllMotionDone.add(axisID);
//        QByteArray data = QByteArray((char*)outVelProfile.get(), outVelProfile.length());
        qDebug()<<"Sending (action_setSingleJointSpeed):"<<outVelProfile.get();
        serialHandler[0]->write_crc(outVelProfile.get(), outVelProfile.length());
//        serialHandler[0]->write(data);
    }
    return true;
}

void mkServer_Serial2Arm3::action_genLinearMotion(PacketJobs *job)
{
    // Job is already unpacked...
    qDebug()<<"------------------------------- action_genLinearMotion ------------------\n";
    statusAllSpeedReady.reset();// check if all speed setups done in ACK response for trigger SC_MOVE...
    statusAllMotionDone.reset();// check if all movement is done in PacketStatusRec for next job...

    EEMovePacket linearProfile;
    linearProfile.XeeStart = job->data[0];
    linearProfile.XeeEnd = job->data[1];
    linearProfile.YeeStart = job->data[2];
    linearProfile.YeeEnd = job->data[3];
    linearProfile.ZeeStart = job->data[4];
    linearProfile.ZeeEnd = job->data[5];
    linearProfile.THee = job->data[6];
    linearProfile.Vel = job->data[7];
    linearProfile.jobID = job->jobID;
    linearProfile.nSequence = job->nSequence;
    linearProfile.pack();// make a string command ...

    // Response One time
    statusAllSpeedReady.add(SM_KIN);
//    statusAllSpeedReady.add(SM_Z);

    // Report 3 times for each motor...
    // it will wait until all motions are completed otherwise it will wait forever so
    // it needs to be fixed with time out...
    statusAllMotionDone.add(SM_X);
    statusAllMotionDone.add(SM_R1);
    statusAllMotionDone.add(SM_R2);
    if(abs(linearProfile.ZeeEnd-linearProfile.ZeeStart)>=0.1) {
        statusAllSpeedReady.add(SM_Z);
        statusAllMotionDone.add(SM_Z); // Consider Z motion too...
    }
//    QByteArray data = QByteArray((char*)linearProfile.get(), linearProfile.length());
//    serialHandler[0]->write(data);
    serialHandler[0]->write_crc(linearProfile.get(), linearProfile.length());
    qDebug()<<"Sending (action_genLinearMotion):"<<linearProfile.get()<<", selected Mode:"<<statusAllSpeedReady.selectMotors;
//     delay(WRITE_DELAY, false);
}
//
void mkServer_Serial2Arm3::action_genRotateEEMotion(PacketJobs *job)
{
    qDebug()<<"------------------------------- action_genRotateEEMotion ------------------\n";
    statusAllSpeedReady.reset();// check if all speed setups done in ACK response for trigger SC_MOVE...
    statusAllMotionDone.reset();// check if all movement is done in PacketStatusRec for next job...

    EERotatePacket eeRotateProfile;
    eeRotateProfile.XeeStart = job->data[0];
    eeRotateProfile.YeeStart = job->data[1];
    eeRotateProfile.THeeStart = job->data[2];
    eeRotateProfile.THeeEnd = job->data[3];
    eeRotateProfile.Vel = job->data[4];
    eeRotateProfile.jobID = job->jobID;
    eeRotateProfile.nSequence = job->nSequence;
    eeRotateProfile.pack();// make a string command ...

    // Response One time
    statusAllSpeedReady.add(SM_KIN);

    // Report 3 times for each motor...
    // it will wait until all motions are completed otherwise it will wait forever so
    // it needs to be fixed with time out...
    statusAllMotionDone.add(0);
    statusAllMotionDone.add(1);
    statusAllMotionDone.add(2);
//    QByteArray data = QByteArray((char*)eeRotateProfile.get(), eeRotateProfile.length());
//    serialHandler[0]->write(data);
    serialHandler[0]->write_crc(eeRotateProfile.get(), eeRotateProfile.length());
    qDebug()<<"Sending (action_genRotateEEMotion):"<<eeRotateProfile.get()<<", selected Mode:"<<statusAllSpeedReady.selectMotors;

}
void mkServer_Serial2Arm3::action_genCircularMotion(PacketJobs *job)
{
    // Job is already unpacked...
    qDebug()<<"------------------------------- action_circularMotion ------------------\n";
    qDebug()<<"Speed="<<job->data[0]<<", Radius="<<job->data[1];
    qDebug()<<"-------------------------------------------------\n";
    statusAllSpeedReady.reset();// check if all speed setups done in ACK response for trigger SC_MOVE...
    statusAllMotionDone.reset();// check if all movement is done in PacketStatusRec for next job...

    PacketCircle circleProfile;
    circleProfile.speed =  job->data[0];
    circleProfile.radius = job->data[1];
    circleProfile.cenPosX = job->data[2];
    circleProfile.cenPosY = job->data[3];
    circleProfile.EETheta = job->data[4];
    circleProfile.arcAng =  job->data[5];
    circleProfile.jobID = job->jobID;
    circleProfile.nSequence = job->nSequence;
    circleProfile.pack();

    // Response One time
    statusAllSpeedReady.add(SM_KIN);

    // Report 3 times for each motor...
    statusAllMotionDone.add(0);
    statusAllMotionDone.add(1);
    statusAllMotionDone.add(2);
//    QByteArray data = QByteArray((char*)circleProfile.get(), circleProfile.length());
//    serialHandler[0]->write(data);
    serialHandler[0]->write_crc(circleProfile.get(), circleProfile.length());
    qDebug()<<"Sending (action_circularMotion):"<<circleProfile.get()<<", selected Mode:"<<statusAllSpeedReady.selectMotors;
}

void mkServer_Serial2Arm3::action_genSpiralMotion(PacketJobs *job)
{
    // Job is already unpacked...
    qDebug()<<"------------------------------- action_spiralMotion ------------------\n";
    qDebug()<<"Speed="<<job->data[0]<<", Radius="<<job->data[1];
    qDebug()<<"-------------------------------------------------\n";
    statusAllSpeedReady.reset();// check if all speed setups done in ACK response for trigger SC_MOVE...
    statusAllMotionDone.reset();// check if all movement is done in PacketStatusRec for next job...

    PacketSpiral spiralProfile;
    spiralProfile.speed =  job->data[0];
    spiralProfile.radius = job->data[1];
    spiralProfile.cenPosX = job->data[2];
    spiralProfile.cenPosY = job->data[3];
    spiralProfile.EETheta = job->data[4];
    spiralProfile.arcAng =  job->data[5];
    spiralProfile.posZ =    job->data[6];
    spiralProfile.heightZ = job->data[7];
    spiralProfile.jobID = job->jobID;
    spiralProfile.nSequence = job->nSequence;
    spiralProfile.pack();

    // Response One time
    statusAllSpeedReady.add(SM_KIN);

    // Report 3 times for each motor...
    statusAllMotionDone.add(0);
    statusAllMotionDone.add(1);
    statusAllMotionDone.add(2);
    if(abs(spiralProfile.heightZ)>=0.1) {
        statusAllSpeedReady.add(SM_Z);
        statusAllMotionDone.add(3); // Consider Z motion too...
    }
//    QByteArray data = QByteArray((char*)spiralProfile.get(), spiralProfile.length());
//    serialHandler[0]->write(data);
    serialHandler[0]->write_crc(spiralProfile.get(), spiralProfile.length());
    qDebug()<<"Sending (action_genSpiralMotion):"<<spiralProfile.get()<<", selected Mode:"<<statusAllSpeedReady.selectMotors;
}

bool mkServer_Serial2Arm3::action_order(PacketJobs *job)
{
    //const int nJ=11;
    int sequence=0;

    int dropCupDelayTime=120;
    double cupPosOffset=-5;
    EE_LOCACTION EE_entryPickupCup = {331.71+cupPosOffset, 179.00, 500.0, 120.46 *DTOR, 150.0};
    double cupApproachZHeight=80;// Relative position...
    double cupPicupXZ[2]={218.44, EE_entryPickupCup.Z};
    EE_LOCACTION EE_targetPickupCup = {224.88+cupPosOffset, 363.36, 500.0, 120.46 *DTOR, 150.0};

    EE_LOCACTION EE_entryCoffeeMachine = { 700.00, 130.00, 371.00, 90.00 *DTOR, 150.0};
    EE_LOCACTION EE_targetCoffeeMachine ={700.00,  388.00, 371.00, 90.00 *DTOR, 150.0};

    EE_LOCACTION EE_entrySyrup1 = { 1000.00, 130.00, 520.00, 90.00 *DTOR, 150.0};
    EE_LOCACTION EE_targetSyrup1 ={ 1000.00,  300.00, 520.00, 90.00 *DTOR, 150.0};

    EE_LOCACTION EE_entrySyrup2 = { 1200.00, 130.00, 520.00, 90.00 *DTOR, 150.0};
    EE_LOCACTION EE_targetSyrup2 ={ 1200.00,  300.00, 520.00, 90.00 *DTOR, 150.0};

    EE_LOCACTION EE_entryPlaceCup1 = { 943.00, -120.00, 250.0, -90 *DTOR, 150.0};
    EE_LOCACTION EE_targetPlaceCup1 ={ 943.00,  -355.00, 250.00, -90*DTOR, 150.0};

    EE_LOCACTION EE_entryPlaceCup2 = { 1337.00-100*0, -176.00, 250.0, -129 *DTOR, 150.0};
    EE_LOCACTION EE_targetPlaceCup2 ={ 1193.00-100*0,  -355.00, 250.00, -129*DTOR, 150.0};

    double loweringZforPlaceCup = 84;// Absolute position...
    double foldArm[2]={-40.0, -140.0};// [Degree]

   // EE_LOCACTION EE_rotationCross ={ 1200.00,  130.00, 520.00, 0.00 *DTOR, 35*DTOR};
    double EE_rotationCrossTh = 0;
    double EE_rotationFrontTh = -90 *DTOR;
    double EE_rotationCrossSpeed = 32*DTOR;
    double EE_rotationFrontSpeed = 40*DTOR;

    double jointMotionSpeedPercent = 100.0;

    double circleRadius = 25.0, circleTurns=2.0, circleSpeed=360.0/2.5;
    double spiralRadius = 33.0, spiralTurns=3.0, spiralSpeed=360.0/1.6;

//    double linearSpeed = 150.0;
    MKZoeRobotKin robotKin;
    bool ret=0;

#if 0
    job->jobID=0;
    //////////////////////////////////////////////////////////////////////////
    // Joint Mode Test....
    for(int i=0; i<50; i++) {
        job->jobID++;
        //        motion_moveSingleJoint(job->jobID, sequence, SM_X, 1400, 100.0);
        //        motion_moveSingleJoint(job->jobID, sequence, SM_X, 300, 100.0);
        for(int n=0; n<14; n++) {
            motion_moveAllJoints(job->jobID, sequence, 100.0 + n*100, -120 + n*17,  140.0  - n*21, 100.0 + n*65, 100.0);
            motion_delay(job->jobID, sequence,400);
        }
    }
    jobIDDone=-1;// Start action...
    return true;
#endif

#if 0
    //////////////////////////////////////////
    // Linear Motion Test...
    double EEx = 635.0;
    double EEy = -138.0;
    double EEth = -M_PI;
    double EEz = 0;

    double EETargetX =  635.0;
    double EETargetY =  -355.00;
    double EETargetTh =  -90.00;
    for(int i=0; i<100; i++) {
        job->jobID++;

        //////////////////////////////////////////////////////////////////////////////////////////////////////
        // Move to the entrance of the syrup place #3...
        EEx = 1200.00;
        EEy = 130.00;
        EEth = 90 *DTOR;
        EEz = 520.00;
        ret = robotKin.invKinEE_W_XYRZ(EEx, EEy, EEth, EEz);
        if(!ret) {
            cout<< "action_order:: no solution"<<endl;
            goto GOTO_ORDER_CANCEL;
        }
        // motion_delay(job->jobID, sequence, 200);
        motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, 100.0);
        // Move to the syrup place #3 from the entrance through the linear motion...
        EETargetX =  EEx;
        EETargetY =  300.00;
        double sprialDownHeight = -50.0;
        if(!motion_EELinear(  job->jobID, sequence, EEx,EEy, EEth,  EETargetX, EETargetY,  linearSpeed)) goto GOTO_ORDER_CANCEL; // check if there is solution of IK for the target position...
        // motion_delay(job->jobID, sequence, 200);
        // Circular motion...Sending (action_genSpiralMotion): "G5 S-50.000 R33.000 X1200.000 Y300.000 T90.000 A1080.000  B360.000  C520.000 J26 N33\n" , selected Mode: 24
        if(!motion_spiral(job->jobID, sequence, EETargetX, EETargetY, 90.0 , spiralRadius, 360*spiralTurns,  EEz, sprialDownHeight ,360.0/1.6 )) goto GOTO_ORDER_CANCEL;
        // Extract cup to the entrance position through the linear motion...
        if(!motion_EELinear(  job->jobID, sequence, EETargetX, EETargetY, EEth, EEx,EEy,  linearSpeed)) goto GOTO_ORDER_CANCEL;
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        EETargetTh=0;
        if(!motion_EERotation(job->jobID, sequence, EEx, EEy, EEth, EETargetTh, 30*DTOR)) goto GOTO_ORDER_CANCEL;
        // Carrying cup arm configuration to move around...
        //        motion_rotateArmJoint(job->jobID, sequence, -40.0, -140.0, 100.0);// Only Arm configure...


        // Rotate EE by -90 degree to face to the front...
        // EEx = 1337.00;
        EEy = -176.00;
        EEz = 150.0;
        EEth = -90.00*DTOR;
        EETargetX =  1193.00;
        EETargetY =  -355.00;
        EETargetTh = -129*DTOR;

        ret = robotKin.invKinEE_W_XYRZ(EEx, EEy, EEth, EEz);
        if(!ret) {
            cout<< "action_order:: no solution"<<endl;
            goto GOTO_ORDER_CANCEL;
        }
        // Move the cup to the entrance of target...
        motion_delay(job->jobID, sequence, 400);
        motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, 100.0);

        if(!motion_EELinear(  job->jobID, sequence,  EEx,EEy, EEth, EETargetX, EETargetY,  linearSpeed)) goto GOTO_ORDER_CANCEL;
        if(!motion_EELinear(  job->jobID, sequence, EETargetX, EETargetY, EEth, EEx,EEy,  linearSpeed)) goto GOTO_ORDER_CANCEL;

        motion_delay(job->jobID, sequence, 400);
        motion_moveAllJoints(job->jobID, sequence, 100.0 , -120 ,  140.0  , 100.0 , 100.0);

    }


    jobIDDone=-1;// Start action...
    return true;
#endif
    /////////////////////////////////////////////////////////////////////////////////////
    if(job->mode==1) {
        job->jobID=0;
        for(int i=0; i<1; i++) {
            //cout<<"=================== Loop No:"<<i+1<<" ===================="<<endl;
            job->jobID++;
            sequence=0;
            // lift up in Z direction...
            motion_moveSingleJoint(job->jobID, sequence, SM_Z, cupPicupXZ[1], 100.0);



            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // Move to the entrance of cup drop place by the joint motion...

            ret = robotKin.invKinEE_W_XYRZ(EE_entryPickupCup.X, EE_entryPickupCup.Y, EE_entryPickupCup.Th, EE_entryPickupCup.Z);
            if(!ret) {
                cout<< "action_order:: no solution"<<endl;
                goto GOTO_ORDER_CANCEL;
            }

            motion_moveSingleJoint(job->jobID, sequence, SM_R1, robotKin.param.th1*RTOD, 100.0);

            motion_moveSingleJoint(job->jobID, sequence, SM_R2, robotKin.param.th2*RTOD, 100.0);
            motion_moveSingleJoint(job->jobID, sequence, SM_X, robotKin.param.Px, 100.0);

            // motion_delay(job->jobID, sequence, 200);
//            motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, jointMotionSpeedPercent);

            // Move to the cup drop place from the entrance through the linear motion...
            //        ret = robotKin.invKinEE_W_XYRZ(EETargetX, EETargetY, EEth, EEz); // check if there is solution of IK for the target position...
            //        if(!ret) {
            //            cout<< "action_order:: no solution"<<endl;
            //             goto GOTO_ORDER_CANCEL;
            //        }

            if(!motion_EELinear(job->jobID, sequence, EE_entryPickupCup.X, EE_entryPickupCup.Y, EE_entryPickupCup.Th,  EE_targetPickupCup.X, EE_targetPickupCup.Y,  EE_entryPickupCup.S)) goto GOTO_ORDER_CANCEL;
            motion_moveSingleJoint(job->jobID, sequence, SM_Z, EE_entryPickupCup.Z+cupApproachZHeight, 100.0);
            // Wait for droping cup into the cup holder...
            //motion_delay(job->jobID, sequence, 1500);

            // ... temporary commented, please uncomment it ...
//            motion_dropCup(job->jobID, sequence, dropCupDelayTime);

            motion_moveSingleJoint(job->jobID, sequence, SM_Z, EE_entryPickupCup.Z-cupApproachZHeight, 100.0);
            // Extract cup to the entrance position through the linear motion...
            if(!motion_EELinear(  job->jobID, sequence,  EE_targetPickupCup.X, EE_targetPickupCup.Y, EE_targetPickupCup.Th, EE_entryPickupCup.X, EE_entryPickupCup.Y,  EE_targetPickupCup.S)) goto GOTO_ORDER_CANCEL;

            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // Move to the entrance of the coffee place...

            ret = robotKin.invKinEE_W_XYRZ(EE_entryCoffeeMachine.X, EE_entryCoffeeMachine.Y, EE_entryCoffeeMachine.Th, EE_entryCoffeeMachine.Z);
            if(!ret) {
                cout<< "action_order:: no solution"<<endl;
                goto GOTO_ORDER_CANCEL;
            }
            //motion_delay(job->jobID, sequence, 200);
            motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, jointMotionSpeedPercent);
            //motion_delay(job->jobID, sequence, 200);
            // Move to the coffee place from the entrance through the linear motion...
            if(!motion_EELinear(  job->jobID, sequence, EE_entryCoffeeMachine.X, EE_entryCoffeeMachine.Y, EE_entryCoffeeMachine.Th,  EE_targetCoffeeMachine.X, EE_targetCoffeeMachine.Y,  EE_entryCoffeeMachine.S)) goto GOTO_ORDER_CANCEL; // check if there is solution of IK for the target position...

            // Wait for coffee extraction...
            motion_delay(job->jobID, sequence, 2500);
            // Extract cup to the entrance position through the linear motion...
            if(!motion_EELinear(  job->jobID, sequence, EE_targetCoffeeMachine.X, EE_targetCoffeeMachine.Y, EE_targetCoffeeMachine.Th,  EE_entryCoffeeMachine.X, EE_entryCoffeeMachine.Y,  EE_targetCoffeeMachine.S)) goto GOTO_ORDER_CANCEL;
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            // Move to the entrance of the syrup place #1...
            //        EEx = 1100.00;
            //        EEy = 130.00;
            //        EEth = 90 *DTOR;
            //        EEz = 520.00;
            //        ret = robotKin.invKinEE_W_XYRZ(EEx, EEy, EEth, EEz);
            //        if(!ret) {
            //            cout<< "action_order:: no solution"<<endl;
            //            goto GOTO_ORDER_CANCEL;
            //        }
            //        motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, 100.0);
            //        // Move to the syrup place #1 from the entrance through the linear motion...
            //        EETargetX =  EEx;
            //        EETargetY =  300.00;
            //        if(!motion_EELinear(  job->jobID, sequence, EEx,EEy, EEth,  EETargetX, EETargetY,  150.0)) goto GOTO_ORDER_CANCEL; // check if there is solution of IK for the target position...

            //        // Circular motion...
            //        motion_circle(job->jobID, sequence,EETargetX, EETargetY, 90.0 , circleRadius, 360*circleTurns,  360.0/2.5 );
            //        // Extract cup to the entrance position through the linear motion...
            //        if(!motion_EELinear(  job->jobID, sequence, EETargetX, EETargetY, EEth, EEx,EEy,  150.0)) goto GOTO_ORDER_CANCEL;
            //        ///////////////////////////////////////////////////////////////////////////////////////////////////////
            ret = robotKin.invKinEE_W_XYRZ( EE_entrySyrup1.X, EE_entrySyrup1.Y, EE_entrySyrup1.Th,  EE_entrySyrup1.Z);

            if(!ret) {
                cout<< "action_order:: no solution"<<endl;
                goto GOTO_ORDER_CANCEL;
            }
            // motion_delay(job->jobID, sequence, 200);
            motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, 100.0);
            // Move to the syrup place #3 from the entrance through the linear motion...
            if(!motion_EELinear(  job->jobID, sequence, EE_entrySyrup1.X, EE_entrySyrup1.Y, EE_entrySyrup1.Th,  EE_targetSyrup1.X, EE_targetSyrup1.Y,  EE_entrySyrup1.S)) goto GOTO_ORDER_CANCEL; // check if there is solution of IK for the target position...
            // Circular motion...
            motion_circle(job->jobID, sequence,EE_targetSyrup1.X, EE_targetSyrup1.Y, EE_targetSyrup1.Th*RTOD , circleRadius, 360*circleTurns,  circleSpeed );
            //if(!motion_spiral(job->jobID, sequence, EE_targetSyrup1.X, EE_targetSyrup1.Y, EE_targetSyrup1.Th*RTOD , spiralRadius, 360*spiralTurns,  EE_targetSyrup1.Z, sprialDownHeight ,spiralSpeed )) goto GOTO_ORDER_CANCEL;
            // Extract cup to the entrance position through the linear motion...
            if(!motion_EELinear(  job->jobID, sequence, EE_targetSyrup1.X-spiralRadius, EE_targetSyrup1.Y, EE_targetSyrup1.Th, EE_entrySyrup1.X, EE_entrySyrup1.Y,  EE_targetSyrup1.S)) goto GOTO_ORDER_CANCEL;
            ///////////////////////////////////////////////////////////////////////////////////////////////////////
            if(!motion_EERotation(job->jobID, sequence, EE_entrySyrup1.X, EE_entrySyrup1.Y, EE_entrySyrup1.Th, EE_rotationCrossTh, EE_rotationCrossSpeed)) goto GOTO_ORDER_CANCEL;
            // Carrying cup arm configuration to move around...
             motion_rotateArmJoint(job->jobID, sequence, 37, -127.0, 100.0);// Only Arm configure...
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            // Move to the entrance of the syrup place #3...
//            ret = robotKin.invKinEE_W_XYRZ( EE_entrySyrup2.X, EE_entrySyrup2.Y, EE_entrySyrup2.Th,  EE_entrySyrup2.Z);

//            if(!ret) {
//                cout<< "action_order:: no solution"<<endl;
//                goto GOTO_ORDER_CANCEL;
//            }
//            // motion_delay(job->jobID, sequence, 200);
//            motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, 100.0);
//            // Move to the syrup place #3 from the entrance through the linear motion...
//            double sprialDownHeight = -50.0;
//            if(!motion_EELinear(  job->jobID, sequence, EE_entrySyrup2.X, EE_entrySyrup2.Y, EE_entrySyrup2.Th,  EE_targetSyrup2.X, EE_targetSyrup2.Y,  EE_entrySyrup2.S)) goto GOTO_ORDER_CANCEL; // check if there is solution of IK for the target position...
//            // motion_delay(job->jobID, sequence, 200);
//            if(!motion_spiral(job->jobID, sequence, EE_targetSyrup2.X, EE_targetSyrup2.Y, EE_targetSyrup2.Th*RTOD , spiralRadius, 360*spiralTurns,  EE_targetSyrup2.Z, sprialDownHeight ,spiralSpeed )) goto GOTO_ORDER_CANCEL;
//            // Extract cup to the entrance position through the linear motion...
//            if(!motion_EELinear(  job->jobID, sequence, EE_targetSyrup2.X-spiralRadius, EE_targetSyrup2.Y, EE_targetSyrup2.Th, EE_entrySyrup2.X, EE_entrySyrup2.Y,  EE_targetSyrup2.S)) goto GOTO_ORDER_CANCEL;
//            ///////////////////////////////////////////////////////////////////////////////////////////////////////
//            if(!motion_EERotation(job->jobID, sequence, EE_entrySyrup2.X, EE_entrySyrup2.Y, EE_entrySyrup2.Th, EE_rotationCrossTh, EE_rotationCrossSpeed)) goto GOTO_ORDER_CANCEL;
//            // Carrying cup arm configuration to move around...
//             motion_rotateArmJoint(job->jobID, sequence, 37, -127.0, 100.0);// Only Arm configure...


            // Rotate EE by -90 degree to face to the front...
            ret = robotKin.invKinEE_W_XYRZ(EE_entryPlaceCup1.X, EE_entryPlaceCup1.Y, EE_rotationFrontTh, EE_entryPlaceCup1.Z);
            if(!ret) {
                cout<< "action_order:: no solution"<<endl;
                goto GOTO_ORDER_CANCEL;
            }
            // Move the cup to the entrance of target...
            //motion_delay(job->jobID, sequence, 200);
            motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, jointMotionSpeedPercent);

            if(!motion_EERotation(job->jobID, sequence, EE_entryPlaceCup1.X, EE_entryPlaceCup1.Y, EE_rotationFrontTh, EE_targetPlaceCup1.Th, EE_rotationFrontSpeed)) goto GOTO_ORDER_CANCEL;
            // Reach out to the Cup place
            //Received data from Client: J146 N1 G2 M30 W635.026 X635.000 Y-138.044 Z-355.000 V-1.570 A150.000
            if(!motion_EELinear(job->jobID, sequence, EE_entryPlaceCup1.X, EE_entryPlaceCup1.Y, EE_entryPlaceCup1.Th,  EE_targetPlaceCup1.X, EE_targetPlaceCup1.Y,  EE_entryPickupCup.S)) goto GOTO_ORDER_CANCEL;

            // Place cup on the counter top by lower in Z direction...
            motion_moveSingleJoint(job->jobID, sequence, SM_Z, loweringZforPlaceCup, jointMotionSpeedPercent);
            // Wait for droping cup on the counter top...
            //motion_delay(job->jobID, sequence, 500);
            // Extract Cup holder to the entrance of cup place...

            if(!motion_EELinear(  job->jobID, sequence,  EE_targetPlaceCup1.X, EE_targetPlaceCup1.Y, EE_targetPlaceCup1.Th, EE_entryPlaceCup1.X, EE_entryPlaceCup1.Y,  EE_targetPlaceCup1.S)) goto GOTO_ORDER_CANCEL;


            // Carrying cup arm configuration to move around...
            motion_rotateArmJoint(job->jobID, sequence, foldArm[0], foldArm[1], jointMotionSpeedPercent);// Only Arm configure...

            // move back to start position
            motion_moveXZ(job->jobID, sequence, cupPicupXZ[0], cupPicupXZ[1], jointMotionSpeedPercent);



            jobIDDone=-1;// Start action...
        }// test for(50)
        /*
        sequence=0;
        // lift up in Z direction...
        motion_moveSingleJoint(job->jobID, sequence, SM_Z, cupDropHeight, 100.0);

        motion_moveSingleJoint(job->jobID, sequence, SM_R1, 0, 100.0);

        motion_moveSingleJoint(job->jobID, sequence, SM_R2, 135.0, 100.0);

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        // Move to the entrance of cup drop place by the joint motion...

        ret = robotKin.invKinEE_W_XYRZ(EE_entryPickupCup.X, EE_entryPickupCup.Y, EE_entryPickupCup.Th, EE_entryPickupCup.Z);
        if(!ret) {
            cout<< "action_order:: no solution"<<endl;
            goto GOTO_ORDER_CANCEL;
        }
        motion_delay(job->jobID, sequence, 200);
        motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, 100.0);

        // Move to the cup drop place from the entrance through the linear motion...
        EETargetX =  9;
        EETargetY =  355.00;
        //        ret = robotKin.invKinEE_W_XYRZ(EETargetX, EETargetY, EEth, EEz); // check if there is solution of IK for the target position...
        //        if(!ret) {
        //            cout<< "action_order:: no solution"<<endl;
        //             goto GOTO_ORDER_CANCEL;
        //        }

        if(!motion_EELinear(job->jobID, sequence, EE_entryPickupCup.X, EE_entryPickupCup.Y, EE_entryPickupCup.Th,  EE_targetPickupCup.X, EE_targetPickupCup.Y,  linearSpeed)) goto GOTO_ORDER_CANCEL;
        // Wait for droping cup into the cup holder...
        motion_delay(job->jobID, sequence, 3000);
        // Extract cup to the entrance position through the linear motion...
        if(!motion_EELinear(  job->jobID, sequence, EETargetX, EETargetY, EEth, EEx,EEy,  linearSpeed)) goto GOTO_ORDER_CANCEL;

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        // Move to the entrance of the coffee place...
        EEx = 700.00;
        EEy = 130.00;
        EEth = 90 *DTOR;
        EEz = 371.00;
        ret = robotKin.invKinEE_W_XYRZ(EEx, EEy, EEth, EEz);
        if(!ret) {
            cout<< "action_order:: no solution"<<endl;
            goto GOTO_ORDER_CANCEL;
        }
        motion_delay(job->jobID, sequence, 200);
        motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, 100.0);

        // Move to the coffee place from the entrance through the linear motion...
        EETargetX =  EEx;
        EETargetY =  388.00;
        if(!motion_EELinear(  job->jobID, sequence, EEx,EEy, EEth,  EETargetX, EETargetY,  linearSpeed)) goto GOTO_ORDER_CANCEL; // check if there is solution of IK for the target position...

        // Wait for coffee extraction...
        motion_delay(job->jobID, sequence, 4000);
        // Extract cup to the entrance position through the linear motion...
        if(!motion_EELinear(  job->jobID, sequence, EETargetX, EETargetY, EEth, EEx,EEy,  linearSpeed)) goto GOTO_ORDER_CANCEL;
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        // Move to the entrance of the syrup place #1...
        EEx = 1100.00;
        EEy = 130.00;
        EEth = 90 *DTOR;
        EEz = 520.00;
        ret = robotKin.invKinEE_W_XYRZ(EEx, EEy, EEth, EEz);
        if(!ret) {
            cout<< "action_order:: no solution"<<endl;
            goto GOTO_ORDER_CANCEL;
        }
        motion_delay(job->jobID, sequence, 200);
        motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, 100.0);
        // Move to the syrup place #1 from the entrance through the linear motion...
        EETargetX =  EEx;
        EETargetY =  300.00;
        if(!motion_EELinear(  job->jobID, sequence, EEx,EEy, EEth,  EETargetX, EETargetY,  linearSpeed)) goto GOTO_ORDER_CANCEL; // check if there is solution of IK for the target position...

        // Circular motion...
        motion_circle(job->jobID, sequence,EETargetX, EETargetY, 90.0 , circleRadius, 360*circleTurns,  360.0/2.5 );
        // Extract cup to the entrance position through the linear motion...
        if(!motion_EELinear(  job->jobID, sequence, EETargetX, EETargetY, EEth, EEx,EEy,  linearSpeed)) goto GOTO_ORDER_CANCEL;
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        //        //////////////////////////////////////////////////////////////////////////////////////////////////////
        //        // Move to the entrance of the syrup place #3...
        //        EEx = 1200.00;
        //        EEy = 130.00;
        //        EEth = 90 *DTOR;
        //        EEz = 520.00;
        //        ret = robotKin.invKinEE_W_XYRZ(EEx, EEy, EEth, EEz);
        //        if(!ret) {
        //            cout<< "action_order:: no solution"<<endl;
        //            goto GOTO_ORDER_CANCEL;
        //        }
        //        motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, 100.0);
        //        // Move to the syrup place #1 from the entrance through the linear motion...
        //        EETargetX =  EEx;
        //        EETargetY =  300.00;
        //        double sprialDownHeight = -50.0;
        //        if(!motion_EELinear(  job->jobID, sequence, EEx,EEy, EEth,  EETargetX, EETargetY,  150.0)) goto GOTO_ORDER_CANCEL; // check if there is solution of IK for the target position...

        //        // Circular motion...Sending (action_genSpiralMotion): "G5 S-50.000 R33.000 X1200.000 Y300.000 T90.000 A1080.000  B360.000  C520.000 J26 N33\n" , selected Mode: 24
        //        if(!motion_spiral(job->jobID, sequence, EETargetX, EETargetY, 90.0 , spiralRadius, 360*spiralTurns,  EEz, sprialDownHeight ,360.0/1.0 )) goto GOTO_ORDER_CANCEL;
        //        // Extract cup to the entrance position through the linear motion...
        //        if(!motion_EELinear(  job->jobID, sequence, EETargetX-spiralRadius, EETargetY, EEth, EEx,EEy,  150.0)) goto GOTO_ORDER_CANCEL;
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        // Carrying cup arm configuration to move around...
        motion_rotateArmJoint(job->jobID, sequence, -40.0, -140.0, 100.0);// Only Arm configure...

        // Move the cup to the entrance of target...

        ret = robotKin.invKinEE_W_XYRZ(EE_entryPickupCup.X, EE_entryPickupCup.Y, EE_entryPickupCup.Th, EE_entryPickupCup.Z);
        if(!ret) {
            cout<< "action_order:: no solution"<<endl;
            goto GOTO_ORDER_CANCEL;
        }
        motion_delay(job->jobID, sequence, 200);
        motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, 100.0);
        // Rotate EE by -90 degree to face to the front...
        if(!motion_EERotation(job->jobID, sequence, EEx, EEy, EEth, -90*DTOR, 40*DTOR)) goto GOTO_ORDER_CANCEL;
        // Reach out to the Cup place
        //Received data from Client: J146 N1 G2 M30 W635.026 X635.000 Y-138.044 Z-355.000 V-1.570 A150.000
        if(!motion_EELinear(  job->jobID, sequence, EEx,EEy, -90*DTOR,  EETargetX, EETargetY,  linearSpeed)) goto GOTO_ORDER_CANCEL;

        // Place cup on the counter top by lower in Z direction...
        motion_moveSingleJoint(job->jobID, sequence, SM_Z, 95, 100.0);
        // Wait for droping cup on the counter top...
        motion_delay(job->jobID, sequence, 500);
        // Extract Cup holder to the entrance of cup place...
        if(!motion_EELinear(  job->jobID, sequence,  EETargetX, EETargetY,  -90*DTOR, EEx,EEy,  linearSpeed)) goto GOTO_ORDER_CANCEL;

        // lift up in Z direction...
        //         motion_moveSingleJoint(job->jobID, sequence, SM_Z, cupDropHeight, 100.0);
        // move back to start position
        motion_moveXZ(job->jobID, sequence, 100, cupDropHeight, 100.0);

        // Carrying cup arm configuration to move around...
        motion_rotateArmJoint(job->jobID, sequence, -40.0, -140.0, 100.0);// Only Arm configure...

        jobIDDone=-1;// Start action...
        */
    }
    else if(job->mode==2) {
        job->jobID=0;
        for(int i=0; i<1; i++) {
            //cout<<"=================== Loop No:"<<i+1<<" ===================="<<endl;
            job->jobID++;
            sequence=0;

            // Carrying cup arm configuration to move around...
            motion_rotateArmJoint(job->jobID, sequence, foldArm[0], foldArm[1], jointMotionSpeedPercent);// Only Arm configure...

            // move back to start position
            motion_moveXZ(job->jobID, sequence, cupPicupXZ[0], cupPicupXZ[1], jointMotionSpeedPercent);

            // lift up in Z direction...
            motion_moveSingleJoint(job->jobID, sequence, SM_Z, cupPicupXZ[1], 100.0);



            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // Move to the entrance of cup drop place by the joint motion...

            ret = robotKin.invKinEE_W_XYRZ(EE_entryPickupCup.X, EE_entryPickupCup.Y, EE_entryPickupCup.Th, EE_entryPickupCup.Z);
            if(!ret) {
                cout<< "action_order:: no solution"<<endl;
                goto GOTO_ORDER_CANCEL;
            }

            motion_moveSingleJoint(job->jobID, sequence, SM_R1, robotKin.param.th1*RTOD, 100.0);

            motion_moveSingleJoint(job->jobID, sequence, SM_R2, robotKin.param.th2*RTOD, 100.0);
            motion_moveSingleJoint(job->jobID, sequence, SM_X, robotKin.param.Px, 100.0);

            // motion_delay(job->jobID, sequence, 200);
//            motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, jointMotionSpeedPercent);

            // Move to the cup drop place from the entrance through the linear motion...
            //        ret = robotKin.invKinEE_W_XYRZ(EETargetX, EETargetY, EEth, EEz); // check if there is solution of IK for the target position...
            //        if(!ret) {
            //            cout<< "action_order:: no solution"<<endl;
            //             goto GOTO_ORDER_CANCEL;
            //        }

            if(!motion_EELinear(job->jobID, sequence, EE_entryPickupCup.X, EE_entryPickupCup.Y, EE_entryPickupCup.Th,  EE_targetPickupCup.X, EE_targetPickupCup.Y,  EE_entryPickupCup.S)) goto GOTO_ORDER_CANCEL;
            motion_moveSingleJoint(job->jobID, sequence, SM_Z, EE_entryPickupCup.Z+cupApproachZHeight, 100.0);
            // Wait for droping cup into the cup holder...
            //motion_delay(job->jobID, sequence, 1500);

            // ... temporary commented, please uncomment it ...
//            motion_dropCup(job->jobID, sequence, dropCupDelayTime);

            motion_moveSingleJoint(job->jobID, sequence, SM_Z, EE_entryPickupCup.Z-cupApproachZHeight, 100.0);
            // Extract cup to the entrance position through the linear motion...
            if(!motion_EELinear(  job->jobID, sequence,  EE_targetPickupCup.X, EE_targetPickupCup.Y, EE_targetPickupCup.Th, EE_entryPickupCup.X, EE_entryPickupCup.Y,  EE_targetPickupCup.S)) goto GOTO_ORDER_CANCEL;

            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // Move to the entrance of the coffee place...

            ret = robotKin.invKinEE_W_XYRZ(EE_entryCoffeeMachine.X, EE_entryCoffeeMachine.Y, EE_entryCoffeeMachine.Th, EE_entryCoffeeMachine.Z);
            if(!ret) {
                cout<< "action_order:: no solution"<<endl;
                goto GOTO_ORDER_CANCEL;
            }
            //motion_delay(job->jobID, sequence, 200);
            motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, jointMotionSpeedPercent);
            //motion_delay(job->jobID, sequence, 200);
            // Move to the coffee place from the entrance through the linear motion...
            if(!motion_EELinear(  job->jobID, sequence, EE_entryCoffeeMachine.X, EE_entryCoffeeMachine.Y, EE_entryCoffeeMachine.Th,  EE_targetCoffeeMachine.X, EE_targetCoffeeMachine.Y,  EE_entryCoffeeMachine.S)) goto GOTO_ORDER_CANCEL; // check if there is solution of IK for the target position...

            // Wait for coffee extraction...
            motion_delay(job->jobID, sequence, 2500);
            // Extract cup to the entrance position through the linear motion...
            if(!motion_EELinear(  job->jobID, sequence, EE_targetCoffeeMachine.X, EE_targetCoffeeMachine.Y, EE_targetCoffeeMachine.Th,  EE_entryCoffeeMachine.X, EE_entryCoffeeMachine.Y,  EE_targetCoffeeMachine.S)) goto GOTO_ORDER_CANCEL;
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            // Move to the entrance of the syrup place #1...
            //        EEx = 1100.00;
            //        EEy = 130.00;
            //        EEth = 90 *DTOR;
            //        EEz = 520.00;
            //        ret = robotKin.invKinEE_W_XYRZ(EEx, EEy, EEth, EEz);
            //        if(!ret) {
            //            cout<< "action_order:: no solution"<<endl;
            //            goto GOTO_ORDER_CANCEL;
            //        }
            //        motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, 100.0);
            //        // Move to the syrup place #1 from the entrance through the linear motion...
            //        EETargetX =  EEx;
            //        EETargetY =  300.00;
            //        if(!motion_EELinear(  job->jobID, sequence, EEx,EEy, EEth,  EETargetX, EETargetY,  150.0)) goto GOTO_ORDER_CANCEL; // check if there is solution of IK for the target position...

            //        // Circular motion...
            //        motion_circle(job->jobID, sequence,EETargetX, EETargetY, 90.0 , circleRadius, 360*circleTurns,  360.0/2.5 );
            //        // Extract cup to the entrance position through the linear motion...
            //        if(!motion_EELinear(  job->jobID, sequence, EETargetX, EETargetY, EEth, EEx,EEy,  150.0)) goto GOTO_ORDER_CANCEL;
            //        ///////////////////////////////////////////////////////////////////////////////////////////////////////

            //////////////////////////////////////////////////////////////////////////////////////////////////////
            // Move to the entrance of the syrup place #3...
            ret = robotKin.invKinEE_W_XYRZ( EE_entrySyrup2.X, EE_entrySyrup2.Y, EE_entrySyrup2.Th,  EE_entrySyrup2.Z);

            if(!ret) {
                cout<< "action_order:: no solution"<<endl;
                goto GOTO_ORDER_CANCEL;
            }
            // motion_delay(job->jobID, sequence, 200);
            motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, 100.0);
            // Move to the syrup place #3 from the entrance through the linear motion...
            double sprialDownHeight = -50.0;
            if(!motion_EELinear(  job->jobID, sequence, EE_entrySyrup2.X, EE_entrySyrup2.Y, EE_entrySyrup2.Th,  EE_targetSyrup2.X, EE_targetSyrup2.Y,  EE_entrySyrup2.S)) goto GOTO_ORDER_CANCEL; // check if there is solution of IK for the target position...
            // motion_delay(job->jobID, sequence, 200);
            if(!motion_spiral(job->jobID, sequence, EE_targetSyrup2.X, EE_targetSyrup2.Y, EE_targetSyrup2.Th*RTOD , spiralRadius, 360*spiralTurns,  EE_targetSyrup2.Z, sprialDownHeight ,spiralSpeed )) goto GOTO_ORDER_CANCEL;
            // Extract cup to the entrance position through the linear motion...
            if(!motion_EELinear(  job->jobID, sequence, EE_targetSyrup2.X-spiralRadius, EE_targetSyrup2.Y, EE_targetSyrup2.Th, EE_entrySyrup2.X, EE_entrySyrup2.Y,  EE_targetSyrup2.S)) goto GOTO_ORDER_CANCEL;
            ///////////////////////////////////////////////////////////////////////////////////////////////////////
            if(!motion_EERotation(job->jobID, sequence, EE_entrySyrup2.X, EE_entrySyrup2.Y, EE_entrySyrup2.Th, EE_rotationCrossTh, EE_rotationCrossSpeed)) goto GOTO_ORDER_CANCEL;
            // Carrying cup arm configuration to move around...
             motion_rotateArmJoint(job->jobID, sequence, 37, -127.0, 100.0);// Only Arm configure...


            // Rotate EE by -90 degree to face to the front...
            ret = robotKin.invKinEE_W_XYRZ(EE_entryPlaceCup2.X, EE_entryPlaceCup2.Y, EE_rotationFrontTh, EE_entryPlaceCup2.Z);
            if(!ret) {
                cout<< "action_order:: no solution"<<endl;
                goto GOTO_ORDER_CANCEL;
            }
            // Move the cup to the entrance of target...
            //motion_delay(job->jobID, sequence, 200);
            motion_moveAllJoints(job->jobID, sequence, robotKin.param.Px, robotKin.param.th1*RTOD, robotKin.param.th2*RTOD, robotKin.param.Pz, jointMotionSpeedPercent);

            if(!motion_EERotation(job->jobID, sequence, EE_entryPlaceCup2.X, EE_entryPlaceCup2.Y, EE_rotationFrontTh, EE_targetPlaceCup2.Th, EE_rotationFrontSpeed)) goto GOTO_ORDER_CANCEL;
            // Reach out to the Cup place
            //Received data from Client: J146 N1 G2 M30 W635.026 X635.000 Y-138.044 Z-355.000 V-1.570 A150.000
            if(!motion_EELinear(job->jobID, sequence, EE_entryPlaceCup2.X, EE_entryPlaceCup2.Y, EE_entryPlaceCup2.Th,  EE_targetPlaceCup2.X, EE_targetPlaceCup2.Y,  EE_entryPickupCup.S)) goto GOTO_ORDER_CANCEL;

            // Place cup on the counter top by lower in Z direction...
            motion_moveSingleJoint(job->jobID, sequence, SM_Z, loweringZforPlaceCup, jointMotionSpeedPercent);
            // Wait for droping cup on the counter top...
            //motion_delay(job->jobID, sequence, 500);
            // Extract Cup holder to the entrance of cup place...

            if(!motion_EELinear(  job->jobID, sequence,  EE_targetPlaceCup2.X, EE_targetPlaceCup2.Y, EE_targetPlaceCup2.Th, EE_entryPlaceCup2.X, EE_entryPlaceCup2.Y,  EE_targetPlaceCup2.S)) goto GOTO_ORDER_CANCEL;


            // Carrying cup arm configuration to move around...
            motion_rotateArmJoint(job->jobID, sequence, foldArm[0], foldArm[1], jointMotionSpeedPercent);// Only Arm configure...

            // move back to start position
            motion_moveXZ(job->jobID, sequence, cupPicupXZ[0], cupPicupXZ[1], jointMotionSpeedPercent);



            jobIDDone=-1;// Start action...
        }// test for(50)
    }
    return true;
GOTO_ORDER_CANCEL:
    cout<<"------ Order Canceled!! -------\n";
    return false;
}

bool mkServer_Serial2Arm3::action_dropCup(PacketJobs *job)
{
    issuedXCMD  = SC_DROP_CUP;
    char packet[68];
    sprintf(packet, "G%dM%dJ%dN%d", SC_DROP_CUP, job->mode, job->jobID, job->nSequence);
//    QByteArray data = QByteArray(packet, strlen(packet));
    qDebug()<<"Sending(action_dropCup)="<<packet;//<<endl<<flush;//
    serialHandler[0]->write_crc(packet, strlen(packet));
//    serialHandler[0]->write(data);
    return true;
}



void mkServer_Serial2Arm3::motion_rotateArmJoint(int jobID, int &sequence, double rotDeg1, double rotDeg2, double speedPercent)
{
    const int nJ=3;
    int s=0;

    PacketJobs *newJob[nJ]={nullptr};

    for(int i=0; i<nJ; i++) {
        newJob[i] = new PacketJobs;
    }

    newJob[s]->packf("J%d N%d G%d M%d W0 X%5.3f Y%5.3f Z0 V%5.3f A%5.3f\n",
                     jobID, sequence+1, SC_SET_SPEED, MULTI_ARM_JNT_MODE, rotDeg1, rotDeg2, speedPercent, speedPercent);
    newJob[s]->unpack();s++;sequence++;

    newJob[s]->packf("J%d N%d G%d M%d\n", jobID, sequence+1, SC_MOVE, MULTI_ARM_JNT_MODE);
    newJob[s]->unpack();s++;sequence++;

    newJob[s]->packf("G%d J%d N%d", SC_STATUS_ALL_POS, jobID, 0);
    newJob[s]->unpack();s++;sequence++;

    for(int i=0; i<s; i++) {
        queuePacketJobs.push(newJob[i]);
    }

}

void mkServer_Serial2Arm3::motion_moveAllJoints(int jobID, int &sequence, double posX, double R1Deg, double R2Deg, double posZ, double speedPercent)
{
    const int nJ=3;
    int s=0;

    PacketJobs *newJob[nJ]={nullptr};

    for(int i=0; i<nJ; i++) {
        newJob[i] = new PacketJobs;
    }

    newJob[s]->packf("J%d N%d G%d M%d W%5.3f X%5.3f Y%5.3f Z%5.3f V%5.3f A%5.3f\n",
                     jobID, sequence+1, SC_SET_SPEED, MULTI_ALL_JNT_MODE, posX, R1Deg, R2Deg, posZ, speedPercent, speedPercent);
    newJob[s]->unpack();s++;sequence++;

    newJob[s]->packf("J%d N%d G%d M%d\n", jobID, sequence+1, SC_MOVE, MULTI_ALL_JNT_MODE); newJob[s]->unpack();s++;sequence++;

//    newJob[s]->packf("J%d N%d G%d M%d\n", jobID, sequence+1, SC_STATUS_ALL_POS, MULTI_ALL_JNT_MODE); newJob[s]->unpack();s++;sequence++;

    newJob[s]->packf("G%d J%d N%d", SC_STATUS_ALL_POS, jobID, 0);
    newJob[s]->unpack();s++;sequence++;
    for(int i=0; i<s; i++) {
        queuePacketJobs.push(newJob[i]);
    }
}

void mkServer_Serial2Arm3::motion_moveXZ(int jobID, int &sequence, double posX, double posZ, double speedPercent)
{
    const int nJ=3;
    int s=0;

    PacketJobs *newJob[nJ]={nullptr};

    for(int i=0; i<nJ; i++) {
        newJob[i] = new PacketJobs;
    }
    // Cup Place approach #1...
    //newJob[s]->packf("J%d N%d G%d M%d W720.290 X-39.995 Y-139.995 Z150.000 V100.000 A100.000\n",jobID, sequence+1, SC_SET_SPEED, MULTI_XZ_JNT_MODE); newJob[s]->unpack();s++;sequence++;
    newJob[s]->packf("J%d N%d G%d M%d W%5.3f X0 Y0 Z%5.3f V%5.3f A%5.3f\n",
                     jobID, sequence+1, SC_SET_SPEED, MULTI_XZ_JNT_MODE, posX, posZ, speedPercent, speedPercent);
    newJob[s]->unpack();s++;sequence++;

    newJob[s]->packf("J%d N%d G%d M%d\n", jobID, sequence+1, SC_MOVE, MULTI_ARM_JNT_MODE);          newJob[s]->unpack();s++;sequence++;

    newJob[s]->packf("G%d J%d N%d", SC_STATUS_ALL_POS, jobID, 0);
    newJob[s]->unpack();s++;sequence++;
    for(int i=0; i<s; i++) {
        queuePacketJobs.push(newJob[i]);
    }
}

void mkServer_Serial2Arm3::motion_moveSingleJoint(int jobID, int &sequence, int selectedAxis, double pos, double speedPercent)
{
    const int nJ=3;
    int s=0;

    PacketJobs *newJob[nJ]={nullptr};

    for(int i=0; i<nJ; i++) {
        newJob[i] = new PacketJobs;
    }

    // 1. Joint motion
    // Data[0] = X- absolute position
    // Data[1] = R1- absolute position
    // Data[2] = R2- absolute position
    // Data[3] = Z- absolute position
    // Data[4] = percent of max velocity for each axis
    // Data[5] =percent of max acceleration for each axis

    newJob[s]->packf("J%d N%d G%d M%d W0 X0 Y0 Z0 V%5.3f A%5.3f\n",
                     jobID, sequence+1, SC_SET_SPEED, selectedAxis, speedPercent, speedPercent);
    newJob[s]->unpack();
    newJob[s]->data[selectedAxis]=pos;
    s++;sequence++;

    newJob[s]->packf("J%d N%d G%d M%d\n", jobID, sequence+1, SC_MOVE, SINGLE_JNT_MODE);
    newJob[s]->unpack();s++;sequence++;

    newJob[s]->packf("G%d J%d N%d", SC_STATUS_ALL_POS, jobID, 0);
    newJob[s]->unpack();s++;sequence++;


    for(int i=0; i<s; i++) {
        queuePacketJobs.push(newJob[i]);
    }
}

bool mkServer_Serial2Arm3::motion_EERotation(int jobID, int &sequence, double EEx, double EEy, double EEthStartRad, double EEthEndRad, double speed)
{
    const int nJ=3;
    int s=0;
    bool ret = robotKin.invKinEE_W_XYR(EEx, EEy, EEthEndRad); // check if there is solution of IK for the target position...
    if(!ret) return false;
    PacketJobs *newJob[nJ]={nullptr};

    for(int i=0; i<nJ; i++) {
        newJob[i] = new PacketJobs;
    }
    // Cup Place approach #1...
    newJob[s]->packf("J%d N%d G%d M%d W%5.3f X%5.3f Y%5.3f Z%5.3f V%5.3f\n",
                     jobID, sequence+1, SC_GEN_EEROTATION, CARTESIAN_MODE, EEx, EEy, EEthStartRad, EEthEndRad, speed);
    newJob[s]->unpack();s++;sequence++;

    newJob[s]->packf("J%d N%d G%d M%d\n", jobID, sequence+1, SC_MOVE, CARTESIAN_MODE);
    newJob[s]->unpack();s++;sequence++;

    newJob[s]->packf("G%d J%d N%d;", SC_STATUS_ALL_POS, jobID, 0);
    newJob[s]->unpack();s++;sequence++;

    for(int i=0; i<s; i++) {
        queuePacketJobs.push(newJob[i]);
    }
    return true;
}

bool mkServer_Serial2Arm3::motion_EELinear(int jobID, int &sequence, double EEx, double EEy, double EEth, double EETargetX, double EETargetY, double speed)
{
    const int nJ=3;
    int s=0;

    bool ret = robotKin.invKinEE_W_XYR(EETargetX, EETargetY, EEth); // check if there is solution of IK for the target position...
    if(!ret) return false;


    PacketJobs *newJob[nJ]={nullptr};

    for(int i=0; i<nJ; i++) {
        newJob[i] = new PacketJobs;
    }
    // Cup Place approach #1...
    newJob[s]->packf("J%d N%d G%d M%d W%5.3f X%5.3f Y%5.3f Z%5.3f V%5.3f A%5.3f B%5.3f C%5.3f\n",
                     jobID, sequence+1, SC_GEN_EELINEAR, CARTESIAN_MODE, EEx, EETargetX, EEy, EETargetY, robotProperty[3].getDistanceFromStep(),robotProperty[3].getDistanceFromStep(), EEth, speed);
    newJob[s]->unpack();s++;sequence++;

    newJob[s]->packf("J%d N%d G%d M%d\n", jobID, sequence+1, SC_MOVE, CARTESIAN_MODE);
    newJob[s]->unpack();s++; sequence++;

    newJob[s]->packf("G%d J%d N%d;", SC_STATUS_ALL_POS, jobID, 0);
    newJob[s]->unpack();s++;sequence++;

    for(int i=0; i<s; i++) {
        queuePacketJobs.push(newJob[i]);
    }
    return true;
}

bool mkServer_Serial2Arm3::motion_circle(int jobID, int &sequence, double EEx, double EEy, double EEthDeg, double radius, double arcAng, double speed)
{
    const int nJ=3;
    int s=0;

    //    MKZoeRobotKin testRobotKin;
    //    bool ret = testRobotKin.invKinEE_W_XYR(EEx+radius,//EEx
    //                                           EEy, //EEy
    //                                           EEthDeg*DTOR //EETh
    //                                           );//EEZ
    //    if(!ret) return false;

    PacketJobs *newJob[nJ]={nullptr};

    for(int i=0; i<nJ; i++) {
        newJob[i] = new PacketJobs;
    }
    // Cup Place approach #1...
    newJob[s]->packf("J%d N%d G%d M%d W%5.3f X%5.3f Y%5.3f Z%5.3f V%5.3f A%5.3f\n",
                     jobID, sequence+1, SC_GEN_CIRCLE, CIRCLE_MODE, speed, radius, EEx,  EEy, EEthDeg, arcAng);
    newJob[s]->unpack();s++;sequence++;

    newJob[s]->packf("J%d N%d G%d M%d\n", jobID, sequence+1, SC_MOVE, CIRCLE_MODE);
    newJob[s]->unpack();s++; sequence++;

    newJob[s]->packf("G%d J%d N%d;", SC_STATUS_ALL_POS, jobID, 0);
    newJob[s]->unpack();s++;sequence++;

    for(int i=0; i<s; i++) {
        queuePacketJobs.push(newJob[i]);
    }
    return true;
}

bool mkServer_Serial2Arm3::motion_spiral(int jobID, int &sequence, double EEx, double EEy, double EEthDeg, double radius, double arcAng, double EEz, double heightZ, double speed)
{
    const int nJ=3;
    int s=0;

    MKZoeRobotKin testRobotKin;
    bool ret = testRobotKin.invKinEE_W_XYRZ(EEx,//EEx
                                            EEy, //EEy
                                            EEthDeg*DTOR, //EETh
                                            EEz+heightZ);//EEZ
    if(!ret) return false;

    PacketJobs *newJob[nJ]={nullptr};

    for(int i=0; i<nJ; i++) {
        newJob[i] = new PacketJobs;
    }
    // Cup Place approach #1...
    newJob[s]->packf("J%d N%d G%d M%d W%5.3f X%5.3f Y%5.3f Z%5.3f V%5.3f A%5.3f B%5.3f C%5.3f\n",
                     jobID, sequence+1, SC_GEN_SPIRAL, CIRCLE_MODE, speed, radius, EEx,  EEy, EEthDeg, arcAng, EEz, heightZ);
    newJob[s]->unpack();s++;sequence++;

    newJob[s]->packf("J%d N%d G%d M%d\n", jobID, sequence+1, SC_MOVE, CIRCLE_MODE);
    newJob[s]->unpack();s++; sequence++;

    newJob[s]->packf("G%d J%d N%d;", SC_STATUS_ALL_POS, jobID, 0);
    newJob[s]->unpack();s++;sequence++;

    for(int i=0; i<s; i++) {
        queuePacketJobs.push(newJob[i]);
    }
    return true;
}

void mkServer_Serial2Arm3::motion_delay(int jobID, int &sequence, int ms)
{
    const int nJ=1;
    int s=0;

    PacketJobs *newJob[nJ]={nullptr};

    for(int i=0; i<nJ; i++) {
        newJob[i] = new PacketJobs;
    }
    newJob[s]->packf("J%d N%d G%d M%d\n",jobID, sequence+1, SC_TIME_DELAY_MC, ms);  newJob[s]->unpack();s++; sequence++;

    for(int i=0; i<s; i++) {
        queuePacketJobs.push(newJob[i]);
    }
}

void mkServer_Serial2Arm3::motion_dropCup(int jobID, int &sequence, int runningTime)
{
    const int nJ=1;
    int s=0;

    PacketJobs *newJob[nJ]={nullptr};

    for(int i=0; i<nJ; i++) {
        newJob[i] = new PacketJobs;
    }
    newJob[s]->packf("J%d N%d G%d M%d\n",jobID, sequence+1, SC_DROP_CUP, runningTime);  newJob[s]->unpack();s++; sequence++;

    for(int i=0; i<s; i++) {
        queuePacketJobs.push(newJob[i]);
    }
}



////////////////////////////////////////////////////////////////
void mkServer_Serial2Arm3::action_moveRobotStation(PacketJobs *job)
{
    if(statusAllSpeedReady.isDone()) {
        issuedXCMD  = SC_MOVE;
        char packet[68];
        serialSendTimer.stop();
        sprintf(packet, "G%dA%dJ%dN%d",SC_MOVE, statusAllSpeedReady.selectMotors, job->jobID, job->nSequence);
//        QByteArray data = QByteArray((char*)packet, strlen(packet));
//        serialHandler[0]->write(data);
        serialHandler[0]->write_crc(packet, strlen(packet));
        statusAllSpeedReady.reset();
        // ... request current position every 70 msec by calling the action_requestMotionPosition...
//        delay(WRITE_DELAY, false);

        // ...Every 150ms request update position...
//        serialSendTimer.start(70);

        qDebug()<<"Sending (action_moveRobot):"<<packet;

    }
}

void mkServer_Serial2Arm3::action_setPosition(PacketJobs *job)
{
    issuedXCMD  = SC_SETPOS;
    char packet[128];
    sprintf(packet, "G%dA%dB%dC%dD%dJ%dN%d", SC_SETPOS,
            robotProperty[0].absSteps, robotProperty[1].absSteps, robotProperty[2].absSteps, robotProperty[3].absSteps, job->jobID, job->nSequence);
//    QByteArray data = QByteArray((char*)packet, strlen(packet));
    // qDebug()<<"Sending(action_setPosition) data:"<<data;
    qDebug()<<"Sending(action_setPosition)="<<packet;//<<endl<<flush;//
    serialHandler[0]->write_crc(packet, strlen(packet));
//    serialHandler[0]->write(data);
}

void mkServer_Serial2Arm3::action_setZeroEncoder(PacketJobs *job)
{
    issuedXCMD  = SC_SET_ZERO_ENCODER;
    char packet[128];
    sprintf(packet, "G%dM%dJ%dN%d", SC_SET_ZERO_ENCODER,
            job->mode, job->jobID, job->nSequence);
//    QByteArray data = QByteArray((char*)packet, strlen(packet));
    qDebug()<<"Sending(action_setZeroEncoder)="<<packet;//<<endl<<flush;//
//    serialHandler[0]->write(data);
    serialHandler[0]->write_crc(packet, strlen(packet));
}

void mkServer_Serial2Arm3::action_stop(int cmdID, int jobID)
{

    serialSendTimer.stop();
    issuedXCMD  = SC_STOP;
    char packet[68];

    serialHandler[0]->writeBuffer.bReadyToWrite=false;
    serialHandler[0]->writeBuffer.reset();// remove all data in writeBuffer.

    sprintf(packet, "G%dJ%dN%d", SC_STOP, jobID, 0);
    QByteArray data = QByteArray((char*)packet, strlen(packet));
    // qDebug()<<"Sending(action_stop) data:"<<data;
    qDebug()<<"Sending(action_stop)="<<packet;//<<endl<<flush;
//    serialHandler[0]->write_crc(packet, strlen(packet));
    serialHandler[0]->write(data);

    while(!queuePacketJobs.empty()) {
        delete queuePacketJobs.front();
        queuePacketJobs.pop();
    }
    tcpServerForOrder->ringBuffer.readDone();
    serialRequestAllPosition();
    bSocketOrderProcessDone=true;
}

void mkServer_Serial2Arm3::action_cancelAllJobs(int cmd, int ErrorCode, int jobID, int seqNumber)
{

    //////////////////////////////////////////
    // Remove current message from the MicroController...
    serialHandler[0]->readBuffer.readDone();

    /////////////////////////////////////////
    // Let other messege get through from the MicroController..
    bSerialProcessDone=true;
    serialSendTimer.stop();

    if(ErrorCode>=100 && ErrorCode<200){
        serialSendTimer.stop();
        serialHandler[0]->writeBuffer.bReadyToWrite=true;
        serialHandler[0]->writeBuffer.bErrorOccered=true;
         cout<<"Error occured and resending: cmd"<<cmd<<", ErrorCode: "<<ErrorCode<<", jobID:"<<jobID<<", seqNumber:"<<seqNumber<<endl<<flush;
        return;
    }

     cout<<"action_cancelAllJobs: cmd"<<cmd<<", ErrorCode: "<<ErrorCode<<", jobID:"<<jobID<<", seqNumber:"<<seqNumber<<endl<<flush;
    /////////////////////////////////////////
    // Send Stop Message to Microcontroller ...

    issuedXCMD  = SC_STOP;
    char packet[68];
    sprintf(packet, "G%dJ%dN%d", SC_STOP, jobID, seqNumber);
//    QByteArray data = QByteArray((char*)packet, strlen(packet));
    qDebug()<<"Sending(action_stop)="<<packet;//<<endl<<flush;
//    serialHandler[0]->write(data);
    serialHandler[0]->write_crc(packet, strlen(packet));
//    delay(WRITE_DELAY, false);

    sprintf(packet, "G%dJ%dN%d", SC_STATUS_ALL_POS, jobID, 0);
    qDebug()<<"Sending(requestAllPosition)="<<packet;//<<endl<<flush;//
    serialHandler[0]->write_crc(packet, strlen(packet));
//    data = QByteArray((char*)packet, strlen(packet));
//    serialHandler[0]->write(data);


    ///////////////////////////////////////////
    /// Remove all messages in ring buffer of Socket...
    tcpServerForOrder->ringBuffer.reset();
    ///////////////////////////////////////////
    // Remove all messages from Socket Client on Queue...
    while(!queuePacketJobs.empty()) {
        PacketJobs *job_tmp = queuePacketJobs.front();
        //job->print();
        delete job_tmp;
        queuePacketJobs.pop();
    }
    //job=nullptr;

    //////////////////////////////////////
    // Let message get through into processSocketExecuteTimer() to excute a command...
    jobIDDone = -1;

    ///////////////////////////////////////////////
    /// Report the Client with Error Code...
    response2Socket_jobDone(cmd, ErrorCode);
    cout<<"--- action_cancelAllJobs ---";//<<endl<<flush;
}





////////////////////////////////////////////////////////////////
void mkServer_Serial2Arm3::response2SocketupdateAxisPosition(int resCode, int cmdCode)
{
    PacketResSocketAxisPos packetResSocketAxisPos;
    packetResSocketAxisPos.reponseCode = resCode;
    packetResSocketAxisPos.cmdCode = cmdCode;
    ////////////////////////////////////////
    int idM=0;
    packetResSocketAxisPos.pos[idM] = robotProperty[idM].getDistanceFromStep();//[mm or deg]//.absSteps*X_STEP2DIST;
    //////////////////////////////////////
    idM=1;
    packetResSocketAxisPos.pos[idM]  = robotProperty[idM].getDistanceFromStep() - robotProperty[idM].centerPos;//[deg]//.absSteps*X_STEP2DIST;
    //////////////////////////////////////
    idM=2;
    packetResSocketAxisPos.pos[idM]  = robotProperty[idM].getDistanceFromStep()- robotProperty[idM].centerPos;//[deg];
    //////////////////////////////////////
    idM=3;
    packetResSocketAxisPos.pos[idM]  = robotProperty[idM].getDistanceFromStep();//[mm ]
    packetResSocketAxisPos.jobID = currentPacketJob.jobID;
    packetResSocketAxisPos.nSequence = currentPacketJob.nSequence;
    packetResSocketAxisPos.pack();
    //////////////////////////////////////
    // Send axis position to client through TCP/IP...
    if(packetResSocketAxisPos.length()>0){
        if(tcpServerForOrder->isConnected && tcpServerForOrder->socket->isOpen())
            tcpServerForOrder->socket->write(packetResSocketAxisPos.get());
    }
    if(cmdCode!=SC_UPDATE_MOTION)
        qDebug()<<"Sending(response2SocketupdateAxisPosition) to Client="<<packetResSocketAxisPos.get()<<endl<<flush;
}
void mkServer_Serial2Arm3::response2Socket_jobDone(int cmd,  int ErrorCode)
{
    char packet[68];
    sprintf(packet, "R%d C%d J%d E%d\n", RC_STATUS_JOB_DONE, cmd, currentPacketJob.jobID, ErrorCode);
    if(tcpServerForOrder->isConnected && tcpServerForOrder->socket->isOpen()) tcpServerForOrder->socket->write(packet);
    qDebug()<<"Sending TO CLIENT(response2Socket_jobDone): cmd="<<cmd<<", "<<packet;//<<endl<<flush;
}


