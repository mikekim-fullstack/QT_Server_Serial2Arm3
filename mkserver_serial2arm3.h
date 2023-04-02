#ifndef MKSERVER_SERIAL2ARM3_H
#define MKSERVER_SERIAL2ARM3_H
#include <QCoreApplication>
#include <QSerialPort>
#include <iostream>
#include <QTimer>
#include <queue>
#include "mkserialport.h"
#include "mksocketclient.h"
#include "mksocketserver.h"
#include "../sharedfiles/mkRobotKin.h"
#include "mkredis.h"
#include "rediqtadapter.h"

#define NUM_PORT 1
#define NUM_AXIS 4

///////////////////////////////////////////////
typedef struct _EE_LOCATION_
{
    double X,Y,Z,Th,S;
}EE_LOCACTION;
///////////////////////////////////////////////
typedef struct _CHECK_ALL_SPEED_STATUS_{
    bool bAxis[5]={false};
    int selectMotors= 0;
    bool done=false;
    void reset() {
        memset(bAxis,0, sizeof(bool)*4);
        selectMotors= 0;
        done=false;
    }
    void add(int num) {
        bAxis[num]=true;
        if(num==0) selectMotors |=1<<SM_X;
        if(num==1) selectMotors |=1<<SM_R1;
        if(num==2) selectMotors |=1<<SM_R2;
        if(num==3) selectMotors |=1<<SM_Z;
        if(num==SM_KIN) selectMotors = 1<<SM_KIN;

    }
    bool isDone() {
        return done;
    }
    bool check(int num) {
        if(done) return true;
        bAxis[num]=false;

        int n=0;
        for(int i=0; i<4; i++) n += bAxis[i];
        if(n==0 ){ done=true; return true;}
        return false;
    }

}CHECK_SPEED_STATUS;
////////////////////////////////////////////////
class mkServer_Serial2Arm3: public QObject
{
    Q_OBJECT

private:
    QCoreApplication *app;
public:
     explicit mkServer_Serial2Arm3( QObject *parent = nullptr);
    ~mkServer_Serial2Arm3() override;
    void quit();
signals:
    void finished();
private:

    //char *strchr_pointer;
    QTimer serialResponseTimer;
    QTimer serialSendTimer;
    QTimer serialInitTimer;
    QTimer serialTimeOut;
    QTimer socketOrderResponseTimer;
    QTimer socketExecuteTimer;
    QTimer socketCameraResponseTimer;

    char rxBuffer[24][96];
    std::string tmpBuffer;
    QByteArray arrBuffer;
    CHECK_SPEED_STATUS statusAllSpeedReady, statusAllMotionDone;
    /////////////////////////////////////////////////////////

    bool bSerialProcessDone=true;
    bool bSocketOrderProcessDone=true;
    bool bSocketCameraProcessDone=true;

    bool bRobotMoving=false;

    unsigned long startActionTime=0;// .. start time since  robot moves ...
    double maxActionTime=0;//[sec] calulate the estimated time...
public:
    int selectedMotorID=0;
    QSerialPort serialPort[NUM_PORT];
    QByteArray serialData[2];
    mkSerialPort *serialHandler[2]={0};
    QString cmdLists[2];
    bool st_stop[NUM_AXIS]={true};
    int cmdIndex[NUM_AXIS]={0};
    long repeatCnt[NUM_AXIS]={0};

    bool isStartedJobForRobot=false;
    mkSocketServer * tcpServerForOrder=0;
    mkSocketServer * tcpServerForCamera=0;
    RobotProperty robotProperty[NUM_AXIS+1];// X, R1, R2, Z, Last one is for all axisis
    int  receivedJobID=0;

    bool bXUpdatePosSlider=false;
    bool bZUpdatePosSlider=false;
    bool bMotionUpdateTimer=true;// ... If bMotionUpdateTimer is true, send SC_MOTION_UPDATE every 150ms to get current posion...
    int issuedXCMD=-1;
    int currentSetJobID=0;
    char send_SerialPacket[128];
    int statusPausMode=0;

    std::queue<PacketJobs*> queuePacketJobs;
    std::queue<PacketJobs*> queueBackupPacketJobs;
    PacketJobs currentPacketJob;
    PacketBase *packetBase=0;
    MKZoeRobotKin robotKin;
    PacketAckRec packetAckRec;
    PacketAckStopRec packetAckStopRec;
    PacketStatusRec packetStatusRec;
    PacketEncoderRec packetEncoderRec;
    PacketAllPosRec packetAllPosRec;
    PacketLinearRec packetLinearRec;
    PacketEERotateRec packetEERotateRec;
    PacketCircleRec packetCircleRec;
    PacketSpiralRec packetSpiralRec;
    PacketSpeed packetSpeed;
    PacketCircle packetCircle;

//    PacketResSocketAxisPos packetResSocketAxisPos;
    volatile int jobIDDone=-1;
    volatile bool bDelayOp=false;
    int encoderJobSequence[2];// remember which sequence is encoder1 and 2...
   // volatile int currentSequence=-1;

    // Redis Client
     MKRedis mkRedis;
public slots:
    void serialAboutClose();
    void processSerialPortResponseTimer();
    void processSerialPortSendTimer();
    void processSerialPortInitTimer();
    void processSerialPortTimeOut();
    void processSocketOrderTimer();
    void processSocketCameraTimer();
    void processSocketExecuteTimer();
    void aboutToQuitApp();
    void startRun();
    void socketOrderConnected_slot();
    void socketCameraConnected_slot();
public:
    void startSerialPort();
    void serialRequestStatus(int ch);
    void serialRequestAllPosition();
    void startHomingProcess(RobotProperty & robotStatus);
    void removeJobFromQueque();
    void startMove(int select_motors);
    void loadConfigFile(QString name, RobotProperty * robotStatus);
    void savePos(QString name, RobotProperty * robotStatus);
    void delay(unsigned long ms, bool bJobDone=true);
    void handlingJobDone(int commanedCode);

    // ...Backing up the setMotions and move for pause and resume functions...
    void putSetJob2Backup(PacketJobs *job);//  reset and add setMotion.
    void putMoveJob2Backup(PacketJobs *job);//  add move command.


    ///////////////////////////////////////
    void action_initRobot(PacketJobs *job=nullptr);
    void action_requestStatus(PacketJobs *job);
    void action_requestAllPosition(PacketJobs *job);
    void action_requestEncoderValue(PacketJobs *job);
    void action_requestMotionPosition(PacketJobs *job);
    void action_reboot(PacketJobs *job);
    void action_controlPower(PacketJobs *job);
    void action_controlPower(bool bPowerOn);
    void action_controlZBrake(PacketJobs *job);
    void action_controlZBrake(bool bPowerOn);
    void action_homing(PacketJobs *job);
    bool action_genSeedProfile(PacketJobs *job);
    void action_moveRobotStation(PacketJobs *job);
    void action_setPosition(PacketJobs *job);
    void action_setZeroEncoder(PacketJobs *job);
    void action_stop(int cmdID, int jobID);
    void action_pause(int cmdID, int jobID, int mode);
    void action_getPauseStatus(int jobID);
    void action_cancelAllJobs(int cmd, int ErrorCode, int jobID, int seqNumber);
    void action_genLinearMotion(PacketJobs *job);
    void action_genRotateEEMotion(PacketJobs *job);
    void action_genCircularMotion(PacketJobs *job);
    void action_genSpiralMotion(PacketJobs *job);
    bool action_order(PacketJobs *job);
    bool action_dropCup(PacketJobs *job);
    void action_delayMicroController(PacketJobs *job);
    //////////////////////////////////////////////
    void motion_rotateArmJoint(int jobID, int &sequence, double rotDeg1, double rotDeg2, double speedPercent);
    void motion_moveAllJoints(int jobID, int &sequence, double posX, double R1Deg, double R2Deg, double posZ, double speedPercent);
    void motion_moveXZ(int jobID, int &sequence, double posX, double posZ, double speedPercent);
    void motion_moveSingleJoint(int jobID, int &sequence, int selectedAxis, double pos, double speedPercent);
    bool motion_EERotation(int jobID, int &sequence, double EEx, double EEy, double EEthStartRad, double EEthEndRad, double speed);
    bool motion_EELinear(int jobID, int &sequence, double EEx, double EEy, double EEthDeg, double EEtargetX, double EETargetY, double speed);
    bool motion_circle(int jobID, int &sequence, double EEx, double EEy, double EEthDeg, double radius, double arcAng, double speed);
    bool motion_spiral(int jobID, int &sequence, double EEx, double EEy, double EEthDeg, double radius, double arcAng,double EEz, double heightZ, double speed);
    void motion_delay(int jobID, int &sequence, int ms);
    void motion_dropCup(int jobID, int &sequence, int runningTime);
    //void motion_requestMotionPosition(int jobID, int &sequence);

    ////////////////////////////////////////////
    void response2Socket_jobDone(int cmd, int ErrorCode=0);
    void response2SocketupdateAxisPosition(int resCode, int cmdCode);

};

#endif // MKSERVER_SERIAL2ARM3_H
