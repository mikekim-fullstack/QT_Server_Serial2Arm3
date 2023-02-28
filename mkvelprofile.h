#ifndef MKVELPROFILE_H
#define MKVELPROFILE_H

#include <iostream>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <QStringList>
#include "mkGlobalDefine.h"
using namespace std;
//enum MOTOR_ID {M_X, M_Z, M_R1, M_R2};


class MKVelProfile
{
    public:
    MKVelProfile() {

    }
    ~MKVelProfile(){

    }
    public:
   // void genSpeedProfile(targetDest &target, int n);
    void genSpeedProfile(RobotProperty & robotStatus, targetDest &target, speedProfile& outSpeedProfile);
    bool code_seen(char code);
    double code_value();
    int  processCmd(RobotProperty & robotStatus,QString strCmd, char *packet);
    speedProfile * processCmdInjectX(char * strCmd, double x, int n);

    public:
     bool st_stop=true;
    int cmdIndex=0;
    long repeatCnt=0;
    char *strchr_pointer;
    char *cmdbuffer;
    speedProfile CMDData[4];
    int curIndex=0;
};
#endif // MKVELPROFILE_H
