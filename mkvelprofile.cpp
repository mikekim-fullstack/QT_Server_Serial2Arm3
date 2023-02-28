#include "mkvelprofile.h"

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <QFile>
#include <QDataStream>

#include <QStringList>
#include <QTextStream>
#include <QThread>

#include <QtDebug>

#include "mkGlobalDefine.h"
#include "mkvelprofile.h"
//#define SIGN(a) ((a<0)? -1:1)
/////////////////////////////////////////////////////
// 20Teeth 5M Synchronous Pulley Drawing and Specs: 20th*5mm(=100/rev)
//HBS86H Diver DEFAUTL MICROSTEP: 400
//#define BAUDRATE 250000 //115200


//extern speedProfile CMDData;

//extern double X_STEP2DIST;//[MM]
bool MKVelProfile::code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer, code);
  return (strchr_pointer != nullptr);  //Return True if a character was found
}
double  MKVelProfile::code_value()
{
    return (strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], nullptr));
}
int MKVelProfile::processCmd(RobotStatus & robotStatus, QString strCmd, char *packet)
{
    speedProfile outP;
    QByteArray ba = strCmd.toLocal8Bit();
    cmdbuffer = ba.data();
    targetDest target;
    if(code_seen('G'))
    {
        int codeValue=(int)code_value();

        ////////////////////////////
        // G1: Sending the relative step position command
        // G9: Homing Cmd
       if( codeValue == SC_SET_SPEED || codeValue == SC_HOMING)
       {
           target.seen = true;

           if(code_seen('M'))// select motor
           {
               target.motorID = (int)code_value();
           }
           else target.seen =false;

           if(code_seen('D'))// Step Distance
           {
              // if(g_MainWindow->selectedMotorID == target.motorID)
               {
                   target.distance = code_value();// steps
                   if(codeValue==SC_MOVE){
                       double totalDist = robotStatus.getDistanceFromStep() + target.distance;
                       cout<<"RobotID:"<<robotStatus.ID<<", totalDist: "<<totalDist<<", target.distance: "<<target.distance<<endl;
                       if(totalDist>robotStatus.maxDistance)
                       {
                           cout<<"Exceed to Max travel Limit: "<<robotStatus.maxDistance<<endl;
                           return -1;
                       }
                       else if(totalDist<0)
                       {
                           cout<<"Exceed to Min Travel Limit: 0 !!!!"<<endl;
                           return -2;
                       }

                   }
               }
           }
           else target.seen =false;



           if(code_seen('V'))// Velocity
           {
               target.speed = code_value();
           }
           else target.seen =false;


           if(code_seen('A'))// Accelleration
           {
               target.accel = code_value();
               target.decel = target.accel;
           }
           else target.seen =false;

           if(target.seen)
           {
               genSpeedProfile(robotStatus, target, outP);

               sprintf(packet, "G%d M%d S%d A%d C%d D%d T%d O%d;", codeValue, target.motorID, outP.steps, outP.Na, outP.Nac, outP.NNb, outP.Cn_acc0, outP.dir);
              // qDebug()<<"packet"<<packet<<"size:"<<strlen(packet);
               return strlen(packet);
           }
       }
       ////////////////////////////////////
       // G10: Asking current absolute step position
       else //if(codeValue==10)
       {
            sprintf(packet, "G%d M%d;", codeValue, target.motorID);
            return strlen(packet);
       }

    }

    return 0;
}
speedProfile * MKVelProfile::processCmdInjectX(char * strCmd, double x, int n)
{
    //speedProfile outP;
    curIndex=n;
    strcpy(cmdbuffer, strCmd);
    targetDest target;
    if(code_seen('G'))
    {
       if( (int)code_value() == 17)
       {
           target.seen = true;
           if(code_seen('X'))
           {
               target.distance = x;
           }
           else target.seen =false;

           if(code_seen('F'))
           {
               target.speed = code_value();
           }
           else target.seen =false;


           if(code_seen('A'))
           {
               target.accel = code_value();
               target.decel = target.accel;
           }
           else target.seen =false;
       }

    }
    if(target.seen)
    {
        //double distance,  double speed, double accel, double decel,  speedProfile& outSpeedProfile
      //  genSpeedProfile(target,n);
      //  CMDData[n].selection = 10;// Linear Motion
        return &CMDData[n];
        //char packet[128];
        //sprintf(packet, "G17 S%d A%d C%d D%d T%d O%d;", CMDData.steps, CMDData.Na, CMDData.Nac, CMDData.NNb, CMDData.Cn_acc0, CMDData.dir);
        //G17 S[steps] A[Na] C[Nac] D[Nd] T[time0] O[dir]
        //qDebug()<<packet;

    }

    return NULL;
}
void MKVelProfile::genSpeedProfile(RobotStatus & robotStatus, targetDest &target, speedProfile& outSpeedProfile)
{


    double distance = target.distance;
    double speed =  target.speed*(2.0*M_PI/robotStatus.LEADPITCH);
    double accel = target.accel*(2.0*M_PI/robotStatus.LEADPITCH);//*0 + speed*1.0833;
    double decel = target.decel*(2.0*M_PI/robotStatus.LEADPITCH);
    if(robotStatus.jointType==JT_ROT)
    {
        //////////////////////////////
        // COVERT ALL UINT FROM DEGREE TO RADIAN
        //distance *= M_PI/180.0;
        speed *= M_PI/180.0;
        accel *= M_PI/180.0;
        decel *= M_PI/180.0;
    }
    // ...OUTPUT...
    // steps: total steps(pluse)
    // Na : pulse counts for acceleration area
    // Nac : pulse counts for acceleration and constant areas
    // NNb : pulse counts for deceleration area
    // Cn_acc0: fisrt period time for 1st pulse of acceleration on Timer.
    // dir: direction

//    if(distance<0){
//        outSpeedProfile.dir = 0;
//        distance*=-1.0;
//    }
//    else
//        outSpeedProfile.dir = 1;

    // dir: direction(1: +X, -1:-X);

    if(distance<0){
        distance*=-1.0;
        outSpeedProfile.dir = -1;
    }
    else {
        outSpeedProfile.dir = 1;

    }


  double steps = robotStatus.getStepsFromDistance(distance);// PULLY & BELT TYPE (20 TEETH, 5MM PITCH)
//  uint32_t steps = DIST2STEP_20TH2*distance;
  double  alpha = robotStatus.getAlpha();//  % [rad] angle per step
  double two_alpha = 2.0*alpha ;// alpha*2
  double Ttotal=0;

  // % 1. Calculate Time
  // % Ta = speed/acceleration
  double Ta = speed/accel; //%[sec]
  double Tb = speed/decel; //%[sec]
  double Tc = (steps*alpha - 0.5*accel*Ta*Ta - 0.5*decel*Tb*Tb)/speed;
  if(Tc>0)
  {
      Ttotal = Ta+Tc+Tb;
  }
  else
  {
      Ttotal = Ta+Tb;
  }


  // % 2. Calculate Step
  // % convert Time Zone to Stepper Motor Count
  // % Acceleration Zone
  // % n = speed^2/(2*alpha*accel)
  uint32_t Nb, Nc;
  outSpeedProfile.steps = floor(steps);

  outSpeedProfile.Na = floor(speed*speed/(two_alpha*accel));
  uint32_t Nacc = floor(steps*decel/(accel+decel));


  if(outSpeedProfile.Na<Nacc){
      //%Nb = floor(speed^2/(2*alpha*decel));
      Nb = floor(accel/decel*outSpeedProfile.Na);
      Nc = steps - (outSpeedProfile.Na+Nb);
  }
  else{
      outSpeedProfile.Na = Nacc;
      Nb = steps - Nacc;
      Nc = 0;
  }
  outSpeedProfile.Nac = outSpeedProfile.Na + Nc;
  outSpeedProfile.NNb = Nb;
  outSpeedProfile.Cn_acc0 = uint32_t(TICK_FREQ_SQRT_ERROR_COM*sqrt(two_alpha/accel));

  uint32_t Cn_const_speed = uint32_t(TICK_FREQ_SQRT_ERROR_COM*sqrt(two_alpha/accel)*(sqrt(outSpeedProfile.Na+1)-sqrt(outSpeedProfile.Na)));
  uint32_t Cn_dec0 = uint32_t(TICK_FREQ_SQRT_ERROR_COM*sqrt(two_alpha/decel));

    cout<<"genSpeedProfile()=> Total Time:"<< Ttotal<<", Alpha=:"<< alpha<<", Steps:"<<steps<<", Na:"<<outSpeedProfile.Na<<endl;
}
