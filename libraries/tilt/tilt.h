
#include "Arduino.h"
#include <math.h>


#ifndef TILT_H
#define TILT_H

#define HMC 0x1E
#define mx 0x03
#define my 0x07
#define mz 0x05
#define dtime 0.005
#define AVERAGING 20
#define zeroPoint 260

class tilt

{




public:
tilt();
double kalman(double angle, double gyro, double accData);
void update_kalman(int dt);
double get_yaw(bool comp);
int read_hmc(byte axis);
void Init_hmc(void);
void syncup();
void syncoff();
void read_accel_data(float dt);
void init_accel();
void update_global(float dt,float zero);
double get_pitch();
double get_roll();
double get_yaws();
double get_un_yaw();
double get_acc_pitch();
double get_acc_roll();
double get_roll_gyro();

private:
int16_t acx,acy,acz,tmp,gyx,gyy,gyz;
double gyxb[AVERAGING],gyyb[AVERAGING],gyzb[AVERAGING],acxb[AVERAGING],acyb[AVERAGING],aczb[AVERAGING],acxbM[AVERAGING],acybM[AVERAGING],aczbM[AVERAGING];
double K,Kg,pitchAcc,rollAcc,yaw,yaw1,yaw2;
double xcomp,ycomp,zcomp,acxf,acyf,aczf,gxf,gyf,gzf;
double gxcomp,gycomp,gzcomp,kalx,kaly,kalz;
double lastacxf,lastacyf,lastaczf;
double lastYaw;
double lastPitch;
double yawFilter;
double gzeroPoint;
//double kalx,kaly,kalz;
int magx,magy,magz;
float dt;
void sort(double pDataArr[], size_t pSize);






};
#endif
