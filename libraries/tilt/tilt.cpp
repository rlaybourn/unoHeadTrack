#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "tilt.h"

tilt::tilt()
{
Wire.begin();

//init_accel();
//Init_hmc();

  K = 16384;
  Kg = 131.1;
  xcomp = 0.0;
  ycomp = 400.0;
  zcomp = -400.0;
  gxcomp = 420;
  gycomp = -85;
  gzcomp = 100;
  //sumx = sumy = sumz = 0;

  yaw = 1;
  yaw1 = yaw2 = 1;
  kalz = 0;
  kalx = 0;
  kaly = 0;
  gzeroPoint = zeroPoint;


}

void tilt::init_accel()
{
  int i;
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // write to power management register to wake up
  Wire.write(0);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); // write to low pass filter
  Wire.write(4); //3=44 HZ (2= 98  1= 260 4 = 21)
  Wire.endTransmission(true);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x37); // enable pass through to compass
  Wire.write(2); //
  Wire.endTransmission(true);
  for(i =0;i <10; i++)
  {
	gyxb[i] = 0;
	gyyb[i] = 0;
	gyzb[i] = 0;
	acxb[i] = 0;
	acyb[i] = 0;
	aczb[i] = 0;
  }
  
}

void tilt::read_accel_data(float dt)
{
  int i;
  for(i =0;i <(AVERAGING - 1);i++)
  {
	gyxb[i] = gyxb[i+1];
	gyyb[i] = gyyb[i+1];
	gyzb[i] = gyzb[i+1];
	acxb[i] = acxb[i+1];
	acyb[i] = acyb[i+1];
	aczb[i] = aczb[i+1];
  }
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //start reading at 0x3B
  Wire.endTransmission(false); //end get ready to recieve
  Wire.requestFrom(0x68,14,true); //read 14 values
  acx= Wire.read()<<8 | Wire.read();
  acy= Wire.read()<<8 | Wire.read();
  acz= Wire.read()<<8 | Wire.read();
  tmp= Wire.read()<<8 | Wire.read();
  gyx= Wire.read()<<8 | Wire.read();
  gyy= Wire.read()<<8 | Wire.read();
  gyz= Wire.read()<<8 | Wire.read();


  Wire.endTransmission(true);
  double tempx,tempy,tempz;
  lastacxf = acxf;
  lastacyf = acyf;
  lastaczf = aczf;

  acxf = (acx + xcomp) /K; //converted accel readings
  acyf = (acy + ycomp) /K;
  aczf = (acz + zcomp) /K;



  acxb[(AVERAGING - 1)] = acxf;
  acyb[(AVERAGING - 1)] = acyf;
  aczb[(AVERAGING - 1)] = aczf;

  // for(int i =0;i < AVERAGING;i++)
  // {
  //   acxbM[i] = acxb[i];
  //   acybM[i] = acyb[i];
  //   aczbM[i] = aczb[i];
  // }

  //sort(acxbM,AVERAGING);
  //sort(acybM,AVERAGING);
  //sort(aczbM,AVERAGING);

  gxf = ((gyx + gxcomp) /Kg) * (dt*0.001);//dtime;
  gyf = ((gyy + gycomp) /Kg) *-(dt*0.001);//dtime; //gyro goes opposite way *shrug*
  gzf = ((gyz + gzcomp) /Kg) * -(dt * 0.001);
  gyxb[(AVERAGING - 1)] = gxf;
  gyyb[(AVERAGING - 1)] = gyf;
  gyzb[(AVERAGING - 1)] = gzf;


 



  for(i = 0;i < (AVERAGING - 1);i++)
  {
  	gxf += gyxb[i];
  	gyf += gyyb[i];
  	gzf += gyzb[i];
  	acxf += acxb[i];
  	acyf += acyb[i];
  	aczf += aczb[i];
  }
  gxf = gxf /AVERAGING;
  gyf = gyf/AVERAGING;
  gzf = gzf/AVERAGING;
  acxf = acxf/AVERAGING;
  acyf = acyf/AVERAGING;
  aczf = aczf/AVERAGING;
  // acxf = (acxbM[10]+acxbM[11]+acxbM[12])/3;
  // acyf = (acybM[10]+acybM[11]+acybM[12])/3;
  // aczf = (aczbM[10]+aczbM[11]+aczbM[12])/3;
  
}



void tilt::syncup()
{

  // read_accel_data(0.005);
  
  // magx = read_hmc(mx); // get mag data
  // magy = read_hmc(my);
  // magz = read_hmc(mz);
  
  // magx = magx + 180;
  // magy = magy -165;
  
  // kalx = atan(acyf/sqrt(pow(acxf,2)+pow(aczf,2)))*57.296;
  // kaly = atan(acxf/sqrt(pow(acyf,2)+pow(aczf,2)))*57.296;
  // kalz = atan2((double)magy,(double)magx) * 57.296;
  //Serial.println("start cal");

  gzeroPoint = get_yaw(false);
  Serial.println(gzeroPoint);
  //kalz = 0;
  
  
}
void tilt::syncoff()
{
  long int accumulator = 0;
  for(int i =0; i < 100; i++)
  {
          read_accel_data(10);        
          accumulator += gyz; 
    //      Serial.println(gyz);
  }
  //Serial.print("new comp value is: ");
  //Serial.println(gzcomp);
  gzcomp = -1 * (accumulator / 100);
     
 }

void tilt::Init_hmc(void)
{
  /* Set the module to 8x averaging and 15Hz measurement rate */
   Wire.beginTransmission(HMC);
   Wire.write(0x00);
   Wire.write(0x70);
         
    //Set a gain of 1 
   Wire.write(0x01);
   Wire.write(0x20);
   Wire.endTransmission();

}

int tilt::read_hmc(byte axis)
{
 int Result;
 
   /* Initiate a single measurement */
   Wire.beginTransmission(HMC);
   Wire.write(0x02);
   Wire.write(0x01);
   Wire.endTransmission();
   delay(6);

 
   /* Move modules the resiger pointer to one of the axis data registers */
   Wire.beginTransmission(HMC);
   Wire.write(axis);
   Wire.endTransmission();
   
   /* Read the data from registers (there are two 8 bit registers for each axis) */ 
   Wire.requestFrom(HMC, 2);
   Result = Wire.read() << 8;
   Result |= Wire.read();


  return Result; 
}

double tilt::get_un_yaw()
{
  magx = read_hmc(mx); // get mag data
  magy = read_hmc(my);
  magz = read_hmc(mz);
  double magxF = magx + 180.0;
  double magyF = magy -165.0;
  double magzF = magz + 225.0;
  double res = atan2(magyF,magxF)*57.29578;
  if (res < 0)
      res +=360;
  return res;
       
}

double tilt::get_yaw(bool comp)
{
  double compxtmp,compytmp,compztmp,compx,compy,compz,res;
  double sinpitch,sinroll,cospitch,cosroll;
  magx = read_hmc(mx); // get mag data
  magy = read_hmc(my);
  magz = read_hmc(mz);
  read_accel_data(dt); //get accel and gyro data

 
  pitchAcc = atan(acxf/sqrt(pow(acyf,2)+pow(aczf,2)))*57.29578;
  rollAcc = atan(acyf/sqrt(pow(acxf,2)+pow(aczf,2)))*57.29578;
  
  double magxF = magx + 180.0;
  double magyF = magy -165.0;
  double magzF = magz + 225.0;
  sinpitch = sin((-1*get_acc_pitch())/57.29578);                 //compensation values remove if problem
  sinroll = sin((1 * get_acc_roll())/57.29578);
  cospitch = cos((-1 * get_acc_pitch())/57.29578);   //undo "-" on rollAcc
  cosroll = cos((1 * get_acc_roll())/57.29578);


 compx = (magx * cospitch) + (magy * sinroll*sinpitch) + (magz * cosroll * sinpitch);
 compy = (magy * cosroll) - (magz*sinroll);

  res = atan2(compy,compx)*57.29578;
  
  double lsum = 0;

  if (res < 0)
      res +=360;
  if(comp)
  {
   res = res - gzeroPoint;
   }
   else
   {
       return res;
   }
  res = (res * 0.1) + (yawFilter *0.9);    
  lastYaw = res;//atan2(compy,compx)*57.29578;
  yawFilter = res;
  return res;

}


double tilt::kalman(double angle, double gyro, double accData)
{
  double temp;
  temp = (0.9*(angle + gyro )) + (0.1 * accData);
  return temp;
}

void tilt::update_kalman(int dt)
{

 
 kalx = kalman(kalx,gxf,rollAcc);
 
 kaly = kalman(kaly,gyf,pitchAcc);
 //kaly = (0.7 * kaly) + (0.3 * lastPitch);
 

 //if((abs(get_pitch() < 1.0)) && (abs(get_roll() < 1.0)))
 //{
 //   kalz = (0.9 * (kalz + gzf)) + (0.1 * get_yaw());
 //}
 //else
 //{
 double ltempAngle = kalz + (gzf *3.5 );
// if(ltempAngle > 360.0)
// {
//               ltempAngle -= 360;
// }
// else if(ltempAngle < 0.0 )
// {
//      ltempAngle += 360;
//  }

   //kalz = ltempAngle;//(0.999 * (ltempAngle)) + (0.001 * lastYaw);
   if(abs(pitchAcc) < 5.0)
   {
               kalz = (0.98 * (ltempAngle)) + (0.02 * lastYaw);
               }
   else if(abs(pitchAcc) < 10.0)
   {
       //kalz = ltempAngle;
       kalz = (0.995 * (ltempAngle)) + (0.005 * lastYaw);
   }
   else
   {
       kalz = ltempAngle;
   }
   kalz = (0.9 * kalz) + (0.1 * lastPitch);
   lastPitch = kalz;
 //}
  
 if(kalx != kalx)
    kalx = 0.0;
  if(kaly != kaly)
    kaly = 0.0;
  if(kalz != kalz)
    kalz = 0.0;
  
}

void tilt::update_global(float dt, float zero)
{
 double yaw_temp,yaw_comp;
  read_accel_data(dt); //get accel and gyro data

 
  pitchAcc = atan(acxf/sqrt(pow(acyf,2)+pow(aczf,2)))*57.29578;
  rollAcc = atan(acyf/sqrt(pow(acxf,2)+pow(aczf,2)))*57.29578;
  
//  if((abs(pitchAcc) < 3.0 ) && (abs(rollAcc) < 3.0 ))
//  {
//                    kalz = get_yaw();
//                    }


 update_kalman(0);

}
void tilt::sort(double pDataArr[], size_t pSize)
{
int i;   
int j;   
double temp;
for( i = pSize-1;i > 0; i--)
{
    for(j = 0;j < i;j++)
    {
      if(pDataArr[j] > pDataArr[i])
      {
        temp = pDataArr[j];
        pDataArr[j] = pDataArr[i];
        pDataArr[i] = temp;
      }
    }

}   
  
}

double tilt::get_pitch()
{
return kaly;
}

double tilt::get_roll()
{
return kalx;
}
double tilt::get_yaws()
{
return kalz;
}
double tilt::get_acc_pitch()
{
  return pitchAcc;
}
double tilt::get_acc_roll()
{
  return  rollAcc;
}
double tilt::get_roll_gyro()
{
  return gyz +gzcomp;//gzf;
}
