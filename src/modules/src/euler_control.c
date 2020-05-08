/*
 * euler_control.c
 *
 *  Created on: Oct 10, 2019
 *      Author: commander
 */

//#include "euler_control.h"
#include "led.h"
#include "debug.h"
#include "stabilizer.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



#define M_PI_F ((float) M_PI)

extern radio_t radio;
extern control_tpos ctrlpos;

control_tv2 Econtrol;


void eulerControl(state_t *state, sensorData_t *sensor)
{
double phi, phid, ephi;//, up_phi;
double theta, thetad, etheta;//, up_theta;
double psi, psid, epsi;//, up_psi;
double kp=0.6, kd=0.175, ki=0.05;
double kpz=0.35, kdz=0.05, kiz=0.0, T=0.27;

double p,q,r;
double dephi, detheta, depsi;
double d[4],up[4];
double Iephi=0.0, Ietheta=0.0, Iepsi=0.0;

/*phid   = radio.Rps4*M_PI_F/180;
thetad = radio.Pps4*M_PI_F/180;
psid   = radio.Yps4*M_PI_F/180;
T      = radio.Tps4*0.002308235;//((radio.Tps4/2.55)*0.6*9.81)/1000;
*/

if (radio.Pps4*M_PI_F/180<0.349f)
	T      = radio.Tps4*0.002308235;
else
	T      = ctrlpos.thrust;

phid   = ctrlpos.phid;
thetad = ctrlpos.thetad;
psid   = -0.01311;

//T      = radio.Tps4*0.002308235;

phi   = state->attitude.roll*M_PI_F/180;
theta = state->attitude.pitch*M_PI_F/180;
psi   = state->attitude.yaw*M_PI_F/180;

p = sensor->gyro.x*M_PI_F/180;
q = -(sensor->gyro.y*M_PI_F/180); //inverted signed (maybe by construction)
r = sensor->gyro.z*M_PI_F/180;

//Errors
ephi   = phid - phi;
etheta = thetad - theta;
epsi   = psid - psi;

dephi   = -p;
detheta = -q;
depsi   = -r;

Iephi   = ephi + Iephi;
Ietheta = etheta + Ietheta;
Iepsi   = epsi + Iepsi;

/*up_phi   = kp*ephi + kd*dephi;
up_theta = kp*etheta + kd*detheta;
up_psi   = kp*epsi +  kd*depsi;*/

up[0] = (kp*ephi + kd*dephi + ki*Iephi);
up[1] = (kp*etheta + kd*detheta + ki*Ietheta);
up[2] = (kpz*epsi + kdz*depsi + kiz*Iepsi);
up[3] = T;//0.275;//ctrlpos.thrust; //Total thrust (27 gr)

for(int i=0; i<4; i++)
	d[i] = up[i];


//T = 0.1; // N
//d = (0.0962*sqrt(416*T+49)-0.6731)*65536;

//DEBUG_PRINT("T = %f \n", ctrlpos.thrust);

/*Econtrol.roll   = up_phi;
Econtrol.pitch  = up_theta;
Econtrol.yaw    = up_psi;
Econtrol.thrust = (int)d;*/

Econtrol.roll   = d[0];
Econtrol.pitch  = d[1];
Econtrol.yaw    = d[2];
Econtrol.thrust = d[3];

ledSet(2, 0); //M4 red
ledSet(1, 1); //M4 green

}

