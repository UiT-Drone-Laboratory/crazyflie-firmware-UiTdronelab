/*
 * quaternion_control.c
 *
 *  Created on: Oct 21, 2019
 *      Author: commander
 */


//#include "euler_control.h"
#include "led.h"
#include "debug.h"
#include "stabilizer.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



#define M_PI_F ((float) M_PI)

extern radio_t radio;
extern control_tpos ctrlpos;

control_tv2 Econtrol;


void quaternionControl(state_t *state, sensorData_t *sensor)
{
double phid=0.0;// phid;//, up_phi;
double thetad=0.0;// thetad;//, up_theta;
double psid=0.0;//, psid;//, up_psi;
double kp=0.6, kd=0.175;
double kpz=0.25, kdz=0.05, T=0.27;
double up[4]={1,0,0,0};

double qe[4]={1,0,0,0}, qdc[4]={1,0,0,0}, qb[4]={1,0,0,0}, we[3]={0,0,0};
double p=0.0, q=0.0, r=0.0, cy=0.0,sy=0.0,cp=0.0,sp=0.0,cr=0.0,sr=0.0;

//Quaternion desired
//phid   = -(radio.Rps4*M_PI_F/180); // Investigate why the inverse sign
//thetad = -(radio.Pps4*M_PI_F/180);
//psid   = radio.Yps4*M_PI_F/180;

phid   = -ctrlpos.phid;
thetad = -ctrlpos.thetad;
psid   = -0.01311;

if ((double)radio.Pps4*M_PI/180<(double)0.349)
	T      = radio.Tps4*0.002308235;
else
	T      = ctrlpos.thrust;

//T      = radio.Tps4*0.002308235;//((radio.Tps4/2.55)*0.6*9.81)/1000;

cy = cos(psid * 0.5);
sy = sin(psid * 0.5);
cp = cos(thetad * 0.5);
sp = sin(thetad * 0.5);
cr = cos(phid * 0.5);
sr = sin(phid * 0.5);

qdc[0] =  (cy*cp*cr + sy*sp*sr);
qdc[1] = -(cy*cp*sr - sy*sp*cr);
qdc[2] = -(sy*cp*sr + cy*sp*cr);
qdc[3] = -(sy*cp*cr - cy*sp*sr);

/*qdc[0] = 1;
qdc[1] = 0;
qdc[2] = 0;
qdc[3] = 0;*/

//State
p = sensor->gyro.x*M_PI_F/180;
q = -(sensor->gyro.y*M_PI_F/180); //inverted signed (maybe by construction)
r = sensor->gyro.z*M_PI_F/180;

qb[0] = state->attitudeQuaternion.w;
qb[1] = state->attitudeQuaternion.x;
qb[2] = -(state->attitudeQuaternion.y);
qb[3] = state->attitudeQuaternion.z;

//Errors
qe[0] = qdc[0]*qb[0] - qdc[1]*qb[1] + qdc[2]*qb[2] + qdc[3]*qb[3];
qe[1] = qdc[0]*qb[1] - qb[0]*qdc[1] + qdc[2]*qb[3] - qdc[3]*qb[2];
qe[2] = qdc[0]*qb[2] - qb[0]*qdc[2] + qdc[3]*qb[1] - qdc[1]*qb[3];
qe[3] = qdc[0]*qb[3] - qb[0]*qdc[3] + qdc[1]*qb[2] - qdc[2]*qb[1];
//qe = qmul(qdc, qb);

we[0] = -p;
we[1] = -q;
we[2] = -r;

up[0] = (-kp*qe[1] + kd*we[0]);
up[1] = (-kp*qe[2] + kd*we[1]);
up[2] = (-kpz*qe[3] + kdz*we[2]);
up[3] = T; //Total thrust (27 gr)

Econtrol.roll   = up[0];
Econtrol.pitch  = up[1];
Econtrol.yaw    = up[2];
Econtrol.thrust = up[3];

ledSet(2, 0); //M4 red
ledSet(1, 1); //M4 green

//DEBUG_PRINT("qe = %f \n", qe[0]);


}

/*double qmul(double *q, double *r)
{
double tp[4];

tp[0] = q[0]*r[0] - q[1]*r[1] + q[2]*r[2] + q[3]*r[3];
tp[1] = q[0]*r[1] - r[0]*q[1] + q[2]*r[3] - q[3]*r[2];
tp[2] = q[0]*r[2] - r[0]*q[2] + q[3]*r[1] - q[1]*r[3];
tp[3] = q[0]*r[3] - r[0]*q[3] + q[1]*r[2] - q[2]*r[1];

return &tp;
}*/
