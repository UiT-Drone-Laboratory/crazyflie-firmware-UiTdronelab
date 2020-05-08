/*
 * position_control.c
 *
 *  Created on: Dec 12, 2019
 *      Author: commander
 */

#include "led.h"
#include "debug.h"
#include "stabilizer.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define M_PI_F ((float) M_PI)

control_tpos ctrlpos;

double fg, m=0.027, g=9.81;
double ex=0.0, ey=0.0, ez=0.0;
double x=0.0, y=0.0, z=0.0;
double dx=0.0, dy=0.0, dz=0.0;
double xp=0.0,yp=0.0,zp=0.0;
double phid=0.0, thetad=0.0, fT;
double kp=0.43, kd=0.4, kpz=0.29, kdz=0.095;
double xd=1.0, yd=2.0, zd=0.7;

void positionControl(double posq[],double velq[])
{

fg = m*g;

xp = x;
yp = y;
zp = z;

if(posq[1]==yp)
	x = xp;
else{
x = posq[0];
y = posq[1];
z = posq[2];}

ex = xd - x;
ey = yd - y;
ez = zd - z;

//Velocities
dx = velq[0];
dy = velq[1];
dz = velq[2];

phid   = -tanh(kp*ey) + tanh(kd*dy);
thetad = -tanh(kp*ex) + tanh(kd*dx);
fT     = fg + kpz*ez - kdz*dz;

//DEBUG_PRINT("dx=%f \n", dx);

ctrlpos.phid   = phid;
ctrlpos.thetad = thetad;
ctrlpos.thrust = fT;


}
