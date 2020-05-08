#include <math.h>
#include "reactive_control.h"
#include "stabilizer.h"
#include "debug.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define M_PI_F ((float) M_PI)
#define DEG2RAD M_PI/180;
extern radio_t radio;
extern control_tpos ctrlpos;
control_tv2 Econtrol;

void reactive_control(state_t *state, sensorData_t *sensor)
{
    double pos_d[3] = {1,2,0.7};
    double g = 9.81, m = 0.0345;
    double kp=1,kd=1.8,kw=1e-3,kwz=0.1;
    double D2 = 0.1, alpha = 1e-2;
    double f_norm,T;
    double x_e,y_e,z_e,f_nx,f_ny,f_nz;
    double tilde_f_nx,tilde_f_ny,tilde_f_nz;
    double tau_x,tau_y,tau_z;

    double pos_x = state->position.x;
    double pos_y = state->position.y;
    double pos_z = state->position.z;
    double vel_nx = state->velocity.x;
    double vel_ny = state->velocity.y;
    double vel_nz = state->velocity.z;
    double q0 = state->attitudeQuaternion.w;
    double q1 = state->attitudeQuaternion.x;
    double q2 = state->attitudeQuaternion.y;
    double q3 = state->attitudeQuaternion.z;
    double p = sensor->gyro.x*DEG2RAD;
    double q = -sensor->gyro.y*DEG2RAD;
    double r = sensor->gyro.z*DEG2RAD;

    x_e = pos_d[0]-pos_x;
    y_e = pos_d[1]-pos_y;
    z_e = pos_d[2]-pos_z;

    //double qvv = q1*vel_bx+q2*vel_by+q3*vel_bz;
    //double qxv_x = vel_bz*q2-q3*vel_by;
    //double qxv_y = -(vel_bz*q1-q3*vel_bx);
    //double qxv_z = vel_by*q1-q2*vel_bx;

    //double vel_nx = (qvv*q1 + q0*q0*vel_bx + 2*q0*qxv_x + (qxv_z*q2 - q3*qxv_y));
	//double vel_ny = (qvv*q2 + q0*q0*vel_by + 2*q0*qxv_y - (qxv_z*q1 - q3*qxv_x));
	//double vel_nz = (qvv*q3 + q0*q0*vel_bz + 2*q0*qxv_z + (qxv_y*q1 - q2*qxv_x));

    //f_n = m*((1+kp*kd)*pos_e-(kp+kd)*vel+g*e3);
    f_nx = m*((1+kp*kd)*x_e-(kp+kd)*vel_nx);
    f_ny = m*((1+kp*kd)*y_e-(kp+kd)*vel_ny);
    f_nz = m*((1+kp*kd)*z_e-(kp+kd)*vel_nz+g);

    //T = norm(f_n);
    f_norm = f_nx*f_nx+f_ny*f_ny+f_nz*f_nz;
    T = sqrt(f_norm);

    //tilde_f = T > alpha ? alpha*f_n/sqrt(f_n*f_n+D) : f_n;
    if(T>alpha){
        double fac = alpha/sqrt(f_norm+D2);
        tilde_f_nx = fac*f_nx;
        tilde_f_ny = fac*f_ny;
        tilde_f_nz = fac*f_nz;
    }else{
        tilde_f_nx = f_nx;
        tilde_f_ny = f_ny;
        tilde_f_nz = f_nz;
    }

    //tilde_f_b = qrot(qconj(quat),tilde_f_n)
    double qvf = q1*tilde_f_nx+q2*tilde_f_ny+q3*tilde_f_nz;
    double fxq_x = tilde_f_ny*q3-q2*tilde_f_nz;
    double fxq_y = -(tilde_f_nx*q3-q1*tilde_f_nz);
    double fxq_z = tilde_f_nx*q2-q1*tilde_f_ny;

    double tilde_f_b_x = (qvf*q1 + q0*q0*tilde_f_nx + 2*q0*fxq_x + (fxq_y*q3 - q2*fxq_z));
	double tilde_f_b_y = (qvf*q2 + q0*q0*tilde_f_ny + 2*q0*fxq_y - (fxq_x*q3 - q1*fxq_z));
	//double f_b_z = (qvf*q3 + q0*q0*tilde_f_nz + 2*q0*fxq_z + (fxq_x*q2 - q1*fxq_y));

	//tau = -cross([0 0 1]',tilde_f_b)-kw*w;
    tau_x = -tilde_f_b_y - kw*p;
    tau_y =  tilde_f_b_x - kw*q;
    tau_z =              - kw*r;

    //DEBUG_PRINT("taux = %5.5f, tauy = %5.5f\n",tau_x, tau_y);

    Econtrol.roll = tau_x;
	Econtrol.pitch = tau_y;
	Econtrol.yaw = tau_z;
	if (radio.Pps4*M_PI_F/180<0.349)
		Econtrol.thrust      = radio.Tps4*0.002308235;
	else
		Econtrol.thrust      = T;
}
