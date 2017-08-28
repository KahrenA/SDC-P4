#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

//***********************************************
void PID::Init(double Kp, double Ki, double Kd)
{
	// We could call Twiddle or assume some values 
	// Twiddle is mainly dependent on steering drift
	// For Twiddle we need a routine that calculates steering_angle and returns err
	// for further twiddle calculations 
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

//	cout << " Kp = " << Kp << "\t" << "Ki = " << Ki << "\t" << "Kd = " << Kd << "\n";
	
}

//**********************************************
void PID::UpdateError(double cte) 
{
	// differential error is the diff between current and previous cte 
	d_error = cte - p_error; 

	// Update p_error for next time
	p_error= cte;	

	// do the integral 			
	i_error += cte;

//	cout << "p_error = " << p_error << "\t" << "i_error = " << i_error << "\t" << "d_error = " << d_error << "\n";
//	cout << p_error << "\t" << i_error << "\t" << d_error << "\t";

 }

//*********************************************
double PID::TotalError(double speed, double angle)
{
double error, new_steering_angle;
double p_term, i_term, d_term;

	p_term = Kp * p_error;
	i_term = Ki * i_error;

	if (speed > 1.0)
	{
//		d_term = Kd * d_error/speed;
		d_term = Kd * d_error;
		error = -(p_term) - (i_term) - (d_term);
//		cout << "p-term = " << p_term << "\t" << "i_term = " << i_term << "\t" 
//										<< "d-term = " << d_term << "\n";
//		cout << p_term << "\t" <<  i_term << "\t" << d_term << "\t";
	}
	else
	{ 
		error = -(p_term) - (i_term);
//		cout << "p-term = " << p_term << "\t" << "i_term = " << i_term <<  "\n";
	}

	if (error > 1) 
		error = 1;
	else if(error < -1)
		error = -1;

	new_steering_angle = error;
	return new_steering_angle;
}

