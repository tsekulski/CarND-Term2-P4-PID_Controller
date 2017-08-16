#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	/* Errors
	  */
	p_error = 0; //cte
	i_error = 0; //int_cte (integral over cte)
	d_error = 0; //diff_cte = cte - prev_cte

	/*
	* Coefficients
	*/
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;

}

void PID::UpdateError(double cte) {

	d_error = cte - p_error; //difference between current error and previous error
	p_error = cte; //update p_error to current error
	i_error += p_error; //increment total error
	/*
	* I guess it's ok to simply increment the error without taking the time into account,
	* under the assumption that time steps are equal.
	* If time steps are equal, then minimizing the error sum vs. error sum / delta t
	* will result in the same optimal parameter set
	*/

}

double PID::TotalError() {
	double steer_value;
	steer_value = -Kp * p_error - Kd * d_error - Ki * i_error;
	return steer_value;
}

