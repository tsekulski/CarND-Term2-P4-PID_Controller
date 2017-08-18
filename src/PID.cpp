#include "PID.h"
#include <math.h>       /* pow */
#include <iostream>

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
	total_error = 0; //for Twiddle

	/*
	* Coefficients
	*/
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;

	cout << "Initializing PID with:" << endl;
	cout << "Kp = " << Kp << endl;
	cout << "Ki = " << Ki << endl;
	cout << "Kd = " << Kd << endl;

}

void PID::UpdateError(double cte) {

	d_error = cte - p_error; //difference between current error and previous error
	p_error = cte; //update p_error to current error
	i_error += p_error; //increment error
	total_error += pow(cte, 2.0); // for Twiddle, MSE-like
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

void PID::Twiddle() {

//Initially I planned to implement the twiddle method here.
//But finally decided to do it directly in the main function
//- due to the fact that there is no easy way to call the simulator from within twiddle
//- I would need to rework the whole main.cpp anyway
//Here I'm just posting the "clean" Python implementation from the class for potential future reference:
/*
 *  # Don't forget to call `make_robot` before you call `run`!
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)
    # TODO: twiddle loop here
    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p, best_err
 *
 */

}
