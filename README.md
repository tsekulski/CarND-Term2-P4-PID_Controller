# CarND-Term2-P4-PID_Controller

_Technologies: PID control, C++_

Implemented a PID controller in C++ to maneuver a vehicle around a track.

_Part of the Self-Driving Car Engineer Nanodegree Program_

# Reflection

## 1. Parameter values

In my implementation I settled for the following parameters:
* Kp = 0.29
* Ki = 0.0022
* Kd = 4.891

Interestingly, the Kd parameter (differential control parameter) has the highest value, i.e. the steering reacts most strongly to the difference between the previous and the current cross-track errors (CTEs). The Kd value is some 17 times higher than the Kp value (where Kp represents the proportional reaction of the steering to the current CTE value). The way I understand it works is following:
* When the CTE improvements (i.e. difference between current and previous CTEs) are "small" comparing to overall CTE (i.e. the car is far away from the target), then the proportional steering has higher weight and makes sure the car steers "towards" the target.
* At the moment when the current CTE is around 17 times higher than the difference between current and previous CTE, both proportional and differential components neutralize each other, i.e. the car drives in a straight line towards the target.
* Afterwards, when CTE improvements are higher than "current CTE / 17", the differential component has more weight, i.e. the car starts countersteering in order to avoid overshooting (which would happen if it continued in a straight line)

The Ki value is very small, which suggests that there is barely any systematic bias built into the steering.

## 2. Parameter optimization

In order to arrive at the above parameters I implemented the twiddle algorithm. The implementation is coded primarily in the main.cpp with only some minor adaptations in PID.cpp.

I ran two twiddle cycles, each time starting with parameters from Sebastian's class:
* Kp = 0.2
* Ki = 0.004
* Kd = 3.0

In the first cycle, I was resetting the simulator after 200 "time steps" (i.e. at the beginning of the first left turn) and I let it run for 30 iterations. In that cycle the parameter set that yielded the best total error was following:
* Kp = 0.531
* Ki = 0.002
* Kd = 2.9

In the second cycle, I was resetting the simulator after 500 "time steps" (i.e. after the bridge) and I let it run for 50 iterations. In that cycle the parameter set that yielded the best total error was following:
* Kp = 0.29
* Ki = 0.0022
* Kd = 4.891

The logs from each of the cycles can be found in the attached files:
### twiddle_logs_200_time_steps_30_iterations
### twiddle_logs_500_time_steps_50_iterations

I could run twiddle with more time steps (e.g. entire track) and more iterations, but it was very time-consuming to wait for all the iterations to run. Since my car was already completing the track without crashing, I decided not to further optimize the parameters and settled for the ones from the second twiddle cycle.

Overall, I have mixed feelings about twiddle as a parameter optimization algorithm. The car still oscillates quite strongly during turns. The reason for that seems to be that twiddle optimizes the total error sum, i.e. it doesn't seem to actually directly aim at reducing oscillation, especially in "special situations" like turns. On the other hand I did not let it optimize on the entire track and over a large number of iterations - maybe it would yield better results then.
