#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

bool twiddle = false; //Set to "true" to twiddle, set to "false" to just run the simulator
int count = 0;  // For Twiddle - "time steps" counter
double tol = 0.2; //For Twiddle - parameter improvement threshold
double best_err = 0; //For Twiddle - keeping track of error between twiddle runs
bool is_init = false; //For the first twiddle run
int iteration = 0; // To track number of twiddle iterations
double err = 0; //To track error inside the twiddle loop and compare with best err
int parameter_flag = 0; //For twiddle, to switch between parameters
double dp = 0.1, di = 0.002, dd = 1.0; //Initial parameter deltas for twiddle
int twiddle_iteration = 0; //Parameter iterations inside the twiddle loop
double Kp = 0.29, Ki = 0.0022, Kd = 4.891; //Global variables to hold pid parameters

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.

  /*
  First I initialized the PID controller with parameters from Sebastian's class:
   * Kp = 0.2
   * Ki = 0.004
   * Kd = 3.0
   *
   * Twiddle with 200 "time steps" and 30 iterations converged around:
   *
   * Kp = 0.531
   * Ki = 0.002
   * Kd = 2.9
   *
   * Twiddle with 500 "time steps" and 50 iterations converged around:
   *
   * Kp = 0.29
   * Ki = 0.0022
   * Kd = 4.891
   *
   */

  pid.Init(Kp, Ki, Kd);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // ********* TWIDDLE *********
          if (twiddle) {

          //Running twiddle
          count += 1;
          //std::cout << "Count = " << count << std::endl;
          if (count > 500) { // Here I define number of "time steps" for each twiddle iteration
        	  //First run - initialize best_err and err
        	  if (!is_init){
        		  best_err = pid.total_error;
        		  err = best_err;
        		  is_init = true;
        		  //Reset the simulator
        		  std::string msg = "42[\"reset\",{}]";
        		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        		  count = 0;
        		  std::cout << "First run; best_err = " << best_err << std::endl;
        		  //Reset PID controller
        		  pid.Init(Kp, Ki, Kd);
        	  }
        	  // Run twiddle from the second run onwards
        	  else {
        		  err = pid.total_error;

        		  //If param improvement <= threshold, print out final results, stop executing the program
        		  //while sum(dp) > tol:

        		  //Twiddle to update parameters before the next run
        		  //***Code here***

        		  std::cout << "Best error = " << best_err << ", current error = " << err << std::endl;
        		  //std::cout << "Kp (pid) = " << pid.Kp << std::endl;
        		  //std::cout << "Ki (pid) = " << pid.Ki << std::endl;
        		  //std::cout << "Kd (pid) = " << pid.Kd << std::endl;

        		  //std::cout << "Kp (global) = " << Kp << std::endl;
        		  //std::cout << "Ki (global) = " << Ki << std::endl;
        		  //std::cout << "Kd (global) = " << Kd << std::endl;

        		  std::cout << " " << std::endl;
        		  std::cout << "***************" << std::endl;
        		  std::cout << "Iteration = " << iteration << std::endl;
        		  iteration += 1;

        		  // ******** Run twiddle for Kp parameter *********

        		  if (parameter_flag == 0){
        			  std::cout << "Twiddling Kp, parameter flag = " << parameter_flag << std::endl;
        			  //1st iteration: Increase by +=dp[i] and run the simulator
        			  if (twiddle_iteration == 0) {
        				  std::cout << "Twiddle iteration = " << twiddle_iteration << std::endl;
        				  Kp += dp;
        				  twiddle_iteration = 1;
        			  }

        			  //2nd iteration: Check if the error decreased
        			  else if (twiddle_iteration == 1) {
        				  std::cout << "Twiddle iteration = " << twiddle_iteration << std::endl;
        				  //If yes
        				  if (err < best_err) {
        					  //Increase dp[i]
        					  dp *= 1.1;
         					  //Update best_err value
        					  best_err = err;
        					  //Move on to next parameter
        					  parameter_flag = 1;
        					  //Reset twiddle iteration
        					  twiddle_iteration = 0;
        				  }
        				  // If not
        				  else {
        					  //2nd iteration: Deduct dp[i] and run the simulator
        					  Kp -= 2*dp;
        					  twiddle_iteration = 2;
        				  }
        			  }
        			  //3rd iteration: Check if error decreased
        			  else if (twiddle_iteration == 2){
        				  std::cout << "Twiddle iteration = " << twiddle_iteration << std::endl;
        				  //If yes
        				  if (err < best_err) {
        					  //Increase dp[i]
        				      dp *= 1.1;
        				      //Update best_err value
        				      best_err = err;
        				      //Move on to next parameter
        				      parameter_flag = 1;
        				      //Reset twiddle iteration
        				      twiddle_iteration = 0;
        				  }
        				  // If not
        				  else {
        				      //3rd iteration: go back to original parameter value
        				      Kp += dp;
        				      //Decrease dp[i]
        				      dp *= 0.9;
        				      //Reset twiddle iteration
        				      twiddle_iteration = 0;
        				      //Move on to next parameter
        				      parameter_flag = 1;
        				  }
        			  }
        		  }

        		  // ******** Run twiddle for Ki parameter *********

        		  else if (parameter_flag == 1){
        			  std::cout << "Twiddling Ki, parameter flag = " << parameter_flag << std::endl;
        			  //1st iteration: Increase by +=dp[i] and run the simulator
        			  if (twiddle_iteration == 0) {
        				  std::cout << "Twiddle iteration = " << twiddle_iteration << std::endl;
        				  Ki += di;
        				  twiddle_iteration = 1;
        			  }

        			  //2nd iteration: Check if the error decreased
        			  else if (twiddle_iteration == 1) {
        				  std::cout << "Twiddle iteration = " << twiddle_iteration << std::endl;
        				  //If yes
        				  if (err < best_err) {
        					  //Increase dp[i]
        					  di *= 1.1;
         					  //Update best_err value
        					  best_err = err;
        					  //Move on to next parameter
        					  parameter_flag = 2;
        					  //Reset twiddle iteration
        					  twiddle_iteration = 0;
        				  }
        				  // If not
        				  else {
        					  //2nd iteration: Deduct dp[i] and run the simulator
        					  Ki -= 2*di;
        					  twiddle_iteration = 2;
        				  }
        			  }
        			  //3rd iteration: Check if error decreased
        			  else if (twiddle_iteration == 2){
        				  std::cout << "Twiddle iteration = " << twiddle_iteration << std::endl;
        				  //If yes
        				  if (err < best_err) {
        					  //Increase dp[i]
        				      di *= 1.1;
        				      //Update best_err value
        				      best_err = err;
        				      //Move on to next parameter
        				      parameter_flag = 2;
        				      //Reset twiddle iteration
        				      twiddle_iteration = 0;
        				  }
        				  // If not
        				  else {
        				      //3rd iteration: go back to original parameter value
        				      Ki += di;
        				      //Decrease dp[i]
        				      di *= 0.9;
        				      //Reset twiddle iteration
        				      twiddle_iteration = 0;
        				      //Move on to next parameter
        				      parameter_flag = 2;
        				  }
        			  }
        		  }

        		  // ******** Run twiddle for Kd parameter *********

        		  else if (parameter_flag == 2){
        			  std::cout << "Twiddling Kd, parameter flag = " << parameter_flag << std::endl;
        			  //1st iteration: Increase by +=dp[i] and run the simulator
        			  if (twiddle_iteration == 0) {
        				  std::cout << "Twiddle iteration = " << twiddle_iteration << std::endl;
        				  Kd += dd;
        				  twiddle_iteration = 1;
        			  }

        			  //2nd iteration: Check if the error decreased
        			  else if (twiddle_iteration == 1) {
        				  std::cout << "Twiddle iteration = " << twiddle_iteration << std::endl;
        				  //If yes
        				  if (err < best_err) {
        					  //Increase dp[i]
        					  dd *= 1.1;
         					  //Update best_err value
        					  best_err = err;
        					  //Move on to next parameter
        					  parameter_flag = 0;
        					  //Reset twiddle iteration
        					  twiddle_iteration = 0;
        				  }
        				  // If not
        				  else {
        					  //2nd iteration: Deduct dp[i] and run the simulator
        					  Kd -= 2*dd;
        					  twiddle_iteration = 2;
        				  }
        			  }
        			  //3rd iteration: Check if error decreased
        			  else if (twiddle_iteration == 2){
        				  std::cout << "Twiddle iteration = " << twiddle_iteration << std::endl;
        				  //If yes
        				  if (err < best_err) {
        					  //Increase dp[i]
        				      dd *= 1.1;
        				      //Update best_err value
        				      best_err = err;
        				      //Move on to next parameter
        				      parameter_flag = 0;
        				      //Reset twiddle iteration
        				      twiddle_iteration = 0;
        				  }
        				  // If not
        				  else {
        				      //3rd iteration: go back to original parameter value
        				      Kd += dd;
        				      //Decrease dp[i]
        				      dd *= 0.9;
        				      //Reset twiddle iteration
        				      twiddle_iteration = 0;
        				      //Move on to next parameter
        				      parameter_flag = 0;
        				  }
        			  }
        		  }


				//My general twiddle flow in kind of pseudo code:
        		  //For current parameter:
        		  //1st iteration: Increase by +=dp[i] and run the simulator
        		  //2nd iteration: Check if the error decreased
        		  	  //If yes:
        		  	  	  //Increase dp[i]
        		  	  	  //best_err = err;
        		  	  	  //Move on to the next parameter
        		  	  //If not:
        		  	  	  //3rd iteration: Deduct dp[i] and run the simulator
        		  	  	  //4th iteration: Check if error decreased
        		  	  	  	  //If yes:
        		  	  	 	 	 //Increase dp[i]
        		  	  	  	  	 //best_err = err;
        		  	  	  	  	 //Move on to next parameter
        		  	  	  	  //If not:
        		  	  	  	  	 //Go back to original parameter value
        		  	  	  	  	  //Decrease dp[i]
        		  	  	  	  	  //Move on to next parameter

        		  //Re-initialize PID controller with updated parameters
        		  pid.Init(Kp, Ki, Kd);

        		  //Reset the simulator and start from scratch with updated parameters
        		  std::string msg = "42[\"reset\",{}]";
        		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        		  count = 0;
        	  } // end of "Run twiddle from the second run onwards"
          	 } // end of if (count > 200)
        	} //end of if (twiddle)

        } // end of if (event = telemetry)
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
