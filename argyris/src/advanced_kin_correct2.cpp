/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Example ROS subscription to Phidgets advanced servo controller
 *  Copyright (c) 2010, Bob Mottram
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <stdio.h>
#include <ros/ros.h>
#include <argyris/servo_reference.h>
#include <argyris/servo_params.h>
#include <math.h>

#define PHIDGETS_ADVANCED_SERVO_MOTORS 8

ros::ServiceClient client_servo_reference;

/*!
 * \brief callback when a servo has changed position
 * \param servo parameters
 */
void servoCallback(const argyris::servo_params::ConstPtr& ptr)
{
	argyris::servo_params s = *ptr;
	ROS_INFO("Servo %d position %.2f", s.index, s.position);
}

/*!
 * \brief set the position, speed and acceleration for a servo
 * \param index servo index
 * \param engage whether to energise the servo
 * \param position reference position for the servo
 * \param speed reference speed for the servo
 * \param acceleration reference acceleration for the servo
 */
void set_servo_reference(
						 int index,
						 bool engage,
						 float position,
						 float speed,
						 float acceleration)
{
	argyris::servo_reference srv;
	srv.request.index = index;
	srv.request.engage = engage;
	srv.request.position = position;
	srv.request.speed = speed;
	srv.request.acceleration = acceleration;
	srv.response.ack = 0;
	if (client_servo_reference.call(srv)) {
		if ((int)srv.response.ack == 1) {
			ROS_INFO("Changed servo %d reference %.2f",
					 index, position);
		}
		else {
			ROS_INFO("Returned %d", (int)srv.response.ack);
		}
	}
	else {
		ROS_ERROR("Failed to call service servo_reference");
	}
}

/*!
 * \brief energises all servos and sends them
 *        to their neutral position
 * \param neutral_position servo position
 * \param speed servo speed
 * \param acceleration servo acceleration
 */
void energise(
			  float neutral_position,
			  float speed,
			  float acceleration)
{
    for (int servo_index = 0;
		 servo_index < PHIDGETS_ADVANCED_SERVO_MOTORS;
		 servo_index++) {
        set_servo_reference(servo_index, true,
							neutral_position, speed,
							acceleration);
    }
}

/*!
 * \brief deactivates all servos
 */
void disengage()
{
    for (int servo_index = 0;
		 servo_index < PHIDGETS_ADVANCED_SERVO_MOTORS;
		 servo_index++) {
        set_servo_reference(servo_index, false, 0, 0, 0);
    }
}

 // robot geometry
 // (look at pics above for explanation)
 const float e = 140;     // end effector
 const float f = 200;     // base
 const float re = 105;
 const float rf = 70;
 
 // trigonometric constants
 const float sqrt3 = sqrt(3.0);
 const float pi = 3.141592653;    // PI
 const float sin120 = sqrt3/2.0;   
 const float cos120 = -0.5;        
 const float tan60 = sqrt3;
 const float sin30 = 0.5;
 const float tan30 = 1/sqrt3;

 // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
     float t = (f-e)*tan30/2;
     float dtr = pi/(float)180.0;
 
     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;
 
     float y1 = -(t + rf*cos(theta1));
     float z1 = rf*sin(theta1);
 
     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = rf*sin(theta2);
 
     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = rf*sin(theta3);
 
     float dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point
 
     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;
     return 0;
 }
 
 // inverse kinematicss
 // helper functions, calculates angle theta1 (for YZ-pane)
 int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
     float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
     float e0 = 0.5 * 0.57735 * e;    // shift center to edge
     // z = a + b*y
     float k;
     k = y0 - e0;
     float a = (x0*x0 + k*k + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
     float b = (y1-k)/z0;
     // discriminant
     float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
     if (d < 0) return -1; // non-existing point
     float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
     float zj = a + b*yj;
     //theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
     theta = 180.0*(asin(zj/rf))/pi;
     return 0;
 }
 
 // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
     theta1 = theta2 = theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);
     if (status == 0) status = delta_calcAngleYZ(-x0*0.5 + y0*sqrt3/2, -y0*0.5-x0*sqrt3/2, z0, theta2);  // rotate coords to +120 deg
     if (status == 0) status = delta_calcAngleYZ(-x0*0.5 - y0*sqrt3/2, -y0*0.5+x0*sqrt3/2, z0, theta3);  // rotate coords to -120 deg

     ROS_INFO("theta1 = %f", theta1);
     ROS_INFO("theta2 = %f", theta2);
     ROS_INFO("theta3 = %f", theta3);

     return status;
 }




int main(int argc, char** argv)
{
	ros::init(argc, argv, "advanced_servo_client");
	ros::NodeHandle n;
	ros::Subscriber servo_sub =
		n.subscribe("phidgets/servos", 1, servoCallback);
	client_servo_reference =
		n.serviceClient<argyris::servo_reference>("servo_reference");
	float x0, y0, z0;

	
	float theta1 = 0;
	float theta2 = 0;
	float theta3 = 0;

	float moires;
	ros::Rate loop_rate(30);
	while(ros::ok) {

		scanf(" %f", &x0);
		scanf(" %f", &y0);
		scanf(" %f", &z0);
		

		float theta4;
		scanf(" %f", &theta4);
		set_servo_reference(6, true, theta4, 60,60);

		theta4 = theta4*180/(3.1415);


    	float DH[4][4] = {{cos(theta4),-sin(theta4),0,0},{sin(theta4),cos(theta4),0,0},{0,0,1,-59.74},{0,0,0,1}};
    	float DH_3[3][3] = {{cos(theta4),-sin(theta4),0},{sin(theta4),cos(theta4),0},{0,0,1}};
    	
    	


        float k01 =  DH_3[0][1];
        float k02 =  DH_3[0][2];
        float k03 =  DH_3[0][3];
        float k10 =  DH_3[1][0];
        float k11 =  DH_3[1][1];
        float k12 =  DH_3[1][2];
        float k20 =  DH_3[2][0];
        float k21 =  DH_3[2][1];
        float k22 =  DH_3[2][2];

        DH_3[0][1] = k10;
        DH_3[0][2] = k20;
        DH_3[1][0] = k01;
        DH_3[1][1] = k11;
        DH_3[1][2] = k21;
        DH_3[2][0] = k02;
        DH_3[2][1] = k12;
        DH_3[2][2] = k22;


    	float T[3][1];
    	T[0][0] = DH[0][3];
    	T[1][0] = DH[1][3];
    	T[2][0] = DH[2][3];

    	float invDH[4][4];

    	invDH[0][0] = DH_3[0][0];
    	invDH[0][1] = DH_3[0][1];
    	invDH[0][2] = DH_3[0][2];
    	invDH[0][3] = -(DH_3[0][0] *T[0][0] + DH_3[0][1] *T[1][0] + DH_3[0][2] *T[2][0]);
    


    	invDH[1][0] = DH_3[1][0];
    	invDH[1][1] = DH_3[1][1];
    	invDH[1][2] = DH_3[1][2];
    	invDH[1][3] = -(DH_3[1][0] *T[0][0] + DH_3[1][1] *T[1][0] + DH_3[1][2] *T[2][0]);


    	invDH[2][0] = DH_3[2][0];
    	invDH[2][1] = DH_3[2][1];
    	invDH[2][2] = DH_3[2][2];
    	invDH[2][3] = -(DH_3[2][0] *T[0][0] + DH_3[2][1] *T[1][0] + DH_3[2][2] *T[2][0]);


        ROS_INFO("invDH[0][0]= %f", invDH[0][0]);
        ROS_INFO("invDH[0][1]= %f", invDH[0][1]);
        ROS_INFO("invDH[0][2]= %f", invDH[0][2]);
        ROS_INFO("invDH[0][3]= %f", invDH[0][3]);
        ROS_INFO("invDH[1][0]= %f", invDH[1][0]);
        ROS_INFO("invDH[1][1]= %f", invDH[1][1]);
        ROS_INFO("invDH[1][2]= %f", invDH[1][2]);
        ROS_INFO("invDH[1][3]= %f", invDH[1][3]);
        ROS_INFO("invDH[2][0]= %f", invDH[2][0]);
        ROS_INFO("invDH[2][1]= %f", invDH[2][1]);
        ROS_INFO("invDH[2][2]= %f", invDH[2][2]);
        ROS_INFO("invDH[2][3]= %f", invDH[2][3]);



    	invDH[3][0] = 0;
    	invDH[3][1] = 0;
    	invDH[3][2] = 0;
    	invDH[3][3] = 1;


		
    	float teliko[4][1];

    	teliko[0][0] = invDH[0][0] *x0 + invDH[0][1] *y0 + invDH[0][2] *z0 + invDH[0][3] *1;
    	teliko[1][0] = invDH[1][0] *x0 + invDH[1][1] *y0 + invDH[1][2] *z0 + invDH[1][3] *1;
    	teliko[2][0] = invDH[2][0] *x0 + invDH[2][1] *y0 + invDH[2][2] *z0 + invDH[2][3] *1;
    	teliko[3][0] = 1;
		
    	x0 = teliko[0][0];
    	y0 = teliko[1][0];
    	z0 = teliko[2][0];


    	ROS_INFO("x0 = %f", x0);
    	ROS_INFO("y0 = %f", y0);
    	ROS_INFO("z0 = %f", z0);

		delta_calcInverse(x0, y0, z0, theta1, theta2, theta3);
		ROS_INFO("theta4 = %f", theta4*3.1415/180);
		
    	  
		
		set_servo_reference(0, true, -theta1+80, 60,60);
		set_servo_reference(2, true, -theta2+125, 60,60);
		set_servo_reference(4, true, -theta3+57, 60,60);
		
		
		ros::spinOnce();
		loop_rate.sleep();

		if ((int)getc(stdin) == 27) break;
	}
	disengage();

	return 0;
}

