#include <sstream>
#include <iostream>
#include <cmath>
#include "timer.h"
#include "boost/asio.hpp"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "robocup_ssl_client.h"
#include "grSim_Replacement.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_detection.pb.h"

const float PI = acos(-1.0);
using namespace boost::asio;
std::ostringstream stream;
RoboCupSSLClient client(40102,"224.5.23.2","");
SSL_WrapperPacket recieve_packet;
SSL_DetectionFrame detection;
SSL_DetectionBall ball;
SSL_DetectionRobot tracking_robot_pos,destination_robot_pos,keeper_robot_pos;
ip::udp::endpoint remote_endpoint;
boost::system::error_code err;
grSim_Packet send_packet;
grSim_Robot_Command* command;
double x_vel,y_vel,w_vel,angle,angle_between_bot_and_ball;
bool keeper_in_position = false;



class pid{
	private:
		float kp,ki,kd;
		double error,last_error,integral,derivative,min_limit,max_limit,val_pid;
	public:
		pid(float _kp,float _ki,float _kd,double _min_limit=0.0,double _max_limit=3.0){
			kp = _kp;
			ki = _ki;
			kd = _kd;
			min_limit = _min_limit;
			max_limit = _max_limit;
			last_error = 0.0;
		}
		double calculate(double set_point,double current_reading){
			error = current_reading - set_point;
			integral += error;
			if(integral < min_limit) integral = min_limit;
			if(integral > max_limit) integral = max_limit;
			derivative = error - last_error;
			last_error = error;
			val_pid = kp * error + ki * integral + kd * derivative;
			return val_pid;
		}
};
pid pid_X(0.002,0.001,0.0),pid_Y(0.0007,0.001,0.0),pid_W(0.03,0.0,0.0);

//Overload == operator for comparision of robot positions
bool operator==(const SSL_DetectionRobot& p1, const SSL_DetectionRobot& p2){
	if(abs(p1.x()-p2.x())<=10 && abs(p1.y()-p2.y())<=10)
		return true;
	return false;
}

//Calculate angle from x axis to a given two point
double calc_angle_between_points(double x1,double y1,double x2,double y2){
	if(x2 - x1 != 0)
		return fmod((atan2(y2-y1,x2-x1) * 180/PI)+450.0,360.0);
	return 0;
}

double calc_angle_between_lines(double x1,double y1,double x2,double y2){
	double angle1 = calc_angle_between_points(0.0,0.0,x1,y1);
	double angle2 = calc_angle_between_points(0.0,0.0,x2,y2);
	//printf("angle1:%3.2f angle2:%3.2f  ",angle1,angle2);
	return abs(angle1 - angle2);
}

int main(){	    
	io_service io_service;
	ip::udp::socket socket(io_service);
	socket.open(ip::udp::v4());
	remote_endpoint = ip::udp::endpoint(ip::address::from_string("127.0.0.1"),20011);	

    //Starting Connection to grSim Multicast Server
    printf("Connecting to Multicast Server......\n");
    client.open(true);
	//Server Correctly Connected
    while(!keeper_in_position){
		if (!client.receive(recieve_packet)){
			printf("Connection to Server Unsuccessful!!\n");
			return -1;
		}
		//printf("Client Successfully Connected\n");
		if (!recieve_packet.has_detection()){
			printf("Recieved Packet has no Detection Frame!!\n");
			return -1;
		}
		detection = recieve_packet.detection();
		ball = detection.balls(0);
		keeper_robot_pos.set_x(3000.0);
		keeper_robot_pos.set_y(0.0);
		tracking_robot_pos = detection.robots_yellow(0);//Send robot-1 near the ball
		//printf("Bot Position:(%3.2f,%3.2f)  ",tracking_robot_pos.x(),tracking_robot_pos.y());
		grSim_Packet send_packet;
		send_packet.mutable_commands()->set_isteamyellow(true);
		send_packet.mutable_commands()->set_timestamp(0.0);
		grSim_Robot_Command* command = send_packet.mutable_commands()->add_robot_commands();
		
		if(tracking_robot_pos == keeper_robot_pos) keeper_in_position = true;
		
		command->set_id(0);
		angle = tracking_robot_pos.orientation() * 180/PI;
		if(angle >= -180.0 && angle <= -0.0) angle += 360.0;
		angle_between_bot_and_ball = calc_angle_between_points(3000.0,0.0,tracking_robot_pos.x(),tracking_robot_pos.y()); 
		printf("angle:%3.2f ",angle_between_bot_and_ball);
		x_vel = pid_X.calculate(3000.0,tracking_robot_pos.x());
		y_vel = pid_Y.calculate(0.0,tracking_robot_pos.y());
		w_vel = pid_W.calculate(angle_between_bot_and_ball,angle);
		printf("PidX: %3.2f  PidY: %3.2f  PidW: %3.2f ",x_vel,y_vel,w_vel);
		command->set_wheelsspeed(false);
		command->set_veltangent(x_vel); 
		command->set_velnormal(y_vel);
		if(abs(90.0 - angle) < 180.0)
		command->set_velangular(-w_vel);
		else command->set_velangular(w_vel);
		//command->set_velangular(0.0);
		command->set_kickspeedx(0.0);
		command->set_kickspeedz(0.0);
		command->set_spinner(false);		
		
		send_packet.SerializeToOstream(&stream);
		
		std::string data = stream.str();
		//std::cout<<"str: "<<stream.str()<<"\n";	
		socket.send_to(buffer(data,int(data.length())),remote_endpoint,0,err);
		std::cout<<"\n";
	}
	printf("Keeper in Position\n");
	// Keeper in Position protect the ball
	while(true){
		if (!client.receive(recieve_packet)){
			printf("Connection to Server Unsuccessful!!\n");
			return -1;
		}
		//printf("Client Successfully Connected\n");
		if (!recieve_packet.has_detection()){
			printf("Recieved Packet has no Detection Frame!!\n");
			return -1;
		}
		detection = recieve_packet.detection();
		ball = detection.balls(0);
		keeper_robot_pos.set_x(3000.0);
		keeper_robot_pos.set_y(0.0);
		tracking_robot_pos = detection.robots_yellow(0);//Send robot-1 near the ball
		//printf("Bot Position:(%3.2f,%3.2f)  ",tracking_robot_pos.x(),tracking_robot_pos.y());
		grSim_Packet send_packet;
		send_packet.mutable_commands()->set_isteamyellow(true);
		send_packet.mutable_commands()->set_timestamp(0.0);
		grSim_Robot_Command* command = send_packet.mutable_commands()->add_robot_commands();
		
		if(tracking_robot_pos == keeper_robot_pos) keeper_in_position = true;
		
		command->set_id(0);
		angle = tracking_robot_pos.orientation() * 180/PI;
//		printf("angle1: %3.2f ",angle);
		angle = fmod((angle+450),360.0);
//		printf("angle2: %3.2f\n",angle);		
		angle_between_bot_and_ball = calc_angle_between_lines(ball.x(),ball.y(),tracking_robot_pos.x(),tracking_robot_pos.y()); 
//		printf(" angle:%3.2f ",angle_between_bot_and_ball);
		
		double m = (ball.y() - 0.0)/(ball.x() - 4000.0);
		m = m*m; //Since i always need m^2
		double x_to_go = 4000.0 - (800.0/sqrt(1.0+m));//where 4000.0 is the center of goal post and 1000.0 is the radius of semicircle
		double y_to_go = 0.0 - (800.0/sqrt(1.0+(1.0/m)));//y can be +/- based on conditions
		if(ball.y()>0.0) y_to_go = 0.0 + (800.0/sqrt(1.0+(1.0/m)));
		else y_to_go = 0.0 - (800.0/sqrt(1.0+(1.0/m)));
		printf("x:%3.2f y:%3.2f ",x_to_go,y_to_go);

		x_vel = pid_X.calculate(x_to_go,tracking_robot_pos.x());
		y_vel = pid_Y.calculate(y_to_go,tracking_robot_pos.y());
		w_vel = pid_W.calculate(angle_between_bot_and_ball,angle);
		printf("PidX:%3.2f  PidY:%3.2f  PidW:%3.2f ",x_vel,y_vel,w_vel);
		command->set_wheelsspeed(false);
		command->set_veltangent(x_vel); 
		command->set_velnormal(y_vel);
		command->set_veltangent(0.0); 
		command->set_velnormal(0.0);
		//if(x_vel <= 0.1 && y_vel <= 0.1)
		command->set_velangular(-w_vel);
		//else command->set_velangular(0.0);
		command->set_kickspeedx(0.0);
		command->set_kickspeedz(0.0);
		command->set_spinner(false);		
		
		send_packet.SerializeToOstream(&stream);
		
		std::string data = stream.str();
		//std::cout<<"str: "<<stream.str()<<"\n";	
		socket.send_to(buffer(data,int(data.length())),remote_endpoint,0,err);
		std::cout<<"\n";
	}

	printf("\nExiting\n");
	socket.close();
	return 0;
}

