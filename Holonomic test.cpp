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
pid pid_X(0.4,0.0,0.3),pid_Y(0.04,0.0,0.0),pid_W(0.08,0.0,0.0);

//Overload == operator for comparision of robot positions
bool operator==(const SSL_DetectionRobot& p1, const SSL_DetectionRobot& p2){
	if(abs(p1.x()-p2.x())<=20 && abs(p1.y()-p2.y())<=20)
		return true;
	return false;
}


template<class V,class T>
void constrain(V *sp,T lower,T upper){
	if(*sp < lower) *sp = lower;
	if(*sp > upper) *sp = upper;
}

//Motor ordering
//   -Foreward-
//   X1-------X4
//	   -------
//   X2-------X3
// For moving forward we need -,-,+,+
//Inverse Kinematics matrix
const float a[4][4] = {{-0.346411,0.414214,0.292893},
					   {-0.282845,-0.414214,0.207107},
					   {0.282845,-0.414214,0.207107},
					   {0.346411,0.414214,0.292893}};

//Speeds can be negative.
double s1,s2,s3,s4,sp=1.0,maxSpeed,x=0.0,y=0.0,w=0.0;

void invKinematics(){
	maxSpeed = 100.0;
	
	s1 = a[0][0]*x + a[0][1]*y + a[0][2]*w;
	s2 = a[1][0]*x + a[1][1]*y + a[1][2]*w;
	s3 = a[2][0]*x + a[2][1]*y + a[2][2]*w;
	s4 = a[3][0]*x + a[3][1]*y + a[3][2]*w;
	
	constrain<double,double>(&s1,-maxSpeed,maxSpeed);
	constrain<double,double>(&s2,-maxSpeed,maxSpeed);
	constrain<double,double>(&s3,-maxSpeed,maxSpeed);
	constrain<double,double>(&s4,-maxSpeed,maxSpeed);
}

void displaySpeed(){
	std::cout<<"s1: "<<s1<<" ";
	std::cout<<"s2: "<<s2<<" ";
	std::cout<<"s3: "<<s3<<" ";
	std::cout<<"s4: "<<s4<<" ";
}

void Stop_ALL(grSim_Robot_Command** command){
    (*command)->set_id(0);
    (*command)->set_wheelsspeed(false);
    (*command)->set_wheel1(0.0);
    (*command)->set_wheel2(0.0);
    (*command)->set_wheel3(0.0);
    (*command)->set_wheel4(0.0);
    (*command)->set_veltangent(0.0);
    (*command)->set_velnormal(0.0);
    (*command)->set_velangular(0.0);
    (*command)->set_kickspeedx(0.0);
    (*command)->set_kickspeedz(0.0);
    (*command)->set_spinner(false); 
}

int main(){	    
	io_service io_service;
	ip::udp::socket socket(io_service);
	socket.open(ip::udp::v4());
	remote_endpoint = ip::udp::endpoint(ip::address::from_string("127.0.0.1"),20011);	
    //Starting Connection to Command Server
    printf("Connecting to Command Server........\n");
    printf("Connected Successfully to Command Server.\n");
    //Server Correctly Connected
    
    //Starting Connection to grSim Multicast Server
    printf("Connecting to Multicast Server......\n");
    client.open(true);
	//Server Correctly Connected
	
	//Start fetching data from the detected packet	
	bool reached = false;
	while(!reached){
		if (!client.receive(recieve_packet)){
			printf("Connection to Server Unsuccessful!!\n");
			return -1;
		}
		//printf("Client Successfully Connected\n");
		if (!recieve_packet.has_detection()){
			printf("Recieved Packet has no Detection Frame!!\n");
			return -1;
		}
		//printf("Everything's Going well buddy!!\n");
		detection = recieve_packet.detection();
		ball = detection.balls(0);
		destination_robot_pos.set_x(0.0);
		destination_robot_pos.set_y(0.0);		
		tracking_robot_pos = detection.robots_yellow(0);//Send robot-1 near the ball
		//printf("Bot Position:(%3.2f,%3.2f)  ",tracking_robot_pos.x(),tracking_robot_pos.y());
		grSim_Packet send_packet;
		send_packet.mutable_commands()->set_isteamyellow(true);
		send_packet.mutable_commands()->set_timestamp(0.0);
		grSim_Robot_Command* command = send_packet.mutable_commands()->add_robot_commands();
		if(tracking_robot_pos == destination_robot_pos) Stop_ALL(&command),reached = true;
		
		command->set_id(0);
		double angle = tracking_robot_pos.orientation() * 180/PI;
		if(angle >= -180.0 && angle <= -0.0) angle += 360.0; 
		x = pid_X.calculate(0.0,tracking_robot_pos.x());
		y = pid_Y.calculate(0.0,tracking_robot_pos.y());
		w = pid_W.calculate(360.0,angle);
		printf("PidX: %3.2f  PidY: %3.2f  PidW: %3.2f ",x,y,w);
		command->set_wheelsspeed(true);
//		x = 0,y = 0,w = 10;
		w = 0.0;
		invKinematics();
		displaySpeed();
		command->set_wheel1(s1);
		command->set_wheel2(s2);
		command->set_wheel3(s3);
		command->set_wheel4(s4);		 
		command->set_veltangent(x); 
		command->set_velnormal(y);
		if(abs(90.0 - angle) < 180.0)
		command->set_velangular(-w);
		else command->set_velangular(w);
		command->set_kickspeedx(0.0);
		command->set_kickspeedz(0.0);
		command->set_spinner(false);		
		
		send_packet.SerializeToOstream(&stream);
		
		//Send the Command
		std::string data = stream.str();
		//std::cout<<"str: "<<stream.str()<<"\n";	
		socket.send_to(buffer(data,int(data.length())),remote_endpoint,0,err);
		//Command Sent
		//printf("Data Sent");
		std::cout<<"\n";
	}
	printf("\nExiting\n");
	socket.close();
	return 0;
}



