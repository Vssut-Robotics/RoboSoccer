#include "utils.h"

using namespace boost::asio;
std::ostringstream stream;
RoboCupSSLClient client(40102,"224.5.23.2","");
SSL_WrapperPacket recieve_packet;
SSL_DetectionFrame detection;
ip::udp::endpoint remote_endpoint;
boost::system::error_code err;
grSim_Packet send_packet;
grSim_Robot_Command* command;

pid pid_X(0.002,0.001,0.0),pid_Y(0.0007,0.001,0.0),pid_W(0.03,0.0,0.0);

//Overload == operator for comparision of robot positions
bool operator==(const Point& p1, const Point& p2){
	if(abs(p1.first-p2.first)<=10 && abs(p1.second-p2.second)<=10)
		return true;
	return false;
}

class Alternate
{
public:
	//Start with kick off from formation for yellow team
	//After kickoff select positions for 2 bots and do an indirect goal
	//Return to previous positions and try kick-off + direct shot
	grSim_Robot_Command* kickoff(grSim_Robot_Command* command){
		//bot 3 will go to the ball
		double x_vel = pid_X.calculate(yellow_bot[3].first,ball.first);
		double y_vel = pid_Y.calculate(yellow_bot[3].second,ball.second);
		double ang = calc_angle_between_points(yellow_bot[3].first,yellow_bot[3].second,ball.first,ball.second);
		double angle = tracking_robot_pos.orientation() * 180/PI;
		double w_vel = pid_W.calculate();
		printf("PidX: %3.2f  PidY: %3.2f  PidW: %3.2f ",x_vel,y_vel,w_vel);
		command->set_id(3);
		command->set_wheelsspeed(false);
		command->set_veltangent(x_vel); 
		command->set_velnormal(y_vel);
		command->set_velangular(0.0);
		command->set_kickspeedx(0.0);
		command->set_kickspeedz(0.0);
		command->set_spinner(false);
		return command;
	}
};

int main(){
	io_service io_service;
	ip::udp::socket socket(io_service);
	socket.open(ip::udp::v4());
	remote_endpoint = ip::udp::endpoint(ip::address::from_string("127.0.0.1"),20011);	

    //Starting Connection to grSim Multicast Server
    printf("Connecting to Multicast Server......\n");
    client.open(true);
    printf("Connected\n");
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
		detection = recieve_packet.detection();
//		calcCoordinatesOfAllBots(detection);
	SSL_DetectionBall b = detection.balls(0);
	ball.first = b.x();
	ball.second = b.y();
	for(int i=0;i<8;i++){
		SSL_DetectionRobot r = detection.robots_yellow(i);
		yellow_bot[i].first = r.x();
		yellow_bot[i].second = r.y();
	}
	for(int i=0;i<8;i++){
		SSL_DetectionRobot r = detection.robots_blue(i);
		blue_bot[i].first = r.x();
		blue_bot[i].second = r.y();
	}
		if(Point(yellow_bot[3].first,yellow_bot[3].second) == Point(ball.first,ball.second))
			reached = true;
		grSim_Packet send_packet;
		send_packet.mutable_commands()->set_isteamyellow(true);
		send_packet.mutable_commands()->set_timestamp(0.0);
		grSim_Robot_Command* command1 = send_packet.mutable_commands()->add_robot_commands();
		Alternate a;
		grSim_Robot_Command* command2 = a.kickoff(command1);
		send_packet.SerializeToOstream(&stream);
		
		std::string data = stream.str();
		//std::cout<<"str: "<<stream.str()<<"\n";	
		socket.send_to(buffer(data,int(data.length())),remote_endpoint,0,err);
		std::cout<<"\n";
    }	
    printf("Done!!\n");
    return 0;
}