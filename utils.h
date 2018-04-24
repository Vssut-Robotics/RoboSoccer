#pragma once

#ifndef UTILS_INCLUDE
#define UTILS_INCLUDE

#include <sstream>
#include <iostream>
#include <cmath>
#include <array>
#include <algorithm>
#include <random>
#include "timer.h"
#include "boost/asio.hpp"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "robocup_ssl_client.h"
#include "grSim_Replacement.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_detection.pb.h"

#define INF 100000
using namespace std;
const float PI = acos(-1.0);
typedef std::pair<int,int> Point;
typedef std::array<Point,8> Coordinates;

Coordinates blue_bot,yellow_bot;
Point ball;

void calcCoordinatesOfAllBots(SSL_DetectionFrame detection);
double calc_angle_between_points(double x1,double y1,double x2,double y2);
double calc_angle_between_lines(double x1,double y1,double x2,double y2);
bool isPointInsideTriangle(Point pt,Point p1,Point p2,Point p3);
bool isInside0and1(double val);
bool onSegment(Point p, Point q, Point r);
int orientation(Point p, Point q, Point r);
bool doIntersect(Point p1, Point q1, Point p2, Point q2);
bool isInsidePolygon(Point polygon[], int n, Point p);
double getDistance(Point a,Point b);
Point generateRandomPointinRange(int min_x,int max_x,int min_y,int max_y);

class pid{
	private:
		float kp,ki,kd;
		double error,last_error,integral,derivative,min_limit,max_limit,val_pid;
	public:
		pid(float _kp,float _ki,float _kd,double =0.0,double =3.0);		
		double calculate(double set_point,double current_reading);
};

#endif