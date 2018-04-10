#include "utils.h"

pid::pid(float _kp,float _ki,float _kd,double _min_limit=0.0,double _max_limit=3.0){
	kp = _kp;
	ki = _ki;
	kd = _kd;
	min_limit = _min_limit;
	max_limit = _max_limit;
	last_error = 0.0;
}

double pid::calculate(double set_point,double current_reading){
	error = current_reading - set_point;
	integral += error;
	if(integral < min_limit) integral = min_limit;
	if(integral > max_limit) integral = max_limit;
	derivative = error - last_error;
	last_error = error;
	val_pid = kp * error + ki * integral + kd * derivative;
	return val_pid;
}

void calcCoordinatesOfAllBots(SSL_DetectionFrame detection){
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
}

bool isInside0and1(double val){
	return ((0 <= val) && (val <= 1));
}

bool isPointInsideTriangle(Point pt,Point p1,Point p2,Point p3){
	//barycentric coordinate system is used 
	double denominator = ((p2.second - p3.second)*(p1.first - p3.first) + (p3.first - p2.first)*(p1.second- p3.second));
	double a = ((p2.second - p3.second)*(pt.first - p3.first) + (p3.first - p2.first)*(pt.second- p3.second))/denominator;
	double b = ((p3.second - p1.second)*(pt.first - p3.first) + (p1.first - p3.first)*(pt.second- p3.second))/denominator;
	double c = 1 - a - b;
	return isInside0and1(a) && isInside0and1(b) && isInside0and1(c);
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

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(Point p, Point q, Point r)
{
	if (q.first <= max(p.first, r.first) && q.first >= min(p.first, r.first) &&
			q.second <= max(p.second, r.second) && q.second >= min(p.second, r.second))
		return true;
	return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
	int val = (q.second - p.second) * (r.first - q.first) -
			(q.first - p.first) * (r.second - q.second);

	if (val == 0) return 0; // colinear
	return (val > 0)? 1: 2; // clock or counterclock wise
}

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and p2 are colinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are colinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are colinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases
}

// Returns true if the point p lies inside the polygon[] with n vertices
bool isInsidePolygon(Point polygon[], int n, Point p)
{
	// There must be at least 3 vertices in polygon[]
	if (n < 3) return false;

	// Create a point for line segment from p to infinite
	Point extreme = {INF, p.second};

	// Count intersections of the above line with sides of polygon
	int count = 0, i = 0;
	do
	{
		int next = (i+1)%n;

		// Check if the line segment from 'p' to 'extreme' intersects
		// with the line segment from 'polygon[i]' to 'polygon[next]'
		if (doIntersect(polygon[i], polygon[next], p, extreme))
		{
			// If the point 'p' is colinear with line segment 'i-next',
			// then check if it lies on segment. If it lies, return true,
			// otherwise false
			if (orientation(polygon[i], p, polygon[next]) == 0)
			return onSegment(polygon[i], p, polygon[next]);

			count++;
		}
		i = next;
	} while (i != 0);

	// Return true if count is odd, false otherwise
	return count&1; // Same as (count%2 == 1)
}
