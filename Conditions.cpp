#include "utils.h"

class Conditions
{
public:
	Conditions(arguments);
	~Conditions();
	/*For Chip
	1.iSDirectShotToGoal
	2.iSDistanceCheapFeasible
	3.iSFastestWayToPass
	*/
	bool iSDirectShotToGoal(SSL_DetectionRobot tracking_robot,bool iSTeamYellow){

	}
	/*For Direct Goal
	1.iSInLos
	2.iSDistanceInThreshold
	*/
	bool iSInLos(SSL_DetectionRobot tracking_robot,bool iSTeamYellow){
		//form an Los from ball to two goal posts
		int cnt=0;//for counting the number of robots in los if more than 2 then return false
		if(iSTeamYellow){
			Point p1(ball.first,ball.second),p2(-4000,550),p3(-4000,-550);
			for(int i=0;i<n;i++){
				cnt += isPointInsideTriangle(yellow_bot[i],p1,p2,p3);
				cnt += isPointInsideTriangle(blue_bot[i],p1,p2,p3);
			}
		}
		else{
			Point p1(ball.first,ball.second),p2(4000,550),p3(4000,-550);
			for(int i=0;i<n;i++){
				cnt += isPointInsideTriangle(yellow_bot[i],p1,p2,p3);
				cnt += isPointInsideTriangle(blue_bot[i],p1,p2,p3);
			}
		}
		return cnt<=2;
	}
	/*For Indirect Goal
	1.NotInLos
	2.iSInLosOfPassingBot
	3.iSBallPassable
	*/
	bool NotInLos(SSL_DetectionRobot tracking_robot,bool iSTeamYellow){
		return !iSInLos(tracking_robot,iSTeamYellow);
	}
	bool iSInLosOfPassingBot(SSL_DetectionRobot passing_robot,bool iSTeamYellow){
		return iSInLos(passing_robot,iSTeamYellow);
	}
	bool iSBallPassable(SSL_DetectionRobot tracking_robot,SSL_DetectionRobot passing_robot,bool iSTeamYellow){
		bool ret = true;
		double m = (passing_robot.y() - tracking_robot.y())/(passing_robot.x() - tracking_robot.x());
		double d = 100.0,y1 = tracking_robot.y() + d * sqrt(1+m*m),y2 = y1 + m * (passing_robot.x() - tracking_robot.x());
		double y3 = tracking_robot.y() - d * sqrt(1+m*m),y4 = y3 + m * (passing_robot.x() - tracking_robot.x());
		Point polygon[] = {{tracking_robot.x(),y1},{passing_robot.x(),y2},{tracking_robot.x(),y3},{passing_robot.y(),y4}};
		for(int i=0;i<8;i++){
			ret |= isInsidePolygon(polygon,4,yellow_bot[i]);
		}
		for(int i=0;i<8;i++){
			ret |= isInsidePolygon(polygon,4,blue_bot[i]);
		}
		return ret;
	}
	/*For Passing Forward Or Backwards
	1.iSBothBotsPositionLessThanThreshold
	2.iSInLos then Ground Pass
	3.Else Chip Pass
	4.IfNoShotEitherDirectOrIndirect
	5.LimitingConditionForGeometry
	*/
};