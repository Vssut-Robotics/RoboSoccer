#include "utils.h"

class FieldAnalysis
{
public:
	FieldAnalysis();
	~FieldAnalysis();
	//Defense Calculator
	//Team Closest To The Ball
	bool iSBallWithTeamYellow(){
		bool ret = true;
		int dist1 = INF,dist2 = INF;
		for(int i=0;i<8;i++){
			dist1 = min(dist1,getDistance(yellow_bot[i],ball));
			dist2 = min(dist2,getDistance(blue_bot[i],ball));
		}
		ret = dist1 < dist2;
		return ret;
	}
	//Ball Possession ID
	int BallPossessionId(){
		int id = 0,dist = INF;
		if(iSBallWithTeamYellow){
			for(int i=0;i<8;i++){
				if(getDistance(yellow_bot[i],ball) < dist){
					dist = getDistance(yellow_bot[i],ball);
					id = i;
				}
			}		
		}
		else{
			for(int i=0;i<8;i++){
					if(getDistance(blue_bot[i],ball) < dist){
						dist = getDistance(yellow_bot[i],ball);
						id = i;
					}
				}					
		}
		return id;
	}
	//Offense Points Carrier
	//Offense Points receiver	
};