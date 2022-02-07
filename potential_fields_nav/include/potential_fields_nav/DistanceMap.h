#include <stdio.h>
#include <iostream>
#include <vector>

namespace potential_fields_nav {

class DistanceMap {
	
public:
	struct Cella {
		Cella();
		double dist;
		int r, c;
		Cella* parent;
	};
	
	struct Origin{
		Origin(double x_=0.0, double y_=0.0, double yaw_=0.0);
		double x;
		double y;
		double yaw;
	};
  
protected:
	int width;
	int height;
	float resolution;
	Origin origin;

public:
	Cella* dmap;
	
public:
	DistanceMap(int* map,int w,int h,float res,double xo,double yo,double yaw,std::vector<int> oc);
	~DistanceMap();
	//DistanceMap& operator=(const DistanceMap& other);
	int getWidth(){
		return width;	
	};
	int getHeight(){
		return height;	
	};
	float getResolution(){
		return resolution;	
	};
	Origin getOrigin(){
		return origin;
	};
};

double distance(int ia, int ib, int ja, int jb, float resolution);
void compute(DistanceMap::Cella* dmap, int r, int c, int i, int j, float resolution, int w, std::vector<int>& queue);
void expansion(DistanceMap* map, int i, int j, std::vector<int>& queue);
bool isVisited(DistanceMap::Cella c);
bool isValid(int i, int j, int w, int h);

}
