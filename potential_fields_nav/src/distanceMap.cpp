#include "potential_fields_nav/DistanceMap.h"
#include <cmath>

using namespace potential_fields_nav;

DistanceMap::Origin::Origin(double x, double y, double yaw)
{
	this->x=x;
	this->y=y;
	this->yaw=yaw;
}

DistanceMap::DistanceMap(int* map,int w,int h,float res,double xo,double yo,double yaw, std::vector<int> occ)
{
	width=w;
	height=h;
	resolution=res;
	
	origin=DistanceMap::Origin(xo,yo,yaw);
	
	//inizializzazione celle --> usare VECTOR di Cella*
	dmap=(DistanceMap::Cella*)malloc(width*height*sizeof(DistanceMap::Cella));
	for(int i=0;i<width*height;i++)
	{
		if(map[i]==-1)
		{
			//cella undefined
			dmap[i].dist=-2;
			dmap[i].r=(int) i/width;
			dmap[i].c=i-dmap[i].r*width;
			dmap[i].parent=NULL;
		}
		else if(map[i]==100)
		{
			//cella ostacolo
			dmap[i].dist=0.0;
			dmap[i].r=(int) i/width;
			dmap[i].c=i-dmap[i].r*width;
			dmap[i].parent=&dmap[i];
		}
		else
		{
			//cella non visitata
			dmap[i].dist=-1;
			dmap[i].r=(int) i/width;
			dmap[i].c=i-dmap[i].r*width;
			dmap[i].parent=NULL;
		}
	}
	
	//coda per memorizzare celle espanse
	std::vector<int> queue;
	
	//inizio espansione
	int size=occ.size();
	for(int i=0;i<size;i++)
	{
		int index=occ[i];
		int r= (int) index/width;
		int c= index-r*width;
		expansion(this,r,c,queue);
	}
	std::cout<<"Celle espanse:"<<queue.size()<<std::endl;
	
	//espansioni successive
	while(queue.size()>0)
	{
		int index=queue[0];
		queue.erase(queue.begin());
		int r= (int) index/width;
		int c= index-r*width;
		expansion(this,r,c,queue);
	}
	
	std::cout<<"FINITO"<<std::endl;
}

double potential_fields_nav::distance(int ia, int ib, int ja, int jb, float resolution)
{
	return (double)(resolution*sqrt((ja-jb)*(ja-jb) + (ia -ib)*(ia-ib)));
}

void potential_fields_nav::compute(DistanceMap::Cella* dmap, int r, int c, int i, int j, float resolution, int w, std::vector<int>& queue)
{
	if(dmap[i*w+j].dist==-2)
		return;
		
	double d=distance(r,i,c,j,resolution);
	
	if(!isVisited(dmap[i*w + j]))
	{
		dmap[i*w+j].dist= d+dmap[r*w+c].dist;
		dmap[i*w+j].parent= dmap[r*w+c].parent;
		queue.push_back(i*w+j);
	}
	else if(dmap[i*w+j].dist > d+dmap[r*w+c].dist)
	{
		dmap[i*w+j].dist= d+dmap[r*w+c].dist;
		dmap[i*w+j].parent= dmap[r*w+c].parent;
		queue.push_back(i*w+j);
	}
}

void potential_fields_nav::expansion(DistanceMap* map, int i, int j, std::vector<int>& queue)
{
	int w=map->getWidth();
	int h=map->getHeight();
	float res=map->getResolution();
		
	if(isValid(i+1,j,w,h))
		compute(map->dmap,i,j,i+1,j,res,w,queue);
	
	if(isValid(i-1,j,w,h))
		compute(map->dmap,i,j,i-1,j,res,w,queue);
	
	if(isValid(i,j+1,w,h))
		compute(map->dmap,i,j,i,j+1,res,w,queue);
		
	if(isValid(i,j-1,w,h))
		compute(map->dmap,i,j,i,j-1,res,w,queue);
	
	if(isValid(i-1,j-1,w,h))
		compute(map->dmap,i,j,i-1,j-1,res,w,queue);
	
	if(isValid(i+1,j-1,w,h))
		compute(map->dmap,i,j,i+1,j-1,res,w,queue);
		
	if(isValid(i+1,j+1,w,h))
		compute(map->dmap,i,j,i+1,j+1,res,w,queue);
		
	if(isValid(i-1,j+1,w,h))
		compute(map->dmap,i,j,i-1,j+1,res,w,queue);
	
	return;
}

bool potential_fields_nav::isVisited(DistanceMap::Cella c)
{
	if(c.dist==-1 && c.parent==NULL)
		return false;
	else
		return true;
}

bool potential_fields_nav::isValid(int i, int j, int w, int h)
{
	if(i>=0 && i<h && j>=0 && j<w)
		return true;
	
	else
		return false;
}

DistanceMap::~DistanceMap()
{
	if(dmap!=0)
		free(dmap);
	
	std::cout<<"Distance Map distrutta"<<std::endl;
}
