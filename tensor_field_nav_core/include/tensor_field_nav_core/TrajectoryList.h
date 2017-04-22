#ifndef _TRAJECTORYLIST_H
#define _TRAJECTORYLIST_H
#include "dataStructure.h"
#include "Trajectory.h"
class TrajectoryList
{
public:
	Trajectory **trajs;         //the trajectory list
	int ntrajs;                  //current number of existing trajectories
	int curMaxNumTrajs;          //maximum number of trajectories can be stored
	double length;                //the flow length of the trajectory
	TrajectoryList(int initsize); //construction
	~TrajectoryList();
	 bool append(Trajectory *s);
	 bool del_End() ;
	 void copy_Elem(Trajectory *s, Trajectory *d);
	 bool del_Node(Trajectory *s) ;
	  bool isEmpty();
	  bool isFull();
	  bool extend(int step);
	  void reset();
}; //end of TrajectoryList class
#endif
