/***
 Basic structure for streamline
 ***/
#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H
#include "dataStructure.h"
class Trajectory{

public:
	int index;
	int  nlinesegs;
	int  curMaxNumLinesegs;
	LineSeg *linesegs;
	unsigned char roadtype;  /*record the road type for street modeling*/
	bool closed;
	bool is_mapboundary;     /*  record whether it is the boundaries of a loaded map  */

	int saddleID;           /*which saddle this trajectory belongs to*/
    int degpt_id;
    int linked_degpt_id;
    int degpt_sep_index;
	double eulerstep_scalar;

	double traj_len;
    float infoGain;
    bool is_reach_degpt;
    bool is_reach_unknown;
	/*Construct the trajectory*/
	Trajectory(int index, int curMaxNum);

	~Trajectory();
	//void setGlview(GlView *glviewS);
	bool store_to_global_line_segs(CurvePoints *temp, int num);

	/*extend the line segment list if there is not enough space left*/
	bool extend_line_segments(int add_size);

	//get the length of the trajectory
	double get_length();


	//remove the front n line segments
	bool remove_front_nlines(int n);

	//add n new line segments in the front
	bool add_front_nlines(LineSeg *, int);

	//remove the last n line segments
	bool remove_last_nlines(int n);

	//add n new line segments at the end
	bool add_last_nlines(LineSeg *, int);


	//reverse the trajectory
	//bool reverse_lines();
	//int trace_in_quad(int &face_id, double globalp[2], int type, int &flag);

}; //end of Trajectory class
#endif
