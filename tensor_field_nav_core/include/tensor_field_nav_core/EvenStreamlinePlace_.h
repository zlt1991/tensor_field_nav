#ifndef _EVENSTREAMLINEPLACE_H
#define _EVENSTREAMLINEPLACE_H
#include "dataStructure.h"
#include "Util.h"
#include <algorithm>
#include "TrajectoryList.h"
#include "QuadMesh.h"
class TfCore;
class EvenStreamlinePlace
{
public:
	TrajectoryList *evenstreamlines;
	double streamlinelength;
	double dsep;
	double percentage_dsep;
	double discsize;
	double sample_interval;
	int every_nsample;
	double loopdsep;
	double dist2sing;
	double seeddist;
	double minstartdist;
    TfCore *m_tfCore;
	QuadMesh* quadmesh;

	//////////////////////////////////////////////////////////////////////////
	icVector2 tenline_dir_global;  
	double hstep;
	double predict_stepsize;
	double euler_stepsize;
	int globalface;
	int g_face, g_type;
	double majorDensity;
	double mintenline_length;
	//////////////////////////////////////////////////////////////////////////
    EvenStreamlinePlace(int initsize);

	~EvenStreamlinePlace();

	void init();
    void setTfCore(TfCore *tfCore);
	bool grow_a_majRoad(double seed_p[2], int triangle, double dtest, 
		double discsize, double Sample_interval, 
		double loopdsep, double dist2sing, 
        double streamlinelength,int type,icVector2 &direction);
    bool grow_a_separatrix(double start[2],double seed_p[2], int triangle, double dtest,
                                             double discsize, double Sample_interval,
                                             double loopdsep, double dist2sing,
                                             double streamlinelength,
                                             int type, icVector2 direction,int index);
	void compute_tensor_at_quad(int face, double x, double y, icMatrix2x2 &ten);
	bool is_in_cell(int id, double x, double y);
	int get_cellID_givencoords(double x, double y);
	int trace_majRoad_in_quad(int &face_id, double globalp[2], int type, 
		double dtest, double loopsep, double dist2sing, 
		double sample_interval, double discsize, int &flag);
    int trace_separatrix_in_quad(int &face_id, double globalp[2], int type,
        double dtest, double loopsep, double dist2sing,
        double sample_interval, double discsize, int &flag,int index);
	void get_tenvec_quad(double cur_p[2], double vec[2]);
	void RK23_2d(double pre_p[2], double next_p[2], double &hstep_loc, double &hnext,
		double eps, double &eps_did);
	bool get_nextpt_RK23_ten_quad(double first[2], double second[2], int &face_id, int type);
    int get_nextpt_RK23_ten_quad(double first[2], double second[2], int &face_id, int type,int index);
	void get_next_cell_2(int &face_id, double pre[2], double cur[2], 
		int &PassVertornot, int type);
	bool cross_vertex_ten_quad(int &face_id, double cur_p[2], double pre_p[2], int &passornot, int type);
	void get_cell_through_ver(int vertid, int &cell, int type);
	void set_default_parameters(bool fieldtype);
    void init_major_line_info();
    void reset();
};
#endif

