/***
 Implementation of streamline class for path advection and separatrice generation
 ***/
#ifndef _EVENSTREAMLINEPLACE_H
#define _EVENSTREAMLINEPLACE_H
#include "dataStructure.h"
#include "Util.h"
//#include "GlView.h"
#include <algorithm>
#include "TrajectoryList.h"
#include "QuadMesh.h"
class TfCore;
class EvenStreamlinePlace
{
public:
	TrajectoryList *evenstreamlines;
	SamplePtList **samplepts;
    int num_tracingpoints ;
	DynList_Int *trianglelist;
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
	/*to record the sample that stops the tracing of a tensor line*/
	int which_triangle;
	double samp[2];
    icVector2 outside_push;
    bool isAdd_push;
    float push_rank;

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
                                             int type, icVector2 direction,int index,int sep_index);
	void compute_tensor_at_quad(int face, double x, double y, icMatrix2x2 &ten);
	bool is_in_cell(int id, double x, double y);
	int get_cellID_givencoords(double x, double y);
	int trace_majRoad_in_quad(int &face_id, double globalp[2], int type, 
		double dtest, double loopsep, double dist2sing, 
		double sample_interval, double discsize, int &flag);
    int trace_separatrix_in_quad(int &face_id, double globalp[2], int type,
        double dtest, double loopsep, double dist2sing,
        double sample_interval, double discsize, int &flag,int index, int sep_index);
	void get_tenvec_quad(double cur_p[2], double vec[2]);
	void compute_phi_in_quad(int face, double x, double y, double &phi);
	void RK23_2d(double pre_p[2], double next_p[2], double &hstep_loc, double &hnext,
		double eps, double &eps_did);
	bool get_nextpt_RK23_ten_quad(double first[2], double second[2], int &face_id, int type);
    int get_nextpt_RK23_ten_quad(double first[2], double second[2], int &face_id, int type,int index,int sep_index);
	bool close_to_cur_samplePt(double p[2], int triangle, SamplePt **samples, int num_samples,
		double separate_dist, double discsize, double sample_interval);
	void cal_euclidean_dist_2(int triangle, double p[3], double dsep, double discsize, 
		DynList_Int *trianglelist);
	void reset_dist(DynList_Int *trianglelist);
	void get_next_cell_2(int &face_id, double pre[2], double cur[2], 
		int &PassVertornot, int type);
	bool cross_vertex_ten_quad(int &face_id, double cur_p[2], double pre_p[2], int &passornot, int type);
	void get_cell_through_ver(int vertid, int &cell, int type);
	SamplePt **cal_samplepts_when_tracing(int traj, double interval, int &cur_line, int &movetonext, double &cur_length, 
		SamplePt **samples, int &num_samples);
	bool cal_a_sample_of_streamline(int traj, int &cur_lineindex, int &movetonext,
		double curpt[2], double interval, double &cur_length);
	bool is_in_reg_cell(int id, double x, double y);
	void reverse_streamline(int streamlineid);
    void set_default_parameters();
    void init_major_line_info();
    void reset();
};
#endif

