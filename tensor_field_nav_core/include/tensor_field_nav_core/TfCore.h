#ifndef _TfCore_H
#define _TfCore_H

#include <GL/glut.h> 
#include "Parameters.h"
#include "dataStructure.h"
#include "icVector.h"
#include "QuadMesh.h"
#include "Util.h"
#include <std_msgs/String.h>
#include "EvenStreamlinePlace.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include "KeyFrameList.h"
#include "ParaThread.h"
#include "VertTimeList.h"
#include "Numerical.h"
#include "mst.h"
#include <nav_msgs/OccupancyGrid.h>
#include "pure_pursuit_controller/executePath.h"
#include "pure_pursuit_controller/cancelPath.h"
#include "pure_pursuit_controller/cutPath.h"
#include "pure_pursuit_controller/recoverPath.h"
#include "std_msgs/Float64MultiArray.h"
#include "tensor_field_nav_core/CalPathInfoGain.h"

// CTfCore
class TfCore
{
public:
    TfCore(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
    ~TfCore();

	// Attributes
public:
	//
	double  ten_tmax ;
	double  ten_dmax ;
	int    iframe ; 
	int    Npat  ;
	int    alpha  ;
	float  tmax ;
	float  dmax ;
	double trans_x,trans_y,zoom_factor;
	float inverse_tran[16];
	int curMaxNumTenRegElems;
	int MaxNumDegeneratePts;
	int major_iframe;
	int ntenelems;
	int nten_regelems;
	int ndegpts;
	DegeneratePt *degpts;
	TenRegularElem *ten_regularelems;
	GLuint tentextnames[16];
	GLubyte major_tex1[NPIX][NPIX][3], major_tex2[NPIX][NPIX][3], major_tex[NPIX][NPIX][3],
		minor_tex1[NPIX][NPIX][3], minor_tex2[NPIX][NPIX][3], minor_tex[NPIX][NPIX][3],
		major_alpha_map[NPIX][NPIX][3], minor_alpha_map[NPIX][NPIX][3];

	GLubyte major_temp[NPIX][NPIX][4], minor_temp[NPIX][NPIX][4];

	/*the quad mesh object for the tensor field design*/
	QuadMesh *quadmesh;
    //////////////////////////////////////////////////////////////////////////
	Degenerate_Design *ten_designelems;
	int curMaxNumTenDesignElems;

	int ndegenerate_tris;
	int *degenerate_tris;

	EvenStreamlinePlace *major_path;
    //2016_12_14
    int NUM_Slices;
    KeyFrameList *keyframes;
    bool enRelaxKeyframeOn;
    int InterpScheme;
    int curSliceID;

    std::vector<ParaThread *> m_myThreads;
    //
    VertTimeList *vertTimeList;
    Vec_INT *ija_p;
    Vec_DP *sa_p;
    icVector2 *vecsTime;
    ///////////////////////////////
    bool showIBFVOn;
    bool showRegularElemOn;
    bool showGridOn;
    bool showSingularitiesOn;
    bool showMajorTenLine;
    bool isReceiveNewMap;
    double realWorld_to_field_scale;
    double realWorld_to_field_offset_x;
    double realWorld_to_field_offset_y;
    ////////////////////////

    std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point2f> > dirMap2Field;
    std::vector<std::vector<cv::Point2f> > dirMapInWorld;
    std::vector<std::vector<cv::Point2f> > contours2Field;
    std::vector<std::vector<cv::Point2f> > contoursInWorld;
    std::vector<std::vector<icVector2> > constraints_vecs;
    std::vector<cv::Point2f> recoverPoints;

    icVector2 m_robotDirect;
    icVector2 m_globalDirect;
    Seed *robot_loc;
    float robot_world_pos[2];
    bool recoverMode;
    bool ifReachTrisector;
    bool ifCancelPath;
    float leftBottom[2];
    float rightTop[2];
    std::vector<Location> bresenham_line_points;
    int selected_target_tri;

///
    //ros
    ros::NodeHandle nh_;
    tf::TransformListener listener;
    ros::Subscriber gridmap_sub;
    ros::Subscriber target_sub;
    ros::Subscriber first_point_sub;
    ros::Publisher safetyWayPointNum_pub;
    ros::Subscriber frontierPoints_sub;
    ros::Subscriber finishGoTri;
    ros::Subscriber finish_recover;
    ros::Subscriber finish_turn;
    image_transport::Publisher rgb_pub;
    image_transport::ImageTransport m_it;
    nav_msgs::OccupancyGrid dirMap;
    ros::ServiceClient goTriExecuteclient;
    ros::ServiceClient pathExecuteClient;
    ros::ServiceClient pathCancelClient;
    ros::ServiceClient pathCutClient;
    ros::ServiceClient pathRecoverClient;
    ros::ServiceClient turnDirectClient;
    ros::ServiceClient reqCalDegptPathsInfoClient;

    unsigned char  *rgb_im;
    std_msgs::Float64MultiArray frontierPoints;

    bool ifFinish_goTri;
    bool ifFinish_recover;
    bool ifFinish_turn;


    //
    //topo conection
    std::vector<DegeneratePt> validDegpts;
    std::vector<DegeneratePt> validTriDegpts;
    EvenStreamlinePlace *separatrices;
    std::vector<vector<int> > degptsPathsInfoGain;
    int targetTrisectorIndex;
    std::set<pair<int,int> > degptsLinks;

    //
    //key paramters
    float robotInitialPos_x;
    float robotInitialPos_y;
    std::string rgbTopicName;
    std::string gridMapTopicName;
    std::string worldFrameId;
    std::string baseFrameId;
    float safeDistance;
    int interFrameNum;
    float interFrameTime;
    //
	//////////////////////////////////////////////////////////////////////////
	// Operations for OpenGl window setting and displaying
public:
    //opengl
    void TfCoreInit();
	//////////////////////////////////////////////////////////////////////////
	int InitGL();	           //initialize the OpenGl envrionment
	void tensor_init_tex();
	void init_degpts();
	void init_regular_ten_designelems();
	//////////////////////////////////////////////////////////////////////////
	int DrawGLScene(GLenum mode);       //The same function as display routine to show the visual effect in the opengl window
	void cal_inverse_transform();
	void transform_fun();
	int without_anti_aliasing(GLenum mode);
	void make_tens_Patterns(void);
    void makePatterns(void);
	//////////////////////////////////////////////////////////////////////////
	void render_majorfield_quad();
	void major_vis_quad();
    void render_ibfv_tens_quad( bool x_y);
    void render_alpha_map_quad();
	void render_tensor_blend();
    void render_tensor_final();
	void drawSolidRect_size(double cx, double cy, double R);
	void drawSolidCircle_size(double cx, double cy, double R);
	void display_tenRegElem(GLenum mode);
	void display_design_grid();
	void display_degenerate_pts(GLenum mode);
    void display_valid_degenerate_pts(GLenum mode);
    void display_valid_trisectors(GLenum mode);
	void draw_hollow_circle_size(double cx, double cy, double R);
    void display_separatrices(GLenum mode);
//////////////////////////////////////////////////////////////////////////
    //tensor field
    void set_ten_regBasis(double x, double y, int type);
	unsigned char get_region_id(double x, double y);
    void set_ten_regDir(double x, double y);
    void set_ten_regDir(double x, double y,icVector2 &vec);
    void cal_tensorvals_quad();
    void get_tensor(double x, double y, double t[4]);
	void init_ten_designelems();
	void cal_all_eigenvecs_quad();
	void cal_eigenvecs_onevert_quad(int ver);
	void cal_eigen_vector_sym(icMatrix2x2 m, icVector2 ev[2]);
	void normalized_tensorfield_quad();
	void locate_degpts_cells_tranvec_quad(void);
	void compute_degpts_pos_tranvec_quad();
	void compute_onedegpt_pos_tranvec_quad(int cellid, double &x, double &y);
	void compute_a_alongx_degptlocate(double &a, icVector2 v0, icVector2 v1, icVector2 v2, icVector2 v3,
		icVector2 &v0v1, icVector2 &v2v3);
    void compute_separatrixVec_degpt_quad(int degpt_index);
	void display_major_tenlines(GLenum mode);
    void setRegularElems();

    //time-vary tensorfield
  void update_tensor_field();
  void init_keyframeList(int size = 2);
  void set_cur_as_keyframe(int frameID);
  void update_cur_TF_keyframe();
  void cal_TF_at_slice_keyframe_interp(int which_slice);
  void copy_from_keyframe(int keyframelistPos);
  void cal_TF_at_inbetween_slice_keyframe_interp(double ratio, int start, int end);
  void Laplace_relax_timeDep();
  void init_vertTimeList();
  void add_verts_to_vertTimeList();
  void set_constraints_keyframes();
  void count_nonconstrained_verts();
  void con_linear_Sys_spatioTemp(Vec_DP &tsa, Vec_INT &tija, Mat_DP &bound_v);
  void get_laplace_TVTF(int which_slice);


    /////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
    //ros
    void finishRecover_callback(const std_msgs::String &msg);
    void finishTurn_callback(const std_msgs::String &msg);
    void gridMap_callback(const nav_msgs::OccupancyGrid &msg);
    void goTri_callback(const std_msgs::String &msg);
    void frontier_point_callback(const std_msgs::Float64MultiArray &msg);
    void listen_to_robot_loc();
    void add_robot_wayPoints();
    void add_separatrix_points_toRobot(Trajectory * target_traj);
    void publish_image();
    void ensure_robot_safety();
    void cancelPath();
    void cut_robot_path();
    void recover_robot_state();
    void req_turn_service();

    //topo connection
    void filterDegpts();
    void gen_separatrix();
    void cal_sep_infoGain();
    bool check_bypass_trisector();
    bool check_reach_trisector();
    void display_trisectorVec(GLenum mode);
    void cal_separatrix_vec();
    void init_separatrices();
    void reset_separatrices();
    void  display_bresenham_line();
    bool check_line_insect_obstacle();
    bool insertLinks(int i, int j);
    std::map<pair<int ,int>,float> search_degpts_mst(std::set<int> connect_degpts, int target_tri_index);
    std::set<int> find_connect_degpts(int target_tri_index);
    Trajectory * select_a_branch(std::map<pair<int,int>,float> cur_mst, int target_tri_index);
    void select_target_trisector_branch_move();

    //parallel
    void create_paraThread();
    void parallel_update_tensorField();
    void split_tensorField( int i, int threadNum, int &x1, int &x2);
    void parallel_cal_tensorvals_quad(int &x1, int &x2);
      //

    ////////
    void set_obstacles();
    void play_all_frames();
    void draw_map_contour();
    void get_obstacles_contour();
    void reset_regular_and_degenrateElem();
    void gen_major_path();
    void reset_major_path();
    void set_robot_loc(double x,double y);
    void init_recover_points();
    void init_majorPath();
    void locat_point_inMap(double pos[2],cv::Point2i &mapLoc);
    bool check_reach_wedge();
    ////////

};



#endif
