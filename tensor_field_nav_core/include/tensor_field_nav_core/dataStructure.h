#ifndef _DATASTRUCTURE_H
#define _DATASTRUCTURE_H
#include "icMatrix.h"
#include "icVector.h"
typedef struct RegularElem{
	int ID;
	double base[2];                //base under global frame
	icVector2 Direct;                  //direction in global frame
	int type;                       //singular type (0 –regular, 1—convergent, 2—divergent)
	icMatrix3x3  transform_matrix;  //transformation matrix for user editing
	icMatrix3x3 transposeRot;   
	//visual control icons (control points)
	//double rate of decreasing (optional) ---not consider at this moment

	int basis_triangle;            //the triangle that contains the basis

	////variables may be useful in editing
	double rotang;
	double s;
} RegularElement;

typedef struct Point
{
	double x, y;
	int cellid;
}Point;



typedef struct Singularities{
	double gcx, gcy;                 //global center
	float alpha[3];                  //the barycentric coordinates of the fixed point inside the triangle
	int Triangle_ID;                 //which triangle it belongs to
	double local_coordinates[3];     //we can choose to use 2D local coordinates or barycentric coordinates 
	int type;                        //singular type (0-–Null, 1—source, 2—sink, 3—saddle, 4—cwcenter, 
	//5—ccwcenter, 6—repel center, 7—attractive center )
	icMatrix3x3 Jacobian;            //local Jacobian matrix for this singularity
	icVector2 incoming, outgoing;    //for separatrices calculation beginning from a saddle
	//visual icons ( we need not store it, the display routine will use different colors to mark different kinds of singularities)

	/*---------Variables for Conley relation graph 10/14/05---------------*/
	int node_index;                  //the index of the node associates with this singularity
	int separtices;                  //the index of the separatrix group belongs to the saddle 10/13/05
	int selectedtraj;                //the index of selected separatrix for modification 12/23/05
	int *connected_limitcycles;      //An array of limit cycles that the saddle can reach
	int num_connected_limitcycles;   //number of the limit cycles inside the array 'connected_limitcycles'

	int connected;                   //1--connected with other element(s) 08/06/06
} Singularities;


typedef struct SampleListInTriangle{
	int which_traj;
	int limitcycle;
	int which_sample;
}SampleListInTriangle;


typedef struct QuadEdge{
	int index;                //Id for specific edge
	int verts[2];             //The two points for specific edge
	int tris[2];              //Two neighbor faces that share this edge
	bool visited;              //for my subdivision of the triangle mesh
	//double length;            //store the length of the edge

	/*---------Variables for pair cancellation---------------*/
	bool OnRepellBoundary;     //If the edge is at the boundary of repell region mark it as 1, otherwise 0 1/23/06
	bool OnAttractBoundary;    //If the edge is at the boundary of repell region mark it as 1, otherwise 0 1/23/06
	bool repell_visited;       //whether the edge has been visited during the boundary building
	bool attract_visited;
	icVector2 repell_normal;  //outward normal of the repeller region at the edge 
	icVector2 attract_normal; //outward normal of the attractor region at the edge 

	//icVector2 normal;

	////Variables for boundary edges list extraction
	bool OnBoundary;
	bool BoundaryVisited;

	/***---- For finding the separation and attachment points ----***/
	icMatrix2x2 Jacobian;     //for calculate the decomposition of the Jacobian

	/**----- For recording the intersections -----**/
	icVector2 intersections[2];  //we store only the recent two intersections
	int num_intersections;
	//double pre_length;

	QuadEdge *next;
}QuadEdge;

typedef struct LinesInOneCell{
	int whichtraj;    /*the index of the tensor line*/
	int start, end;  /*from the "start" line segement to the "end" one*/
}LinesInOneCell;
class LineInfo{
public:
	LinesInOneCell **lines;
	int nlines;
	int curMaxNum;

	LineInfo(int init_size = 0)
	{
		if (init_size == 0)
		{
			lines = NULL;
			nlines = curMaxNum = 0;
			return;
		}

		lines = (LinesInOneCell**)malloc(sizeof(LinesInOneCell*)*init_size);
		nlines = 0;
		if(lines == NULL)
			exit(-1);
		curMaxNum = init_size;
		for(int i=0; i<curMaxNum; i++)
			lines[i]=NULL;
	}

	~LineInfo()
	{
		int i;
		if(lines != NULL)
		{
			for(i=0; i<curMaxNum; i++)
			{
				if(lines[i] != NULL)
					free(lines[i]);
			}

			free(lines);
		}
	}

	void addNew(LinesInOneCell *l)
	{
		if(nlines>=curMaxNum)
		{
			if(!extend(1))
				exit(-1);  /*probably not enough memory*/
		}
		lines[nlines] = l;
		nlines++;
	}

	bool isFull()
	{
		if(nlines>=curMaxNum)
			return true;
		return false;
	}

	bool extend(int step)
	{
		LinesInOneCell **temp = lines;
		lines = (LinesInOneCell**)malloc(sizeof(LinesInOneCell*)*(curMaxNum+step));
		if(lines == NULL)
			return false;
		int i;
		for(i=0; i<curMaxNum; i++)
			lines[i]=temp[i];
		for(i=curMaxNum; i<curMaxNum+step; i++)
			lines[i]=NULL;
		curMaxNum += step;
		free(temp);
		return true;
	}

	bool is_repeated(LinesInOneCell *l)
	{
		int i;
		for(i=0;i<nlines;i++)
		{
			if(l->whichtraj==lines[i]->whichtraj)
				return true;
		}
		return false;
	}
};

typedef struct QuadCell {
	unsigned char nverts;        // number of vertices in the list 
	int *verts;                  // indices of the vertices associated with current face list 
	int index;                  //index of current face
	void *other_props;           //other properties 
	//double area;                 //area of the face
	//icVector3 normal;           //normal of the face
	//icVector3 center;           //

	QuadEdge *edges[4];             //store the address of the associated edges
	int xstart, xend, ystart, yend;
	double x_start_coord, y_start_coord;

	/*---------Variables for pair cancellation---------------*/
	//bool repell_inregion;        //whether the triangle is inside the repeller neighborhood
	//bool attract_inregion;       //whether the triangle is inside the attractor neighborhood

	bool contain_degpt;    //flag to mark whether this triangle containing a singularity or not
	int degpt_index;      //id of the singularity contained by current triangle

	bool OnBoundary;

	/*----------For degenerate point detection (temp) ---------- 09/18/2007 */
	unsigned char degenerate_type;

	/*------------For streamline placement, using samplin point-----------*/ //copy at 07/22/06
	////SampleListInTriangle *samplepts;
	//////int *sampleindex;
	////int num_samplepts;
	////int MaxSampNum;

	SampleListInTriangle *maj_samplepts, *min_samplepts;
	int maj_nsamplepts, min_nsamplepts;
	int MAJMaxSampNum, MINMaxSampNum;

	bool visited;

	/*----- For calculating the intersections and constructing the street net*/
	LineInfo *majorlines, *minorlines;
	bool hasmajor, hasminor;

	int *intersectlist;   /*a list of intersections in this cell*/
	unsigned char nintersects;      /*the number of the intersections in the list*/

	int *streetgraphedgelist;
	unsigned char nstreetgraphedges;

	/* for region division 11/22/2007*/
	LineInfo *sketchlines;
	unsigned char which_region;

	bool in_region;
	//bool in_veg;                  /*true -- in veg region;  false -- in regular land or water */
	bool is_contour_cell;

	/*  record the water boundaries that cross this cell  */
	int *mapbounds;
	int nmapbounds;

} QuadCell;

typedef struct TenRegularElem{
	int ID;
	double base[2], end[2];                //base under global frame
	icVector2 Direct;                  //direction in global frame
	unsigned char type;                       //singular type (0 –regular, 1—convergent, 2—divergent)
	icMatrix3x3  transform_matrix;  //transformation matrix for user editing
	icMatrix3x3  transpose_matrix;

	//icMatrix3x3 transposeRot;   
	//visual control icons (control points)
	//double rate of decreasing (optional) ---not consider at this moment

	int basis_triangle;            //the triangle that contains the basis

	////variables may be useful in editing
	//double originalang;
	double rotang;
	double s;

	bool deleted;

	////
	unsigned char which_region;    //for two level tensor field design 11/17/2007
}TenRegularElem;

typedef struct QuadVertex {
	double x,y/*,z*/;             //the coordinates of the vertex under global frame
	double vx,vy/*,vz*/;
	double prob_on_path;
	void *other_props;        //other properties 
	//icVector3 normal;         //normal at the vertex

	int *edges_id;            //edges incident to current vertex
	unsigned char Num_edge;             //number of edges
	QuadEdge **edges;             //store the addresses of the edges incident to the vertex

    unsigned char ncells;     //number of cells that share this vertex
	QuadCell **cells;         //the cells that share this vertex
	
	/*---------Variables for region smoothing---------------*/
	bool OnBoundary; 
	bool InRegion;
	int index;               //*Index in the whole object vertices list
	int RegionListID;         //Index in the inner vertices list
	
	/*---------Variables for pair cancellation---------------*/
	//unsigned char repell_flag;          //for repeller neighborhood | 0--unknown, 1--on boundary, 2-- in region
	//unsigned char attract_flag;         //for attractor neighborhood | 0--unknown, 1--on boundary, 2-- in region

	////For geodesic based search, now it is for limit cycle relocation
	int which_line;
	double distance;  /*for distance level set as well*/
	bool visited;
	unsigned char type;      /*0--far away; 1--in narrow band; 2--known 09/30/2007*/

	icMatrix2x2 Jacobian;    /*we will calculate it according to the Jacobian around its one ring neighbor*/
	icMatrix2x2 origin_ten;  /*the old Jacobian*/
	/*eigen vectors*/
	icVector2 major, minor;  /*for visualization only*/
	double major_ang, minor_ang;
	double tensor_major_ang;
	bool major_cos, major_sin, minor_cos, minor_sin;
	icVector2 tran_vec;      /*transfer eigen vector to a real vector field*/

	bool inland;             /*true -- in land; false -- in the water region*/
	bool inveg;              /*true -- in veg region;  false -- in regular land or water */

	unsigned char which_region;  /*for two level design 11/17/2007*/
	bool inbrushregion;

	/*   for multi-density tensor line placement   */
	double density;

	/*   for the asymmetric tensor field design 12/29/2007  */
	double phi;

} QuadVertex;


typedef struct EditBox{
	icVector2 p1, p2, p3, p4, Up;  ////stands for the 4 points of the edit box
}EditBox;

typedef struct SingularElem{
	int ID;
	int Triangle_ID;                //the triangle that contains the singular element
	double centerx, centery;        //center under global frame
	int type;                       //singular type (0 –Null, 1—source, 2—sink, 3—saddle, 4—cwcenter, 
	//5—ccwcenter
	icMatrix3x3  transform_matrix;  //transformation matrix for user editing
	icMatrix3x3  Jacobian;
	//visual control icons ( edit box, control points)
	EditBox editbox;
	EditBox cur_editbox;
	//double rate of decreasing (optional) ---not consider at this moment

	////variables may be useful in editing
	double rotang;
	double sx, sy;
	double s;
	bool deleted;
} SingularElement;

typedef struct DegeneratePt{
	int degpt_index;
	double gcx, gcy;                 //global center
	float alpha[3];                  //the barycentric coordinates of the fixed point inside the triangle
	int Triangle_ID;                 //which triangle it belongs to
	double local_coordinates[3];     //we can choose to use 2D local coordinates or barycentric coordinates 
	int type;                        //degenerate type (0—wedge, 1—trisector, 2—node, 3—center, 
	//4—saddle, 5—other higher order)
	icMatrix2x2 ten;                 //local tensor for this degenerate point
	double s1_ang, s2_ang, s3_ang;   //the starting angles of the separatrices
	icVector2 s[3];
    icVector2 s_vec[3];
    bool s_ifLink[3];
    int nseps;             //the number of separatrices
    int nlinks;
    int links_index[3];
    int valid_index;
	//icVector2 s1, s2, s3;    //for separatrices calculation beginning from a saddle
	//visual icons ( we need not store it, the display routine will use different colors to mark different kinds of singularities)

	bool deleted;
} DegeneratePt;

typedef struct ScalarSingularElem{
	int index;
	double pos[2];
	unsigned char type;
	bool deleted;

	/*  for editing  */
}ScalarSingularElem;

typedef struct Degenerate_Design{
	int ID;
	int Triangle_ID;                //the triangle that contains the singular element
	double centerx, centery;        //center under global frame
	int type;                       //0 ?wedge, 1—trisector, 2—node, 3—center, 4—saddle, 
	//5—ccwcenter
	icMatrix3x3  transform_matrix;  //transformation matrix for user editing
	//icMatrix3x3  Jacobian;
	/*visual control icons ( edit box, control points)*/
	EditBox editbox;
	EditBox cur_editbox;
	//double rate of decreasing (optional) ---not consider at this moment

	////variables may be useful in editing
	double rotang;
	double sx, sy;
	double s;
	bool deleted;

	////
	unsigned char which_region;    //for two level tensor field design 11/17/2007
}Degenerate_Design;

typedef struct Edge{
	int index;                //Id for specific edge
	int verts[2];             //The two points for specific edge
	int OppVerts[2];          //The two vertices that opposite to the edge
	int tris[2];              //Two neighbor faces that share this edge
	int visited;              //for my subdivision of the triangle mesh
	int MidPointID;           //The ID of the vertex of the middle point on the edge
	double length;            //store the length of the edge

	/*---------Variables for pair cancellation---------------*/
	int OnRepellBoundary;     //If the edge is at the boundary of repell region mark it as 1, otherwise 0 1/23/06
	int OnAttractBoundary;    //If the edge is at the boundary of repell region mark it as 1, otherwise 0 1/23/06
	int repell_visited;       //whether the edge has been visited during the boundary building
	int attract_visited;
	icVector2 repell_normal;  //outward normal of the repeller region at the edge 
	icVector2 attract_normal; //outward normal of the attractor region at the edge 

	icVector2 normal;

	////Variables for boundary edges list extraction
	int OnBoundary;
	int BoundaryVisited;

	/*----- To store the special points on the edge (two at most) 07/17/06 ----*/
	icVector2 attp, sep;
	int find_attp, find_sep;
	int att_visit, sep_visit;    //0 -- alive; 1--visited/dead;  2--too close/pending  07/23/06

	/***----For testing the SCC----***/
	int mixed;

	/***---- For finding the separation and attachment points ----***/
	icVector2 evec[2];
	icMatrix2x2 Jacobian;     //for calculate the decomposition of the Jacobian
	int valid;

	/**----- For recording the intersections -----**/
	icVector2 intersections[2];  //we store only the recent two intersections
	int num_intersections;
	//double pre_length;

	Edge *next;
}Edge;

typedef struct Vertex {
	double x,y,z;             //the coordinates of the vertex under global frame
	double bx, by;        //jitter the coordinates a little bit
	double nx,ny,nz;
	double prob_on_path;
	void *other_props;        //other properties 
	icVector3 normal;         //normal at the vertex

	int *edges_id;            //edges incident to current vertex
	int Num_edge;             //number of edges
	Edge **edges;             //store the addresses of the edges incident to the vertex
	
	/*---------Variables for corner table---------------*/
	int *Corners;             //store the indexes of those corners associate with the vertexs
	int Num_corners;          //Number of the corners

	/*---------Variables for region smoothing---------------*/
	int OnBoundary; 
	int InRegion;
	int VertID;               //*Index in the whole object vertices list
	int RegionListID;         //Index in the inner vertices list
	
	/*---------Variables for pair cancellation---------------*/
	int repell_flag;          //for repeller neighborhood | 0--unknown, 1--on boundary, 2-- in region
	int attract_flag;         //for attractor neighborhood | 0--unknown, 1--on boundary, 2-- in region

	////For geodesic based search, now it is for limit cycle relocation
	int which_line;
	double distance;
	//int visited;
	bool visited;
	unsigned char tau_visited;

	/*----------Variable for finding the separation and attachment points --------------*/
	icVector2 vec;           //variable to store the vector on that vertex under global frame
	icVector2 vec_J;
	icVector2 evec[2];
	double length[3];

	icMatrix2x2 Jacobian;    /*we will calculate it according to the Jacobian around its one ring neighbor*/
	/*eigen vectors*/
	icVector2 major, minor;  /*for visualization only*/
	double major_ang, minor_ang;
	double tensor_major_ang;
	bool major_cos, major_sin, minor_cos, minor_sin;
	icVector2 tran_vec;      /*transfer eigen vector to a real vector field*/

	double mag_speed;

	/*------------------------------------------------------*/
	int *connected_limitcycle;
	int num_connectedlimitcycle;
	
	/*------------For local tracing----------------*/
	double *Anglist;   //store the angle allocated for each triangle on the tangent plane

	/*------------For adaptive \tau-----------*/ //03/15/06
	float tau[2];  /*previous two \tau for adaptive adjust \tau*/
	icVector2 endp[2];    /*the global coordinates of the end point*/
	int end_tri[2];       /*the triangle contains the end point*/
	bool end1_ornot;      /*tell program which tau it should use, 0--tau[0], 1--tau[1]*/
	bool done;           /*mark if it has been traced*/

	/*---- For asymmetric method----*/
	float gama_d, gama_r, gama_s;
	float hue, saturation;

	/*--- color mapping ---*/
	float vf_r, vf_g, vf_b;

	float f_color[3], b_color[3];

	float d_color[3];  /*for density visualization*/
	int s_count;       /*the number of the samples associated with this vertex*/


} Vertex;

typedef struct Face {
	unsigned char nverts;        // number of vertices in the list 
	int *verts;                  // indices of the vertices associated with current face list 
	void *other_props;           //other properties 
	double area;                 //area of the face
	double length[3], angle[3];  //???
	int nvisible;
	icVector3 normal;           //normal of the face
	//icVector3 center;           //

	Edge *edges[3];             //store the address of the associated edges

	//////The following members are used to define the local frame for each triangle
	int index;                  //index of current face

	icVector2 LX;               //local frame , x axis
	icVector2 LY;               //local frame,  y axis

	double xy[3][2];            //store the coordinates of its three vertices under local frame

	icVector2 direct_vec[3];    //store the directional vectors in local frame, we have 3 vectors for each triangle

	icMatrix2x2 Jacobian;       //every triangle mesh will have a local linear vector field (Jacobian Matrix)

	/*---------Variables for pair cancellation---------------*/
	int repell_inregion;        //whether the triangle is inside the repeller neighborhood
	int attract_inregion;       //whether the triangle is inside the attractor neighborhood

	int contain_singularity;    //flag to mark whether this triangle containing a singularity or not
	int singularity_index;      //id of the singularity contained by current triangle

	////Variables for limit cycle shape design
	int inDesignCellCycle;

	int contain_separatrix;     //for separatrix editing (grow region)1/3/06
	int fence_flag;             //to set fence to prevent the region growing crossing this triangle

	////Variables for new limit cycle detection 1/24/06
	int access_count;           //record the times that the local tracing accessing
	int discard;                //if it is not a good condition triangle, reuse it as boundary flag 3/16/06

	/*------------For streamline placement-----------*/ //2/18/06
	int *trajlist;              //a list of the trajectory indices that pass through the triangle
	int num_passing_trajs;      //number of the trajectories passing through the triangle

	/*----------------------------------------------*/
	int which_SSC, pre_SCC;              //which strongly component it belongs to

	/*----------For the Jacobian of the triangle--------- 07/17/06 */
	icVector2 eigen[2];
	double evalues[2];
	double center[2];           //store the center of the triangle

	/*----------For degenerate point detection (temp) ---------- 09/18/2007 */
	unsigned char degenerate_type;

	/*------------For streamline placement, using samplin point-----------*/ //copy at 07/22/06
	SampleListInTriangle *samplepts;
	int *sampleindex;
	int num_samplepts;
	int MaxSampNum;

} Face;

typedef struct Corner{
	int index;
	double angle;
	Edge *edge[2];    //two edges associated with this corner

	/////The corner operation 
	int v;            //the ID of the vertex of the corner
	int n;
	int p;
	int t;            //the triangle the corner belongs to
	Edge *e;          //the opposite edge of the corner
	int o;            //the index of its opposite corner
	int ot;           //the index of its opposite triangle for traversal 2/9/05

	int Edge_count;   //special variable for edges search 1/21/05

	/*----------------------------------------------------------*/
	////variables for singularities detection and local tracing
	double BeginAng, EndAng;
	double r;
	int orient;
}Corner;

typedef struct Polygon3D {
	/*unsigned */int nverts, nfaces, nedges;   //number of vertices, faces, edges respectively
	Vertex **vlist;                        //list of vertices
	Face **flist;                          //list of faces
	//PlyOtherProp *vert_other,*face_other;  
	double area;
	double radius;                         //radius of the bounding sphere of the object
	double ave_edge_length;
	icVector3 center;                      //the center of the object

	Edge *elist;              //This is a different link from vertices and faces to store the unknown edges
	Edge **edgelist;

	////New added Corner information 08/17/05
	Corner **clist;
	int ncorners;

} Polygon3D;


// Streamline
typedef struct Seed{
	//double pos[3];                //the coordinates of the seed point
	double pos[2];                  //the coordinates of the seed point
	int triangle;                   //the triangle contains the seed point
	//int which_traj;                 //record which trajectory it locates on
	unsigned char state;            //0—active;1—inactive; 2—can not growing
	double weight;
}Seed; // end of Seed structure

enum road_type{
	MINOR,
	MAJOR,
	HIGHWAY,
	BOUNDARY
	//FREEWAY
	//PLAIN,
};



typedef struct LineSeg{
	int Triangle_ID;                    //which triangle this line segment locates in
	double start[2], end[2];            //local coordinates for start and end points
	double gstart[2], gend[2];          //global coordinates for start and end points
	double length;                      //we may need to store the length of the line segment
} LineSeg;

typedef struct CurvePoints{
	double lpx, lpy;
	double gpx, gpy;
	double length;
	int triangleid;
}CurvePoints;




class SeedList{
public:
	Seed **seeds;
	int nseeds;
	int curMaxNumSeeds;
	double max_weight, min_weight;

	// The similar list operations
	SeedList(int initsize = 3000) //construction
	{
		seeds = (Seed **)malloc(sizeof(Seed *)*initsize);
		curMaxNumSeeds = initsize;
		nseeds = 0;

		if(seeds == NULL)
		{
			//char rout[256], var[256];
			//sprintf(rout, "%s", "SeedList Constructor");
			//sprintf(var, "%s", "seeds");

			//write_mem_error(rout, var, 0);
			curMaxNumSeeds = 0;
			exit(-1);

		}
		int i;
		for(i = 0; i < initsize; i++)
			seeds[i] = NULL;
	} 

	~SeedList()
	{
		int i;
		for(i = 0; i < curMaxNumSeeds; i++)
		{
			if(seeds[i] != NULL)
				free(seeds[i]);
		}
		free(seeds);
	}

	inline void copy_otherseedList(SeedList *otherseeds)
	{
		int i;
		if(otherseeds->nseeds>curMaxNumSeeds)
		{
			if(!extend(otherseeds->nseeds-curMaxNumSeeds))
				exit(-1);
		}

		/*copy element by element*/
		for(i=0;i<otherseeds->nseeds;i++)
		{
			if(seeds[i]==NULL)
				seeds[i]=(Seed *) malloc(sizeof(Seed));

			seeds[i]->pos[0]=otherseeds->seeds[i]->pos[0];
			seeds[i]->pos[1]=otherseeds->seeds[i]->pos[1];

			seeds[i]->triangle=otherseeds->seeds[i]->triangle;
			seeds[i]->state=otherseeds->seeds[i]->state;
			seeds[i]->weight=otherseeds->seeds[i]->weight;
		}
		nseeds=otherseeds->nseeds;
	}

	inline void copyandappend_otherseedList(SeedList *otherseeds)
	{
		int i;
		if(otherseeds->nseeds+nseeds>curMaxNumSeeds)
		{
			if(!extend(otherseeds->nseeds+nseeds-curMaxNumSeeds))
				exit(-1);
		}

		/*copy element by element*/
		for(i=nseeds;i<nseeds+otherseeds->nseeds;i++)
		{
			if(seeds[i]==NULL)
				seeds[i]=(Seed *) malloc(sizeof(Seed));

			seeds[i]->pos[0]=otherseeds->seeds[i-nseeds]->pos[0];
			seeds[i]->pos[1]=otherseeds->seeds[i-nseeds]->pos[1];

			seeds[i]->triangle=otherseeds->seeds[i-nseeds]->triangle;
			seeds[i]->state=otherseeds->seeds[i-nseeds]->state;
		}
		nseeds+=otherseeds->nseeds;
	}

	//add a new vertex to the end of the list, if it succeeds, return true
	inline bool append(Seed *s)
	{
		if(isFull ())
			if(!extend(100))
				return false;             //if not enough memory available, return false
		seeds[nseeds] = s;
		//copyElem(s, polist[nporbits]);
		nseeds++;
		return true;
	} 

	/*  insert a new seed to the proper position of the list according to its weight  */
	inline bool sorted_add(Seed *s)
	{
		if(nseeds>=curMaxNumSeeds)
		{
			if(!extend())
				//exit(-1);
					return false;
		}

		/*sorted insert!*/
		int i, pos=0;
		for(i=0; i<nseeds; i++)
		{
			if(s->weight>seeds[i]->weight)
			{
				pos = i;
				break;
			}
		}
		if(i>=nseeds) pos=nseeds;
		for(i=nseeds; i>pos; i--)
			seeds[i]=seeds[i-1];
		seeds[pos]=s;

		nseeds ++;
		return true;
	}

	inline void update_max_min_weights()
	{
		int i;
		if(nseeds==0) return;
		max_weight=min_weight=seeds[0]->weight;
		for(i=1;i<nseeds;i++)
		{
			if(seeds[i]->weight>max_weight) max_weight=seeds[i]->weight;
			if(seeds[i]->weight<min_weight) min_weight=seeds[i]->weight;
		}
	}

	inline bool del_End() //delete the vertex at the end of the list
	{
		if(isEmpty())  return false;
		nseeds --;
		return true;
	} 

	inline void copy_Elem(Seed *to, Seed *from)
	{
		to->pos[0]=from->pos[0];
		to->pos[1]=from->pos[1];
		to->triangle=from->triangle;
		to->state=from->state;
	}

	//delete the corresponding  vertex, if it succeeds, return true
	inline bool del_Node(Seed *s) 
	{
		if(isEmpty())  return false;

		//find the vertex, if find it, delete and move the following vertices forward
		//otherwise, return false;

		int i, pos = -1;

		for(i = 0; i < nseeds; i++)
		{
			if(seeds[i] == s)
			{
				pos = i;
				break;
			}
		}

		if(pos == -1) return false;

		//delete it
		for(i = pos; i < nseeds-1; i++)
		{
			//we need a copy function
			copy_Elem(seeds[i], seeds[i+1]);
		}

		nseeds--;

		return true;
	} 

	inline bool del_Node_byindex(int pos) 
	{
		int i;
		if(isEmpty())  return false;

		//find the vertex, if find it, delete and move the following vertices forward
		//otherwise, return false;

		if(pos == -1) return false;

		//delete it
		for(i = pos; i < nseeds-1; i++)
		{
			//we need a copy function
			copy_Elem(seeds[i], seeds[i+1]);
		}

		nseeds--;

		return true;
	} 

	inline bool isEmpty()  //judge whether the list is empty
	{
		if(nseeds == 0)   return true;
		return false;
	}

	inline bool isFull()
	{
		if(nseeds == curMaxNumSeeds) return true;
		return false;
	}

	//extend the original list, if it succeeds, return true
	inline bool extend(int step = 100)
	{
		Seed **temp = seeds;
		seeds = (Seed **) malloc(sizeof(Seed *) * (curMaxNumSeeds + step));
		if( temp == NULL)
		{
			//fail
			//char rout[256], var[256];
			//sprintf(rout, "%s", "SeedList::extend");
			//sprintf(var, "%s", "seeds");

			//write_mem_error(rout, var, 1);
			curMaxNumSeeds = 0;
			seeds = temp;
			exit(-1);
			return false;
		}

		int i;
		for(i = 0; i < curMaxNumSeeds; i++)
			seeds[i] = temp[i];

		for(i=curMaxNumSeeds; i<curMaxNumSeeds+step; i++)
			seeds[i]=NULL;

		curMaxNumSeeds += step;

		free(temp);
		return true;
	}

	inline void reset()
	{
		nseeds = 0;
	}
}; //end of SeedList class

/** Sample point structure **/
typedef struct SamplePt{
	//double gpt[3];
	double gpt[2];
	int triangle;
	int traj;                                //which trajectory this sample falls on
}SamplePt; //end of SamplePt class

class SamplePtList{
public:
	SamplePt **samples;
	int nsamples;
	int curMaxNumSamplePts;

	SamplePtList(int initsize = 1000) //construction
	{
		samples = (SamplePt **)malloc(sizeof(SamplePt *)*initsize);
		curMaxNumSamplePts = initsize;
		nsamples = 0;

		if(samples == NULL)
		{
			//char rout[256], var[256];
			//sprintf(rout, "%s", "SamplePtList Constructor");
			//sprintf(var, "%s", "samples");

			//write_mem_error(rout, var, 0);
			curMaxNumSamplePts = 0;
			exit(-1);
		}

		for(int i = 0; i < initsize; i++)
			samples[i] = NULL;
	} 

	~SamplePtList()
	{
		if(samples!= NULL)
		{
			for(int i = 0; i < curMaxNumSamplePts; i++)
			{
				if(samples[i] != NULL)
					free(samples[i]);
			}
			free(samples);
		}
	}

	//add a new vertex to the end of the list, if it succeeds, return true
	inline bool append(SamplePt *s)
	{
		if(isFull ())
			if(!extend(100))
				return false;             //if not enough memory available, return false
		samples[nsamples] = s;
		//copyElem(s, polist[nporbits]);
		nsamples++;
		return true;
	} 

	inline bool del_End() //delete the vertex at the end of the list
	{
		if(isEmpty())  return false;
		nsamples --;
		return true;
	} 

	inline void copy_Elem(SamplePt *s, SamplePt *d)
	{
	}

	//delete the corresponding  vertex, if it succeeds, return true
	inline bool del_Node(SamplePt *s) 
	{
		if(isEmpty())  return false;

		//find the vertex, if find it, delete and move the following vertices forward
		//otherwise, return false;

		int i, pos = -1;

		for(i = 0; i < nsamples; i++)
		{
			if(samples[i] == s)
			{
				pos = i;
				break;
			}
		}

		if(pos == -1) return false;

		//delete it
		for(i = pos; i < nsamples-1; i++)
		{
			//we need a copy function
			copy_Elem(samples[i], samples[i+1]);
		}

		nsamples--;

		return true;
	} 

	inline bool isEmpty()  //judge whether the list is empty
	{
		if(nsamples == 0)   return true;
		return false;
	}

	inline bool isFull()
	{
		if(nsamples == curMaxNumSamplePts) return true;
		return false;
	}

	//extend the original list, if it succeeds, return true
	inline bool extend(int step = 100)
	{
		SamplePt **temp = samples;
		samples = (SamplePt **) malloc(sizeof(SamplePt *) * (curMaxNumSamplePts + step));
		if( temp == NULL)
		{
			//fail
			//char rout[256], var[256];
			//sprintf(rout, "%s", "SamplePtList::extend");
			//sprintf(var, "%s", "samples");

			//write_mem_error(rout, var, 1);
			curMaxNumSamplePts = 0;
			samples = temp;
			exit(-1);
			return false;
		}

		int i;
		for(i = 0; i < curMaxNumSamplePts; i++)
			samples[i] = temp[i];
		for(i = curMaxNumSamplePts; i < curMaxNumSamplePts+step; i++)
			samples[i] = NULL;
		curMaxNumSamplePts += step;

		free(temp);
		return true;
	}

	inline void reset()
	{
		nsamples = 0;
	}
}; //end of SamplePtList class

class DynList_Int{
public:
	//member variables
	int *elems;
	int  nelems;
	int  curMaxNum;
	int  extendstep;

	//member functions
	//constructions and destruction
	DynList_Int(int MaxNum = 500) 
	{
		elems = new int [MaxNum];
		curMaxNum = MaxNum;
		nelems = 0;
	}
	~DynList_Int()
	{delete [] elems; }

	inline bool extend(int step = 100) 
	{
		int *temp = elems;
		elems = new int[curMaxNum + step];
		if(elems == NULL)
		{
			elems = temp;
			return false;
		}

		for(int i = 0; i < curMaxNum; i++)
			elems[i] = temp[i];

		curMaxNum += step;

		delete [] temp;
		return true;
	}

	inline bool add_New(int t)
	{
		if(is_Full())
		{
			if(!extend())
			{
				return false;
			}
		}

		if(!is_repeated(t))
		{
			elems[nelems] = t;
			nelems++;
			return true;
		}

		return false;
	}

	inline bool add_New_2(int t)
	{
		if(nelems>=curMaxNum)
		{
			if(!extend())
			{
				return false;
			}
		}

		//if(!is_repeated_elem(elems, t, nelems))
		//{
		//	elems[nelems] = t;
		//	nelems++;
		//	return true;
		//}
		elems[nelems] = t;
		nelems++;
		return true;

		return false;
	}

	inline bool is_Full()
	{
		return nelems>=curMaxNum;
	}

	inline bool  del_Elem(int t)
	{
		int i, pos = 0;
		for(i = 0; i < nelems; i++)
		{
			if(elems[i] == t)
			{
				pos = i;
				break;
			}
		}
		if(pos >= nelems)
			return false;

		for(i = pos; i < nelems-1; i++)
			elems[i] = elems[i+1];

		nelems--;
		return true;
	}


	inline bool  del_Last()
	{
		if(nelems <= 0) return false;
		nelems--;
		return true;
	}

	inline bool  is_repeated(int newelem)
	{
		int i;
		for(i=0; i<nelems; i++)
		{
			if(elems[i] == newelem)
				return true;
		}
		return false;
	}
};
















/** Sample point structure **/


//
#endif // !1
