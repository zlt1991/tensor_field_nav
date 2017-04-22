#ifndef _QUADMESH_H
#define _QUADMESH_H
#include "dataStructure.h"
class QuadMesh {
public:	
	int nverts, nfaces, nedges;   //number of vertices, faces, edges respectively
	QuadCell **quadcells;                      //list of faces
	QuadVertex **quad_verts;				   //list of vertices
	int XDIM, YDIM;
	double xinterval, yinterval;
	double xstart, xend, ystart, yend;

	double area;
	double radius;                         //radius of the bounding sphere of the object
	double ave_edge_length;
	icVector3 center;                      //the center of the object

	QuadEdge *elist;              //This is a different link from vertices and faces to store the unknown edges
	QuadEdge **edgelist;

	QuadMesh();
	~QuadMesh();
	QuadMesh(int xdim, int ydim, double xstart, double xend, 
		double ystart, double yend);
	bool gen_regquad_mesh(int xdim, int ydim, double xstart, double xend, 
		double ystart, double yend);
	bool gen_regquad_vertices(int xdim, int ydim, double xstart, double xend, 
		double ystart, double yend);
	void init_vertices();
	void gen_regquad_faces(int xdim, int ydim)  ;
	void finalize_quad_verts();
	void finalize_quad_cells();
	void construct_edges();
	QuadEdge  **Extend_Elist(QuadEdge **edge_link, int Num_edges);
	QuadCell  **extend_celllist_ver(QuadCell **cells, int ncells);
	void orient_edges_cells();
};



#endif // !_QUADMESH_H

