/***
 Basic data structure for tensor field
 ***/
#include "tensor_field_nav_core/QuadMesh.h"

QuadMesh::QuadMesh()
{
	quad_verts = NULL;
	quadcells = NULL;
	elist = NULL;
	edgelist = NULL;

	nverts = nfaces = nedges = 0;   //number of vertices, faces, edges respectively
}


QuadMesh::QuadMesh(int xdim, int ydim, double xstart, double xend, 
				   double ystart, double yend)
{
	quad_verts = NULL;
	quadcells = NULL;
	elist = NULL;
	edgelist = NULL;

	nverts = nfaces = nedges = 0;   //number of vertices, faces, edges respectively

	gen_regquad_mesh(xdim, ydim, xstart, xend, ystart, yend);

	/*construct edge list*/
	construct_edges();
	orient_edges_cells();

	XDIM = xdim;
	YDIM = ydim;
	this->xstart = xstart;
	this->xend = xend;
	this->ystart = ystart;
	this->yend = yend;
}
QuadMesh::~QuadMesh()
{
	finalize_quad_verts();
	finalize_quad_cells();
}


void QuadMesh::finalize_quad_verts()
{
	int i;
	if(quad_verts != NULL)
	{
		for(i=0; i<nverts; i++)
		{
			if(quad_verts[i]!=NULL)
			{
				if(quad_verts[i]->edges != NULL)
					free(quad_verts[i]->edges);
				free(quad_verts[i]);
			}
		}
		free(quad_verts);
	}
}

/*generate a regular quad mesh*/
bool QuadMesh::gen_regquad_mesh(int xdim, int ydim, double xstart, double xend, 
				   double ystart, double yend)
{
	/*generate the vertex list first*/
	if(gen_regquad_vertices(xdim, ydim, xstart, xend, ystart, yend))
	{
		/*construct the faces to obtain the corresponding connectivity information*/
		gen_regquad_faces(xdim, ydim);

		return true;
	}
	else
		return false;
}


/*generate the vertex list of the quad mesh*/
bool QuadMesh::gen_regquad_vertices(int xdim, int ydim, double xstart, double xend, 
				   double ystart, double yend)
{
	if(xstart > xend || yend < ystart) return false;
	int i, j;
	double d_x, d_y, x_coord, y_coord;
	xinterval = d_x = (xend-xstart)/(xdim-1);
	yinterval = d_y = (yend-ystart)/(ydim-1);
	int quad_vert_id;

	/*first, allocate the memory for the quad mesh*/
	if(quad_verts != NULL)
		finalize_quad_verts();
	nverts = xdim*ydim;
	quad_verts = (QuadVertex**)malloc(sizeof(QuadVertex*)*nverts);
	for(i=0; i<nverts; i++)
		quad_verts[i] = (QuadVertex*)malloc(sizeof(QuadVertex));

	for(j=0; j<ydim; j++)
	{
		y_coord = ystart+j*d_y; /*obtain y coordinates for jth row*/
		for(i=0; i<xdim; i++)
		{
			quad_vert_id = j*xdim+i;
			x_coord = xstart+i*d_x; /*obtain x coordinates for ith column*/
			quad_verts[quad_vert_id]->index = quad_vert_id;
			quad_verts[quad_vert_id]->x = x_coord;
			quad_verts[quad_vert_id]->y = y_coord;
		}
	}

	init_vertices();

	return true;
}


/*intialize the vertex list in the very beginning*/
void QuadMesh::init_vertices()
{
	int i;
	QuadVertex *qv;
	for(i=0; i<nverts; i++)
	{
		qv = quad_verts[i];
		qv->edges = NULL;
		qv->Num_edge = 0;
		qv->InRegion = false;
		qv->distance = 1.e50;
		qv->OnBoundary = false;
		qv->RegionListID = -1;
		//qv->repell_flag = qv->attract_flag = 0;
		qv->visited = false;
		qv->Jacobian.set(0.);
		qv->major.set(0.);
		qv->minor.set(0.);
		qv->ncells = 0;
		qv->cells = NULL;

		qv->inland = true;
		qv->inveg = false;

		qv->which_region=0;
		qv->inbrushregion=false;

		qv->phi=0.;

		qv->density=1.;
	}
}


/*finalize the cell list
*/
void QuadMesh::finalize_quad_cells()
{
	int i;
	if(quadcells != NULL)
	{
		for(i=0; i<nfaces; i++)
		{
			if(quadcells[i] != NULL)
			{
				free(quadcells[i]->verts);
				free(quadcells[i]);
			}
		}
		free(quadcells);
		quadcells=NULL;
	}
}


/*compute the faces/connectivities of the regular quad mesh
NOTE: this routine should be called after generating the vertex list
*/
void QuadMesh::gen_regquad_faces(int xdim, int ydim)
{
	int i, j;
	QuadCell *f;

	/*allocate space for the quad cell list*/
	if(quadcells != NULL)
	{
		finalize_quad_cells();
	}

	nfaces = (xdim-1)*(ydim-1);
	quadcells = (QuadCell**)malloc(sizeof(QuadCell*)*nfaces);
	for(i=0; i<nfaces; i++)
	{
		quadcells[i] = (QuadCell*)malloc(sizeof(QuadCell));
		quadcells[i]->verts = (int*)malloc(sizeof(int)*4);
	}

	int cell_index = 0;

	for(j=0; j<ydim-1; j++)
	{
		for(i=0; i<xdim-1; i++)
		{
			cell_index = j*(xdim-1)+i;
			quadcells[cell_index]->verts[0] = j*xdim+i;
			quadcells[cell_index]->verts[1] = j*xdim+(i+1);
			quadcells[cell_index]->verts[2] = (j+1)*xdim+(i+1);
			quadcells[cell_index]->verts[3] = (j+1)*xdim+i;
			quadcells[cell_index]->nverts=4;
			quadcells[cell_index]->index = cell_index;

			/**/
			quadcells[cell_index]->xstart = i;
			quadcells[cell_index]->xend = i+1;
			quadcells[cell_index]->ystart = j;
			quadcells[cell_index]->yend = j+1;
			quadcells[cell_index]->x_start_coord = quad_verts[quadcells[cell_index]->verts[0]]->x;
			quadcells[cell_index]->y_start_coord = quad_verts[quadcells[cell_index]->verts[0]]->y;

			quadcells[cell_index]->degpt_index = -1;

			quadcells[cell_index]->OnBoundary=false;
			quadcells[cell_index]->visited=false;
			//quadcells[cell_index]->repell_inregion=false;
			//quadcells[cell_index]->attract_inregion=false;

			/*for even tensor line placement*/
			//quadcells[cell_index]->samplepts=NULL;
			//quadcells[cell_index]->num_samplepts = 0;
			quadcells[cell_index]->maj_samplepts=NULL;
			quadcells[cell_index]->maj_nsamplepts = 0;
			quadcells[cell_index]->MAJMaxSampNum = 0;
			quadcells[cell_index]->min_samplepts=NULL;
			quadcells[cell_index]->min_nsamplepts = 0;
			quadcells[cell_index]->MINMaxSampNum = 0;

			/*for the computation of the intersections 10/02/2007*/
			quadcells[cell_index]->majorlines = NULL;
			quadcells[cell_index]->hasmajor = false;
			quadcells[cell_index]->minorlines = NULL;
			quadcells[cell_index]->hasminor = false;

			quadcells[cell_index]->sketchlines = NULL;

			quadcells[cell_index]->intersectlist = NULL;
			quadcells[cell_index]->nintersects = 0;

			quadcells[cell_index]->streetgraphedgelist = NULL;
			quadcells[cell_index]->nstreetgraphedges = 0;

			quadcells[cell_index]->which_region = 0;
			quadcells[cell_index]->in_region=false;

			quadcells[cell_index]->mapbounds=NULL;
			quadcells[cell_index]->nmapbounds=0;

			//quadcells[cell_index]->in_veg=false;

			/*add it to the corresponding vertices*/
			/*v0*/
			QuadVertex *v;
			v = quad_verts[quadcells[cell_index]->verts[0]];
			v->cells= extend_celllist_ver(v->cells, v->ncells);
			v->cells[v->ncells] = quadcells[cell_index];
			v->ncells ++;
			/*v1*/
			v = quad_verts[quadcells[cell_index]->verts[1]];
			v->cells= extend_celllist_ver(v->cells, v->ncells);
			v->cells[v->ncells] = quadcells[cell_index];
			v->ncells ++;
			/*v2*/
			v = quad_verts[quadcells[cell_index]->verts[2]];
			v->cells= extend_celllist_ver(v->cells, v->ncells);
			v->cells[v->ncells] = quadcells[cell_index];
			v->ncells ++;
			/*v3*/
			v = quad_verts[quadcells[cell_index]->verts[3]];
			v->cells= extend_celllist_ver(v->cells, v->ncells);
			v->cells[v->ncells] = quadcells[cell_index];
			v->ncells ++;
		}
	}
}

void QuadMesh::construct_edges()
{
	///First create and initialize the head knot of the edge link
	QuadEdge *Cur_elink;

	elist = new QuadEdge();
	elist->index = -1;
	elist->next = NULL;
	Cur_elink = elist;

	int edge_id = 0;
	nedges = 0;

	////////////////Define variables for vertices and faces operation
	QuadVertex **vert = quad_verts;

	///////////////////////////////
	int i, j, m, n;
	int Cur_vert, Next_vert;

	for( i = 0; i < nfaces; i++)
	{
		for( j = 0; j < quadcells[i]->nverts; j++)
		{
			//We need to check the neighbor vertex on that surface
			if( j == quadcells[i]->nverts - 1){
				Cur_vert = quadcells[i]->verts[j];
				Next_vert = quadcells[i]->verts[0];
			}
			else{
				Cur_vert = quadcells[i]->verts[j];          //extract the ID of the i'th face and j'th vertex
				Next_vert = quadcells[i]->verts[j+1];       //extract the ID of the i'th face and j+1'th vertex
			}

			//check if there is any edge between them or not
			if(vert[Cur_vert]->Num_edge == 0 || vert[Next_vert]->Num_edge == 0)
			//there must be no edge between them at this moment
			{
				///Create new notes for the edge link
				QuadEdge *new_edge = new QuadEdge;
				new_edge->index = edge_id;              //first edge will be marked 0, and so on...
				edge_id ++;

				/////Initialize the id of the adjacent faces that share the edge
				new_edge->tris[0] = -1;
				new_edge->tris[1] = -1;

				new_edge->tris[0] = i;                        //this is the first surface sharing the edge
				new_edge->verts[0] = Cur_vert;                //Save the ids of current vertices as the terminals of the edge
				new_edge->verts[1] = Next_vert;               //Using the current orientation!!!! 1/11
				new_edge->visited = false;                        //for my subdivision
				new_edge->next = NULL;
				Cur_elink->next = new_edge;                   //Add to the current edge link
				Cur_elink = new_edge;

				/*compute the edge length 07/21/07*/
				//new_edge->length = GetEdgeLength(Cur_vert, Next_vert);


				vert[Cur_vert]->edges = 
					Extend_Elist(vert[Cur_vert]->edges, vert[Cur_vert]->Num_edge);
				vert[Next_vert]->edges = 
					Extend_Elist(vert[Next_vert]->edges, vert[Next_vert]->Num_edge);

				vert[Cur_vert]->edges[vert[Cur_vert]->Num_edge] = new_edge;
				vert[Next_vert]->edges[vert[Next_vert]->Num_edge] = new_edge;

				vert[Cur_vert]->Num_edge++;
				vert[Next_vert]->Num_edge++;
						
				/////Add to the face
				quadcells[i]->edges[j] = new_edge;         //Link the new edge to the associated face

                ////The total number of edges add one
				nedges++;
		    }
			else{
				for( m = 0; m < vert[Cur_vert]->Num_edge; m++)
					for( n = 0; n < vert[Next_vert]->Num_edge; n++)
					{
						if( vert[Cur_vert]->edges[m]->index
							== vert[Next_vert]->edges[n]->index) 
						//There already has an edge between these two vertices
						{

							vert[Cur_vert]->edges[m]->tris[1] = i;

							quadcells[i]->edges[j] = vert[Cur_vert]->edges[m];

							goto LL;                          //if same edge ID has been found, jump out of the loop

						}
					}
				
LL:				if( m > vert[Cur_vert]->Num_edge - 1 )
				//Did not find an existing edge between these two vertices
				{
					///Create new notes for the edge link
					QuadEdge *new_edge = new QuadEdge;
					new_edge->index = edge_id;         //first edge will be marked 0, and so on...
					edge_id ++;
					/////Initialize the id of the adjacent faces that share the edge
					new_edge->tris[0] = -1;
					new_edge->tris[1] = -1;

					new_edge->tris[0] = i;                    //this is the first surface sharing the edge
					new_edge->verts[0] = Cur_vert;            //Save the ids of current vertices as the terminals of the edge
					new_edge->verts[1] = Next_vert;
					new_edge->visited = 0;                    //for my subdivision
					new_edge->next = NULL;
					Cur_elink->next = new_edge;               //Add to the current edge link
					Cur_elink = new_edge;
					
					/*compute the edge length 07/21/07*/
					//new_edge->length = GetEdgeLength(Cur_vert, Next_vert);

					///Add the ID of the edge into corresponding vertices and faces

					vert[Cur_vert]->edges = 
						Extend_Elist(vert[Cur_vert]->edges, vert[Cur_vert]->Num_edge);
					vert[Next_vert]->edges = 
						Extend_Elist(vert[Next_vert]->edges, vert[Next_vert]->Num_edge);

					vert[Cur_vert]->edges[vert[Cur_vert]->Num_edge] = new_edge;
					vert[Next_vert]->edges[vert[Next_vert]->Num_edge] = new_edge;

					vert[Cur_vert]->Num_edge++;
					vert[Next_vert]->Num_edge++;

					/////Add to the face

				    quadcells[i]->edges[j] = new_edge;     //Link the new edge to the associated face

					///The total number of edges add one
					nedges++;
				}
			}

		}
	}

	//Object.nedges = global_edge_id;
	//TotalEdgesNum = nedges;

	/*construct the array style edge list for later convenience*/
	edgelist=(QuadEdge**)malloc(sizeof(QuadEdge*)*nedges);
	QuadEdge *e = elist->next;
	for(i=0; i<nedges; i++)
	{
		edgelist[i]= e;
		e=e->next;
	}
}


QuadEdge  **QuadMesh::Extend_Elist(QuadEdge **edge_link, int Num_edges)
{
    QuadEdge **temp = edge_link;
	QuadEdge **new_edge_link = (QuadEdge **) malloc (sizeof (QuadEdge*)*(Num_edges+1)); //Extend the link
	if( Num_edges > 0)
	{
		for(int i = 0; i < Num_edges; i++)
			new_edge_link[i] = temp[i];
		free (temp);
	}
   
	return new_edge_link;
}

/*extend the cell list for vertex*/
QuadCell  **QuadMesh::extend_celllist_ver(QuadCell **cells, int ncells)
{
    QuadCell **temp = cells;
	QuadCell **new_cells = (QuadCell **) malloc (sizeof (QuadCell*)*(ncells+1)); //Extend the link
	if( ncells > 0)
	{
		for(int i = 0; i < ncells; i++)
			new_cells[i] = temp[i];
		free (temp);
	}
    cells = new_cells;
	return new_cells;
}


/*
orient the edge list in each cell in the following order
   e2
e3    e1
   e0
*/
void QuadMesh::orient_edges_cells()
{
	int i, j, k;
	QuadCell *face;
	QuadEdge *e[4];
	
	for(i=0; i<nfaces; i++)
	{
		face = quadcells[i];
		for(j=0; j<face->nverts; j++)
		{
			if((face->edges[j]->verts[0]==face->verts[0]&&face->edges[j]->verts[1]==face->verts[1])
				||(face->edges[j]->verts[0]==face->verts[1]&&face->edges[j]->verts[1]==face->verts[0]))
				e[0] = face->edges[j];
			else if((face->edges[j]->verts[0]==face->verts[1]&&face->edges[j]->verts[1]==face->verts[2])
				||(face->edges[j]->verts[0]==face->verts[2]&&face->edges[j]->verts[1]==face->verts[1]))
				e[1] = face->edges[j];
			else if((face->edges[j]->verts[0]==face->verts[2]&&face->edges[j]->verts[1]==face->verts[3])
				||(face->edges[j]->verts[0]==face->verts[3]&&face->edges[j]->verts[1]==face->verts[2]))
				e[2] = face->edges[j];
			else
				e[3] = face->edges[j];
		}

		for(j=0; j<face->nverts; j++)
			face->edges[j]=e[j];
	}
}
