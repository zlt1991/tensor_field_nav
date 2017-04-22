#include "tensor_field_nav_core/EvenStreamlinePlace.h"
#include "tensor_field_nav_core/TfCore.h"
EvenStreamlinePlace::EvenStreamlinePlace( int initsize=300)
{
    evenstreamlines = new TrajectoryList(initsize);
    samplepts = (SamplePtList **)malloc(sizeof(SamplePtList*)*evenstreamlines->curMaxNumTrajs);


    if(samplepts == NULL)
    {
        exit(-1);
    }

    for(int i = 0; i < evenstreamlines->curMaxNumTrajs; i++)
    {
        samplepts[i] = new SamplePtList(100);

        if(samplepts[i] == NULL)
        {
            exit(-1);
        }
    }

    trianglelist = NULL;
    majorDensity = 13.5;
    mintenline_length=5.9;
}

EvenStreamlinePlace::~EvenStreamlinePlace()
{
	delete evenstreamlines;

	if(samplepts != NULL)
	{
		for(int i = 0; i < evenstreamlines->curMaxNumTrajs; i++)
		{
			if(samplepts[i] != NULL)
				free(samplepts[i]);
		}
		free(samplepts);
	}

}
void EvenStreamlinePlace::init()
{
	int i, j;
	for(i = 0; i < evenstreamlines->curMaxNumTrajs; i++)
	{
		//evenstreamlines->trajs[i] = (Trajectory*)malloc(sizeof(Trajectory));
		evenstreamlines->trajs[i] = new Trajectory(i,200);

		if(evenstreamlines->trajs[i] == NULL)
		{
			exit(-1);
		}
	}
	for(i = 0; i < evenstreamlines->curMaxNumTrajs; i++)
	{
		for(j = 0; j < samplepts[i]->curMaxNumSamplePts; j++)
		{
			//samplepts[i]->samples[j] = new SamplePt[1];
			samplepts[i]->samples[j] = (SamplePt *)malloc(sizeof(SamplePt));
			if(samplepts[i]->samples[j] == NULL)
			{
				exit(-1);
			}
		}
	}
}
void EvenStreamlinePlace::set_default_parameters()
{
	dsep = majorDensity*quadmesh->xinterval;    //using the radius of the object instead of the edge
	percentage_dsep = 0.5;

	discsize = 2.;
	sample_interval = std::min(0.05*dsep, quadmesh->xinterval/2.);

	every_nsample = 4;
	loopdsep = 0.4*quadmesh->xinterval; /*we now allow the loops to be closed*/
	dist2sing = 0.1*quadmesh->xinterval;

	streamlinelength = mintenline_length*quadmesh->xinterval;
	seeddist = 0.9*dsep;
	minstartdist = 0.9*dsep;

	euler_stepsize = quadmesh->xinterval/5.;
	predict_stepsize = quadmesh->xinterval;

}

void EvenStreamlinePlace::setTfCore(TfCore *tfCore){
    m_tfCore=tfCore;
    quadmesh=m_tfCore->quadmesh;
}
void EvenStreamlinePlace::compute_tensor_at_quad(int face, double x, double y, icMatrix2x2 &ten)
{
	//double xstart, xend, ystart, yend;
	//get_x_y_ranges(face, xstart, xend, ystart, yend);

	QuadCell *qc = quadmesh->quadcells[face];

	/*get the x coeff and y coeff*/
	double a = (x-qc->x_start_coord)/quadmesh->xinterval;
	double b = (y-qc->y_start_coord)/quadmesh->yinterval;

	if(fabs(a)<1e-6)
		a = 0;
	if(fabs(b)<1e-6)
		b = 0;

	QuadVertex *v00 = quadmesh->quad_verts[qc->verts[0]];
	QuadVertex *v01 = quadmesh->quad_verts[qc->verts[3]];
	QuadVertex *v10 = quadmesh->quad_verts[qc->verts[1]];
	QuadVertex *v11 = quadmesh->quad_verts[qc->verts[2]];

	/*the all the components of the interpolated tensor, respectively*/
	ten.entry[0][0] = bilinear_interpolate(a, b, v00->Jacobian.entry[0][0], v01->Jacobian.entry[0][0],
		v10->Jacobian.entry[0][0], v11->Jacobian.entry[0][0]);
	ten.entry[0][1] = bilinear_interpolate(a, b, v00->Jacobian.entry[0][1], v01->Jacobian.entry[0][1],
		v10->Jacobian.entry[0][1], v11->Jacobian.entry[0][1]);
	ten.entry[1][0] = bilinear_interpolate(a, b, v00->Jacobian.entry[1][0], v01->Jacobian.entry[1][0],
		v10->Jacobian.entry[1][0], v11->Jacobian.entry[1][0]);
	ten.entry[1][1] = bilinear_interpolate(a, b, v00->Jacobian.entry[1][1], v01->Jacobian.entry[1][1],
		v10->Jacobian.entry[1][1], v11->Jacobian.entry[1][1]);

}
bool EvenStreamlinePlace::is_in_cell(int id, double x, double y)
{
	if(id<0) return false;

	QuadCell *q = quadmesh->quadcells[id];
	double xleft=q->x_start_coord-1.e-8;
	double xright=q->x_start_coord+quadmesh->xinterval+1.e-8;
	double ybuttom=q->y_start_coord-1.e-8;
	double yupper=q->y_start_coord+quadmesh->yinterval+1.e-8;
	if((x>=xleft && x<=xright)
		&&(y>=ybuttom && y<=yupper))
		return true;
	return false;

}
int  EvenStreamlinePlace::get_cellID_givencoords(double x, double y)
{
	int i=(x-quadmesh->xstart)/quadmesh->xinterval;
	int j=(y-quadmesh->ystart)/quadmesh->yinterval;

	//if(i<0) i=0;
	//if(j<0) j=0;

	if(i==quadmesh->XDIM-1) i=quadmesh->XDIM-2;
	if(j==quadmesh->YDIM-1) j=quadmesh->YDIM-2;

	return (j*(quadmesh->XDIM-1)+i);
}
void EvenStreamlinePlace::compute_phi_in_quad(int face, double x, double y, double &phi)
{

	QuadCell *qc = quadmesh->quadcells[face];

	/*get the x coeff and y coeff*/
	double a = (x-qc->x_start_coord)/quadmesh->xinterval;
	double b = (y-qc->y_start_coord)/quadmesh->yinterval;

	if(fabs(a)<1e-6)
		a = 0;
	if(fabs(b)<1e-6)
		b = 0;

	/*obtain the vertices of this cell in the order of
	  v00 v01
	  v10 v11
	*/
	QuadVertex *v00 = quadmesh->quad_verts[qc->verts[0]];
	QuadVertex *v01 = quadmesh->quad_verts[qc->verts[3]];
	QuadVertex *v10 = quadmesh->quad_verts[qc->verts[1]];
	QuadVertex *v11 = quadmesh->quad_verts[qc->verts[2]];

	/*the all the components of the interpolated tensor, respectively*/
	phi = bilinear_interpolate(a, b, v00->phi, v01->phi,
		v10->phi, v11->phi);

}
void  EvenStreamlinePlace::reset(){
    evenstreamlines->reset();
    (*samplepts)->reset();
}

void EvenStreamlinePlace::get_tenvec_quad(double cur_p[2], double vec[2])
{
	double t[4];
	icMatrix2x2 ten;
	icVector2 ev[2];
	double re;

	compute_tensor_at_quad(g_face, cur_p[0], cur_p[1], ten);

		/*  need to obtain the interpolated phi  */
    m_tfCore->cal_eigen_vector_sym(ten, ev);

	if(g_type == 1) /*use minor eigen vector field*/
		ev[0] = ev[1];

	re = dot(tenline_dir_global, ev[0]);
	if(re<0) 
		ev[0] = -ev[0];
	//normalize(ev[0]);

	vec[0] = ev[0].entry[0];
	vec[1] = ev[0].entry[1];
}
void EvenStreamlinePlace::RK23_2d(double pre_p[2], double next_p[2], double &hstep_loc, double &hnext,
			 double eps, double &eps_did)
{
	double dp0[2], dp1[2];
	double temp[2] = {0.};
	double t_vec[2];

	/*compute dp0*/
	get_tenvec_quad(pre_p, t_vec);
	dp0[0] = hstep_loc*t_vec[0];
	dp0[1] = hstep_loc*t_vec[1];

	/*compute dp1*/
	temp[0]=pre_p[0]+dp0[0];
	temp[1]=pre_p[1]+dp0[1];
	get_tenvec_quad(temp, t_vec);
	dp1[0] = hstep_loc*t_vec[0];
	dp1[1] = hstep_loc*t_vec[1];


	/*compute the next position using dp0, dp1, dp2 and dp3*/
	next_p[0]=pre_p[0]+dp0[0]/2+dp1[0]/2;
	next_p[1]=pre_p[1]+dp0[1]/2+dp1[1]/2;

	/*evaluate the error*/
	get_tenvec_quad(next_p, t_vec);

	icVector2 ep;
	ep.entry[0]=(dp1[0]-hstep_loc*t_vec[0])/3;
	ep.entry[1]=(dp1[1]-hstep_loc*t_vec[1])/3;

	double error = length(ep);

	/*adjust the step size accordingly*/
	hnext = hstep_loc;
	if(error<eps/10.) hnext = 2*hstep_loc;
	if(error>eps*5) hnext = hstep_loc/2;

	eps_did = error;
}

bool EvenStreamlinePlace::get_nextpt_RK23_ten_quad(double first[2], double second[2], int &face_id, int type)
{
	g_face = face_id;
	g_type = type;

	double t_vec[2] = {0.};
	get_tenvec_quad(first, t_vec);
	icVector2 vec;
	vec.entry[0] = t_vec[0];
	vec.entry[1] = t_vec[1];
    if(length(vec) < 1.e-10){
       evenstreamlines->trajs[evenstreamlines->ntrajs]->is_reach_degpt=true;
       return false;
    }

	double eps_did, eps = 1.e-9;
	int i;
	double hstep_loc, hnext;

	hstep_loc = predict_stepsize;
	if(hstep_loc > quadmesh->xinterval/2.)
	{
		hstep_loc = quadmesh->xinterval/2.;
		predict_stepsize = hstep_loc;
	}


	for(i=0; i<5; i++)
	{
		hstep_loc = predict_stepsize;
		RK23_2d(first, second, hstep_loc, hnext, eps, eps_did);
		predict_stepsize = hnext;
		if(eps_did<eps)
			break;
	}
	return true;
}

int EvenStreamlinePlace::get_nextpt_RK23_ten_quad(double first[2], double second[2], int &face_id, int type,int index,int sep_index)
{
    g_face = face_id;
    g_type = type;

    double t_vec[2] = {0.};
    get_tenvec_quad(first, t_vec);
    icVector2 vec;
    vec.entry[0] = t_vec[0];
    vec.entry[1] = t_vec[1];
    if(length(vec) < 1.e-10) return -2;

    for(int i=0; i< m_tfCore->validDegpts.size(); i++){
        if(i==index)continue;
        double dist=sqrt(pow(first[0]-m_tfCore->validDegpts[i].gcx,2)+pow(first[1]-m_tfCore->validDegpts[i].gcy,2));
        if(dist < 2.5*1.e-2) {
         // m_tfCore->insertLinks(index,i);
          evenstreamlines->trajs[evenstreamlines->ntrajs]->linked_degpt_id=i;
          //m_tfCore->validDegpts[index].links_index[sep_index]=i;
          return i;
        }
    }

    double eps_did, eps = 1.e-9;
    int i;
    double hstep_loc, hnext;

    hstep_loc = predict_stepsize;
    if(hstep_loc > quadmesh->xinterval/2.)
    {
        hstep_loc = quadmesh->xinterval/2.;
        predict_stepsize = hstep_loc;
    }


    for(i=0; i<5; i++)
    {
        hstep_loc = predict_stepsize;
        RK23_2d(first, second, hstep_loc, hnext, eps, eps_did);
        predict_stepsize = hnext;
        if(eps_did<eps)
            break;
    }
    return -1;
}
void EvenStreamlinePlace::cal_euclidean_dist_2(int triangle, double p[3], double dsep, double discsize, 
											   DynList_Int *trianglelist)
{
	////we suppose the global point always falls in the triangle
	QuadCell *face = quadmesh->quadcells[triangle];
	QuadVertex *vert;
	QuadEdge *cur_edge;
	icVector3 dis;

	////Find all the triangle
	trianglelist->nelems = 0;
	trianglelist->add_New(triangle);
	int cur_id = 0;
	int i, j;

	while(cur_id < trianglelist->nelems)
	{
		face = quadmesh->quadcells[trianglelist->elems[cur_id]];

		for(i = 0; i < face->nverts; i++)
		{
			vert = quadmesh->quad_verts[face->verts[i]];

			if(vert->visited)
				continue;

			dis.entry[0] = vert->x - p[0];
			dis.entry[1] = vert->y - p[1];

			vert->distance = length(dis); /*compute the Euclidean distance*/

			vert->visited = true; //set the visited flag
			if(vert->distance > discsize*dsep)
				continue;

			////if the distance between the vertex and the input point is smaller than the threshold
			////we need to add all its adjacent triangles into the triangle list
			for(j = 0; j < vert->ncells; j++)
			{
				QuadCell *c = vert->cells[j];

				if(c->index < 0) //reach the boundary!
					continue;
				trianglelist->add_New(c->index);
			}
		}

		cur_id++;
	}

	for(i=0; i<trianglelist->nelems; i++)
	{
		face = quadmesh->quadcells[trianglelist->elems[i]];
		face->visited = false;
		for(j=0; j<face->nverts; j++)
		{
			vert = quadmesh->quad_verts[face->verts[j]];
			vert->visited = false;
		}
	}
}

void EvenStreamlinePlace::reset_dist(DynList_Int *trianglelist)
{
	int i, j;
	QuadCell *face;
	QuadVertex *v;
	for(i = 0; i < trianglelist->nelems; i++)
	{
		face = quadmesh->quadcells[trianglelist->elems[i]];
		face->visited = false;
		for(j = 0; j < 3; j++)
		{
			v = quadmesh->quad_verts[face->verts[j]];
			v->distance = 1e50;
		}
	}
}

bool EvenStreamlinePlace::close_to_cur_samplePt(double p[2], int triangle, SamplePt **samples, int num_samples,
												double separate_dist, double discsize, double sample_interval)
{
	int i;
	double dist/*, alpha[3], a, b*/;
	int stop_locate = floor(separate_dist/sample_interval)+3;
	QuadCell *face;
	QuadVertex *v;
	icVector2 VP;

	int *NearbyTriangles = NULL;
	int num_triangles = 0;


	////Get the triangles that fall into the circle region with p as the center and 3*separate_dist as radius
	trianglelist = new DynList_Int();

	trianglelist->nelems = 0;
	cal_euclidean_dist_2(triangle, p, separate_dist, discsize, trianglelist);
	NearbyTriangles = trianglelist->elems;
	num_triangles = trianglelist->nelems;


	for(i = 0 ; i < num_samples-stop_locate; i++)
	{
		if(!is_repeated_elem(NearbyTriangles, samples[i]->triangle, num_triangles))
			continue;

		face = quadmesh->quadcells[samples[i]->triangle];

		VP.entry[0] = p[0]-samples[i]->gpt[0];
		VP.entry[1] = p[1]-samples[i]->gpt[1];

		dist = length(VP);  /*calculate the Euclidean distance*/

		if(dist < separate_dist)
		{

			reset_dist(trianglelist);
			delete trianglelist;

			return true;
		}
	}

	reset_dist(trianglelist);
	delete trianglelist;
	return false;
}

int EvenStreamlinePlace::trace_majRoad_in_quad(int &face_id, double globalp[2], int type, 
					double dtest, double loopsep, double dist2sing, 
					double sample_interval, double discsize, int &flag)
{

	int i;
	double pre_point[2];

	double origin_dtest=dtest;
	double origin_dist2sing=dist2sing;
	double origin_loopsep=loopsep;
	
	if(!is_in_cell(face_id, globalp[0], globalp[1]))
	{
		face_id = get_cellID_givencoords(globalp[0], globalp[1]);
	}

	if(face_id < 0 || face_id>=quadmesh->nfaces)
		return -1;

	QuadCell *face = quadmesh->quadcells[face_id];
	QuadCell *pre_f = face;

	
	////Temporary curve point array

	CurvePoints *temp_point_list = (CurvePoints*) malloc(sizeof(CurvePoints) * 200);

	if(temp_point_list == NULL)
	{
		exit(-1);
	}

	int NumPoints = 0;
	
	/*the tracing will be performed under the global frame*/
	globalface = face_id;

	pre_point[0] = globalp[0];
	pre_point[1] = globalp[1];

	////////////////////////////////////////////////////
    for(i = 0; i < 200; i++)
	{
		////2. if current point is inside current triangle
		if(is_in_cell(face_id, globalp[0], globalp[1]))
		{
			////store the point into the temp curve points list

			temp_point_list[NumPoints].gpx = globalp[0];
			temp_point_list[NumPoints].gpy = globalp[1];
			temp_point_list[NumPoints].triangleid = face->index;  
			NumPoints++;

			pre_point[0] = globalp[0];
			pre_point[1] = globalp[1];
			
            /*change to use other integration scheme */
			if(get_nextpt_RK23_ten_quad(pre_point, globalp, face_id, type))
			//if(get_nextpt_2ndeuler_ten_quad(pre_point, globalp, face_id, type))
			{
                /*obtain the global direction of current tensor line */
				tenline_dir_global.entry[0] = globalp[0] - pre_point[0];
				tenline_dir_global.entry[1] = globalp[1] - pre_point[1];

                if (evenstreamlines->trajs[evenstreamlines->ntrajs]->nlinesegs+NumPoints-1!=0){
                    if (evenstreamlines->trajs[evenstreamlines->ntrajs]->nlinesegs+NumPoints-1<m_tfCore->interFrameNum*NLIENSPERFRAME+1){
                        if ((evenstreamlines->trajs[evenstreamlines->ntrajs]->nlinesegs+NumPoints-1)%NLIENSPERFRAME==0)
                        {
                            m_tfCore->update_tensor_field();
                        }
                    }

                }
				////We may also need to compare the current point with the sampling point on itself!
				if(close_to_cur_samplePt(globalp, face_id, samplepts[evenstreamlines->ntrajs]->samples,
					samplepts[evenstreamlines->ntrajs]->nsamples, loopsep, discsize, sample_interval)) //scale the separate distance
				{
					if(!evenstreamlines->trajs[evenstreamlines->ntrajs]->store_to_global_line_segs
						(temp_point_list, NumPoints))
					{
						////Not enough memory
						flag = 4;
						free(temp_point_list);
						return face_id;
					}

                    flag = 3;
					free(temp_point_list);
					return face_id;
				}
                if(evenstreamlines->trajs[evenstreamlines->ntrajs]->nlinesegs+NumPoints-1>1000){
                    free(temp_point_list);
                    flag=3;
                    return face_id;
                }
			}

			else{  ////the curve reach a singularity/degenerate point
				flag = 1;

				////Store the record into global line segment array
                
				if(!evenstreamlines->trajs[evenstreamlines->ntrajs]->store_to_global_line_segs
					(temp_point_list, NumPoints))
				{
					////Not enough memory
					flag = 4;
					free(temp_point_list);
					return face_id;
				}

				free(temp_point_list);

				return face_id;
			}
		}



		////3. if the point is out of current cell
		else{

			/*!!!!!!need to judge which cell it will enter!!!!!*/
			int PassVertornot = 0;
			get_next_cell_2(face_id, pre_point, globalp, PassVertornot, type);

			//if(face_id==pre_f->index)
			//{
			//	int test=0;
			//}
			
			if(PassVertornot>0)  /*cross a vertex*/
			{
                /*obtain the global direction of current tensor line*/
				tenline_dir_global.entry[0] = pre_point[0] - globalp[0];
				tenline_dir_global.entry[1] = pre_point[1] - globalp[1];

				/**/
				temp_point_list[NumPoints].gpx = globalp[0];
				temp_point_list[NumPoints].gpy = globalp[1];
                temp_point_list[NumPoints].triangleid = face_id/*face->index*/;  //
				NumPoints++;

				temp_point_list[NumPoints].gpx = pre_point[0];
				temp_point_list[NumPoints].gpy = pre_point[1];
                temp_point_list[NumPoints].triangleid = face_id;  //
				NumPoints++;
			}
			else{
                /*obtain the global direction of current tensor line */
				tenline_dir_global.entry[0] = globalp[0] - pre_point[0];
				tenline_dir_global.entry[1] = globalp[1] - pre_point[1];

				////Add the intersection point to the temporary points' list
				temp_point_list[NumPoints].gpx = globalp[0];
				temp_point_list[NumPoints].gpy = globalp[1];
                temp_point_list[NumPoints].triangleid = face->index;  //
				NumPoints++;
			}


			if(NumPoints > 1){
 				////Store the record into global line segment array
				if(!evenstreamlines->trajs[evenstreamlines->ntrajs]->store_to_global_line_segs
					(temp_point_list, NumPoints))
			   {   ////Not enough memory
				   flag = 4;
				   free(temp_point_list);
				   return face_id;
			   }
			}

			free(temp_point_list);
			return face_id;
		}
	
	}

	if(NumPoints > 0)
		if(!evenstreamlines->trajs[evenstreamlines->ntrajs]->store_to_global_line_segs
						(temp_point_list, NumPoints))
						flag=4;

	free(temp_point_list);
	return face_id;
}


int EvenStreamlinePlace::trace_separatrix_in_quad(int &face_id, double globalp[2], int type,
    double dtest, double loopsep, double dist2sing,
    double sample_interval, double discsize, int &flag,int index,int sep_index){

    int i;
    double pre_point[2];

    double origin_dtest=dtest;
    double origin_dist2sing=dist2sing;
    double origin_loopsep=loopsep;

    if(!is_in_cell(face_id, globalp[0], globalp[1]))
    {
        face_id = get_cellID_givencoords(globalp[0], globalp[1]);
    }

    if(face_id < 0 || face_id>=quadmesh->nfaces)
        return -1;

    QuadCell *face = quadmesh->quadcells[face_id];
    QuadCell *pre_f = face;


    ////Temporary curve point array

    CurvePoints *temp_point_list = (CurvePoints*) malloc(sizeof(CurvePoints) * 200);

    if(temp_point_list == NULL)
    {
        exit(-1);
    }

    int NumPoints = 0;

    /*the tracing will be performed under the global frame*/
    globalface = face_id;

    pre_point[0] = globalp[0];
    pre_point[1] = globalp[1];

    ////////////////////////////////////////////////////
    for(i = 0; i < 200; i++)
    {
        ////2. if current point is inside current triangle
        if(is_in_cell(face_id, globalp[0], globalp[1]))
        {
            ////store the point into the temp curve points list

            temp_point_list[NumPoints].gpx = globalp[0];
            temp_point_list[NumPoints].gpy = globalp[1];
            temp_point_list[NumPoints].triangleid = face->index;
            NumPoints++;

            pre_point[0] = globalp[0];
            pre_point[1] = globalp[1];

            int tmp_index=get_nextpt_RK23_ten_quad(pre_point, globalp, face_id, type,index,sep_index);
            if(tmp_index==-1)
            //if(get_nextpt_2ndeuler_ten_quad(pre_point, globalp, face_id, type))
            {
                /*obtain the global direction of current tensor line */
                tenline_dir_global.entry[0] = globalp[0] - pre_point[0];
                tenline_dir_global.entry[1] = globalp[1] - pre_point[1];

                /*      we need to combine the density map to change
                        the separation distance automatically
                */
                ////We may also need to compare the current point with the sampling point on itself!
                if(close_to_cur_samplePt(globalp, face_id, samplepts[evenstreamlines->ntrajs]->samples,
                    samplepts[evenstreamlines->ntrajs]->nsamples, loopsep, discsize, sample_interval)) //scale the separate distance
                {
                    if(!evenstreamlines->trajs[evenstreamlines->ntrajs]->store_to_global_line_segs
                        (temp_point_list, NumPoints))
                    {
                        ////Not enough memory
                        flag = 4;
                        free(temp_point_list);
                        return face_id;
                    }

                    flag = 3; //form a loop!

                    free(temp_point_list);
                    return face_id;
                }
                if(globalp[0]<m_tfCore->leftBottom[0] || globalp[0]>m_tfCore->rightTop[0] || globalp[1]<m_tfCore->leftBottom[1] || globalp[1]>m_tfCore->rightTop[1])
                {
                    flag=5;
                    evenstreamlines->trajs[evenstreamlines->ntrajs]->is_reach_unknown=true;
                    return face_id;
                }

            }

            else{  ////the curve reach a singularity/degenerate point
                if(tmp_index!=-2){
                    temp_point_list[NumPoints].gpx = m_tfCore->validDegpts[tmp_index].gcx;
                    temp_point_list[NumPoints].gpy = m_tfCore->validDegpts[tmp_index].gcy;
                    temp_point_list[NumPoints].triangleid = face->index;
                    NumPoints++;
                }
                flag = 1;

                ////Store the record into global line segment array

                if(!evenstreamlines->trajs[evenstreamlines->ntrajs]->store_to_global_line_segs
                    (temp_point_list, NumPoints))
                {
                    ////Not enough memory
                    ROS_ERROR("Not enough memory");
                    flag = 4;
                    free(temp_point_list);
                    return face_id;
                }

                free(temp_point_list);

                return face_id;
            }
        }



        ////3. if the point is out of current cell
        else{

            /*!!!!!!need to judge which cell it will enter!!!!!*/
            int PassVertornot = 0;
            get_next_cell_2(face_id, pre_point, globalp, PassVertornot, type);

            //if(face_id==pre_f->index)
            //{
            //	int test=0;
            //}

            if(PassVertornot>0)  /*cross a vertex*/
            {
                /*obtain the global direction of current tensor line */
                tenline_dir_global.entry[0] = pre_point[0] - globalp[0];
                tenline_dir_global.entry[1] = pre_point[1] - globalp[1];

                /**/
                temp_point_list[NumPoints].gpx = globalp[0];
                temp_point_list[NumPoints].gpy = globalp[1];
                temp_point_list[NumPoints].triangleid = face_id/*face->index*/;  //
                NumPoints++;

                temp_point_list[NumPoints].gpx = pre_point[0];
                temp_point_list[NumPoints].gpy = pre_point[1];
                temp_point_list[NumPoints].triangleid = face_id;  //
                NumPoints++;
            }
            else{
                /*obtain the global direction of current tensor line*/
                tenline_dir_global.entry[0] = globalp[0] - pre_point[0];
                tenline_dir_global.entry[1] = globalp[1] - pre_point[1];

                ////Add the intersection point to the temporary points' list
                temp_point_list[NumPoints].gpx = globalp[0];
                temp_point_list[NumPoints].gpy = globalp[1];
                temp_point_list[NumPoints].triangleid = face->index;  //
                NumPoints++;
            }


            if(NumPoints > 1){
                ////Store the record into global line segment array
                if(!evenstreamlines->trajs[evenstreamlines->ntrajs]->store_to_global_line_segs
                    (temp_point_list, NumPoints))
               {   ////Not enough memory
                   flag = 4;
                   free(temp_point_list);
                   return face_id;
               }
            }

            free(temp_point_list);
            return face_id;
        }

    }

    if(NumPoints > 0)
        if(!evenstreamlines->trajs[evenstreamlines->ntrajs]->store_to_global_line_segs
                        (temp_point_list, NumPoints))
                        flag=4;

    free(temp_point_list);
    return face_id;

}

void  EvenStreamlinePlace::get_cell_through_ver(int vertid, int &cell, int type)
{
	QuadVertex *v = quadmesh->quad_verts[vertid];
	icVector2 vec; 
	if(type == 0)
		vec = v->major;
	else
		vec = v->minor;

	double re=dot(vec, tenline_dir_global);
	if(re<0) vec=-vec;
	normalize(vec);

	//double x, y;

	double x = v->x+0.02*quadmesh->xinterval*vec.entry[0];
	double y = v->y+0.02*quadmesh->xinterval*vec.entry[1];
	cell = get_cellID_givencoords(x, y);
}
bool EvenStreamlinePlace::cross_vertex_ten_quad(int &face_id, double cur_p[2], double pre_p[2], int &passornot, int type)
{
	int i;
	double vert[2];
	double max_alpha ;
	int newtriangleid = 0;
	int crossVert;
	QuadCell *face = quadmesh->quadcells[face_id];
	QuadVertex *v;

	double A, B, C, pending;
	A = pre_p[1] - cur_p[1];
	B = cur_p[0] - pre_p[0];
	C = (pre_p[0]*cur_p[1] - cur_p[0]*pre_p[1]);

	for(i = 0; i < face->nverts; i++)
	{
		v = quadmesh->quad_verts[face->verts[i]];
		vert[0] = v->x;
		vert[1] = v->y;
		pending = A*vert[0] + B*vert[1] + C;
		////We also need to make sure that the vertex is between 'pre' and 'cur' points
		//if(fabs(pending) == 0.0) ////passing the vertex
		if(fabs(pending) <=1.e-8) ////passing the vertex
		{
			////Test whether the vertex is between 'pre' and 'cur' points
			double t;
			if(pre_p[0] != cur_p[0])
			{
				t = (vert[0] - pre_p[0])/(cur_p[0] - pre_p[0]);

			}
			else{
				t = (vert[1] - pre_p[1])/(cur_p[1] - pre_p[1]);
			}

			if(t < 0 || t > 1)
			{
				passornot = 0;
				continue;
			}

			crossVert = face->verts[i];

			////////////////////////////////////
			newtriangleid = face_id;

			get_cell_through_ver(crossVert, newtriangleid, type);
			if(newtriangleid <0 || newtriangleid>=quadmesh->nfaces) 
				return false;
			face_id = newtriangleid;
			passornot = i+1;
			cur_p[0] = quadmesh->quad_verts[crossVert]->x;
			cur_p[1] = quadmesh->quad_verts[crossVert]->y;
			//pre_p[0] = quadmesh->quad_verts[crossVert]->x;
			//pre_p[1] = quadmesh->quad_verts[crossVert]->y;

            /*make it off the vertex a little bit */
			icVector2 tvec;
			QuadVertex *tv = quadmesh->quad_verts[crossVert];
			if(type==0)
				tvec=tv->major;
			else
				tvec=tv->minor;
			normalize(tvec);
			double re=dot(tvec, tenline_dir_global);
			if(re<0) tvec=-tvec;

			/*  try to avoid the vertex  */
			pre_p[0] = tv->x+0.01*quadmesh->xinterval*tvec.entry[0];
			pre_p[1] = tv->y+0.01*quadmesh->xinterval*tvec.entry[1];

			int test_cell=get_cellID_givencoords(pre_p[0], pre_p[1]);

			//if(test_cell != newtriangleid)
			//{
			//	int test=0;
			//}

			return true;
		}
	}

	passornot = 0;
	return false;
}
void EvenStreamlinePlace::get_next_cell_2(int &face_id, double pre[2], double cur[2], 
					 int &PassVertornot, int type)
{

	/*for horizontal cases*/
	if(fabs(cur[1]-pre[1])<1e-8)
	{
		if(cur[0]>pre[0] && cur[0]<=quadmesh->xend-1.e-8) /*move to the right cell*/
		{
			cur[0]=quadmesh->quadcells[face_id]->x_start_coord+quadmesh->xinterval+1.1e-8;
			face_id++;
			return;
		}

		else if(cur[0]>pre[0] && cur[0]>quadmesh->xend-1.e-8) /*out of mesh*/
		{
			face_id=-1;
			return;
		}

		else if(cur[0]<pre[0] && cur[0]>=quadmesh->xstart+1.e-8)/*move to the left cell*/
		{
			cur[0]=quadmesh->quadcells[face_id]->x_start_coord-1.1e-8;
			face_id--;
			return;
		}

		else if(cur[0]<pre[0] && cur[0]<quadmesh->xstart+1.e-8)/*out of mesh*/
		{
			face_id=-1;
			return;
		}
	}

	/*for vertical cases*/
	if(fabs(cur[0]-pre[0])<1e-8)
	{
		if(cur[1]>pre[1] && cur[1]<=quadmesh->yend-1.e-8) /*move to the upper cell*/
		{
			cur[1]=quadmesh->quadcells[face_id]->y_start_coord+quadmesh->yinterval+1.1e-8;
			face_id+=(quadmesh->XDIM-1);
			return;
		}

		else if(cur[1]>pre[1] && cur[1]>quadmesh->yend-1.e-8)
		{
			face_id = -1;
			return;
		}

		else if(cur[1]<pre[1] && cur[1]>=quadmesh->ystart+1.e-8) /*move to the lower cell*/
		{
			cur[1]=quadmesh->quadcells[face_id]->y_start_coord-1.1e-8;
			face_id-=(quadmesh->XDIM-1);
			return;
		}

		else if(cur[1]<pre[1] && cur[1]<quadmesh->ystart+1.e-8)
		{
			face_id = -1;
			return;
		}
	}

	int pre_f = face_id;
	if(cross_vertex_ten_quad(face_id, cur, pre, PassVertornot, type))
		return;

	face_id = pre_f;

	double t[2];
	double C[2], D[2];
	QuadCell *f = quadmesh->quadcells[face_id];

	if(cur[0]<f->x_start_coord) /*left */
	{
		/*calculate the intersection between edge e3 and L*/
		C[0] = quadmesh->quad_verts[f->verts[0]]->x;
		C[1] = quadmesh->quad_verts[f->verts[0]]->y;
		D[0] = quadmesh->quad_verts[f->verts[3]]->x;
		D[1] = quadmesh->quad_verts[f->verts[3]]->y;
		if(GetIntersection2(pre, cur, C, D, t)==1)
		{
			face_id = face_id-1;
			/*calculate the intersection*/
			cur[0]=C[0]+t[1]*(D[0]-C[0]);
			//cur[0]=f->x_start_coord-1.e-8;
			cur[1]=C[1]+t[1]*(D[1]-C[1]);

			return;
		}
	}
	else if(cur[0]>f->x_start_coord+quadmesh->xinterval)  /*right*/
	{
		/*calculate the intersection between edge e2 and L*/
		C[0] = quadmesh->quad_verts[f->verts[1]]->x;
		C[1] = quadmesh->quad_verts[f->verts[1]]->y;
		D[0] = quadmesh->quad_verts[f->verts[2]]->x;
		D[1] = quadmesh->quad_verts[f->verts[2]]->y;

		if(GetIntersection2(pre, cur, C, D, t)==1)
		{
			face_id = face_id+1;
			/*calculate the intersection*/
			cur[0]=C[0]+t[1]*(D[0]-C[0]);
			//cur[0]=f->x_start_coord+quadmesh->xinterval+1.e-8;
			cur[1]=C[1]+t[1]*(D[1]-C[1]);

			return;
		}
	}


	if(cur[1]<f->y_start_coord) /*bottom */
	{
		/*calculate the intersection between edge e0 and L*/
		C[0] = quadmesh->quad_verts[f->verts[0]]->x;
		C[1] = quadmesh->quad_verts[f->verts[0]]->y;
		D[0] = quadmesh->quad_verts[f->verts[1]]->x;
		D[1] = quadmesh->quad_verts[f->verts[1]]->y;

		if(GetIntersection2(pre, cur, C, D, t)==1)
		{
			face_id = face_id-quadmesh->XDIM+1;
			/*calculate the intersection*/
			cur[0]=C[0]+t[1]*(D[0]-C[0]);
			cur[1]=C[1]+t[1]*(D[1]-C[1]);
			//cur[1]=f->y_start_coord-1.e-8;

			return;
		}
	}
	else if(cur[1]>f->y_start_coord+quadmesh->yinterval)  /*upper*/
	{
		/*calculate the intersection between edge e2 and L*/
		C[0] = quadmesh->quad_verts[f->verts[2]]->x;
		C[1] = quadmesh->quad_verts[f->verts[2]]->y;
		D[0] = quadmesh->quad_verts[f->verts[3]]->x;
		D[1] = quadmesh->quad_verts[f->verts[3]]->y;

		if(GetIntersection2(pre, cur, C, D, t)==1)
		{
			face_id = face_id+quadmesh->XDIM-1;
			/*calculate the intersection*/
			cur[0]=C[0]+t[1]*(D[0]-C[0]);
			cur[1]=C[1]+t[1]*(D[1]-C[1]);
			//cur[1]=f->y_start_coord+quadmesh->yinterval+1.e-8;

			return;
		}
	}

	face_id = -1;
}

bool EvenStreamlinePlace::cal_a_sample_of_streamline(int traj, int &cur_lineindex, int &movetonext,
													 double curpt[2], double interval, double &cur_length)
{
	int i;
	icVector2 len_vec;
	double alpha;

	int num_lines = evenstreamlines->trajs[traj]->nlinesegs;

	curpt[0] = curpt[1] = -1;

	if(cur_length >= interval)
	{
		alpha = (cur_length-interval)/evenstreamlines->trajs[traj]->linesegs[cur_lineindex].length;
		curpt[0] = alpha*evenstreamlines->trajs[traj]->linesegs[cur_lineindex].gstart[0] 
		+ (1-alpha)*evenstreamlines->trajs[traj]->linesegs[cur_lineindex].gend[0];
		curpt[1] = alpha*evenstreamlines->trajs[traj]->linesegs[cur_lineindex].gstart[1] 
		+ (1-alpha)*evenstreamlines->trajs[traj]->linesegs[cur_lineindex].gend[1];

		//if(alpha>1 || alpha<0)
		//{
		//	int test =0;
		//}

		cur_length -= interval;
		return true;
	}

	else{
		cur_lineindex++;
		if(cur_lineindex >= num_lines)
		{
			return false;
		}
		cur_length += evenstreamlines->trajs[traj]->linesegs[cur_lineindex].length;
		return false;
	}
}
bool EvenStreamlinePlace::is_in_reg_cell(int id, double x, double y)
{
	if(id<0) return false;
	int i=(x-quadmesh->xstart)/quadmesh->xinterval;
	int j=(y-quadmesh->ystart)/quadmesh->yinterval;

	if(id==(j*(quadmesh->XDIM-1)+i))
		return true;
	return false;
}
SamplePt **EvenStreamlinePlace::cal_samplepts_when_tracing(int traj, double interval, int &cur_line, int &movetonext, double &cur_length, 
														   SamplePt **samples, int &num_samples)
{
	double cur_p[2] = {0.};
	QuadCell *face;

	while(cur_line < evenstreamlines->trajs[traj]->nlinesegs)
	{
		if(cal_a_sample_of_streamline(traj, cur_line, movetonext,
			cur_p, interval, cur_length))
		{
			if(evenstreamlines->trajs[traj]->linesegs[cur_line].Triangle_ID<0
				||evenstreamlines->trajs[traj]->linesegs[cur_line].Triangle_ID>=quadmesh->nfaces)
				return samplepts[traj]->samples;

			face = quadmesh->quadcells[evenstreamlines->trajs[traj]->linesegs[cur_line].Triangle_ID];
			/*since we use global frame here, we need not convert to global frame here*/
			samplepts[traj]->samples[num_samples]->gpt[0] = cur_p[0];
			samplepts[traj]->samples[num_samples]->gpt[1] = cur_p[1];

			samplepts[traj]->samples[num_samples]->triangle = face->index;

			if(!is_in_reg_cell(face->index, cur_p[0], cur_p[1]))
			{
				int test = 0;

				samplepts[traj]->samples[num_samples]->triangle = 
					get_cellID_givencoords(cur_p[0], cur_p[1]);
			}

			if(samplepts[traj]->samples[num_samples]->triangle < 0 
				|| samplepts[traj]->samples[num_samples]->triangle >= quadmesh->nfaces)
			{
				continue;
			}

			num_samples++;

			if(num_samples >= samplepts[traj]->curMaxNumSamplePts)
			{

				int oldnum = samplepts[traj]->curMaxNumSamplePts;

				if(!samplepts[traj]->extend(200))
				{
					return NULL;
				}

				/*allocate memory for the elements*/
				for(int i = oldnum ; i < samplepts[traj]->curMaxNumSamplePts; i++)
				{
					samplepts[traj]->samples[i] = (SamplePt *)malloc(sizeof(SamplePt));
				}

			}
		}
	}

	cur_line--;

	if(cur_line < 0)
		cur_line = 0;

	return samplepts[traj]->samples;
}
void EvenStreamlinePlace::reverse_streamline(int streamlineid)
{
	////
	int i;
	int num_lines = evenstreamlines->trajs[streamlineid]->nlinesegs;
	Trajectory *traj = evenstreamlines->trajs[streamlineid];

	LineSeg *temp = (LineSeg *)malloc(sizeof(LineSeg)*(num_lines+1));

	if(temp == NULL)
	{
		return;
	}


	int newnum_lines = 0;

	////store the line segment in reversed order

	for(i = num_lines-1; i >= 0; i--)
	{
		if(traj->linesegs[i].Triangle_ID < 0
			|| traj->linesegs[i].Triangle_ID >= quadmesh->nfaces  
			|| traj->linesegs[i].length < 0)
		{
			continue;
		}

		temp[newnum_lines].gstart[0] = traj->linesegs[i].gend[0];
		temp[newnum_lines].gstart[1] = traj->linesegs[i].gend[1];

		temp[newnum_lines].gend[0] = traj->linesegs[i].gstart[0];
		temp[newnum_lines].gend[1] = traj->linesegs[i].gstart[1];

		temp[newnum_lines].start[0] = traj->linesegs[i].end[0];
		temp[newnum_lines].start[1] = traj->linesegs[i].end[1];

		temp[newnum_lines].end[0] = traj->linesegs[i].start[0];
		temp[newnum_lines].end[1] = traj->linesegs[i].start[1];

		temp[newnum_lines].length = traj->linesegs[i].length;
		temp[newnum_lines].Triangle_ID = traj->linesegs[i].Triangle_ID;


		newnum_lines++;
	}

	////Copy it back to the origin array
	for(i = 0; i < newnum_lines; i++)
	{
		traj->linesegs[i].gstart[0] = temp[i].gstart[0];
		traj->linesegs[i].gstart[1] = temp[i].gstart[1];

		traj->linesegs[i].gend[0] = temp[i].gend[0];
		traj->linesegs[i].gend[1] = temp[i].gend[1];

		traj->linesegs[i].start[0] = temp[i].start[0];
		traj->linesegs[i].start[1] = temp[i].start[1];

		traj->linesegs[i].end[0] = temp[i].end[0];
		traj->linesegs[i].end[1] = temp[i].end[1];

		traj->linesegs[i].length = temp[i].length;
		traj->linesegs[i].Triangle_ID = temp[i].Triangle_ID;
	}

	traj->nlinesegs = newnum_lines;

	free(temp);
}
bool EvenStreamlinePlace::grow_a_majRoad(double seed_p[2], int triangle, double dtest, 
					double discsize, double Sample_interval, 
					double loopdsep, double dist2sing, 
					double streamlinelength, 
                    int type, icVector2 &direction)
{
	int i;
	int flag = -1;

	int pre_face, cur_face;
	double globalp[3] = {0.};
	int cur_line = 0;
	double cur_length = 0;
	int movetonext = 0;

	/*  variables for pixel level judgement of "inland" or not  */
	double xstart=quadmesh->xstart;
	double ystart=quadmesh->ystart;
	double xrang=quadmesh->xend-quadmesh->xstart;
	double yrang=quadmesh->yend-quadmesh->ystart;
    double dx=xrang/(NPIX-1);
    double dy=yrang/(NPIX-1);
	double dist_did;


	pre_face = cur_face = triangle;
	globalp[0] = seed_p[0];
	globalp[1] = seed_p[1];

	icMatrix2x2 ten;
	//double t[4]={0.};

	if(!is_in_cell(triangle, seed_p[0], seed_p[1]))
	{
		triangle = get_cellID_givencoords(seed_p[0], seed_p[1]);
	}

	if(triangle < 0 || triangle >= quadmesh->nfaces)
		return false;

	compute_tensor_at_quad(triangle, seed_p[0], seed_p[1], ten);

	double evalues[2] = {0.};
	icVector2 ev[2], startdir;
    m_tfCore->cal_eigen_vector_sym(ten, ev);

	if(type == 1) /*use minor eigen vector field*/
		ev[0] = ev[1];
    if(dot(direction,ev[0])>0){
       tenline_dir_global = startdir = ev[0];  /*obtain the major eigen vector*/
    }else{
       tenline_dir_global = startdir = -ev[0];  /*obtain the major eigen vector*/
    }

	evenstreamlines->trajs[evenstreamlines->ntrajs]->nlinesegs = 0;
	evenstreamlines->trajs[evenstreamlines->ntrajs]->closed=false;


	samplepts[evenstreamlines->ntrajs]->samples[0]->gpt[0] = seed_p[0];
	samplepts[evenstreamlines->ntrajs]->samples[0]->gpt[1] = seed_p[1];
	samplepts[evenstreamlines->ntrajs]->samples[0]->triangle = triangle;
	samplepts[evenstreamlines->ntrajs]->nsamples=1;


	hstep = quadmesh->xinterval/2.;
	predict_stepsize = quadmesh->xinterval/2.;
	euler_stepsize = quadmesh->xinterval/10.;


	cur_line = 0;
	cur_length = 0;

	//////////////////////////////////////////////////////////////////////////

	////Backward tracing
	int NUMTRACETRIS = (int)sqrt((double)quadmesh->nfaces);

	for(i = 0; i < 3*NUMTRACETRIS; i++)
	{

		////The cell does not exist. Something is wrong!
		if(cur_face < 0 || cur_face >= quadmesh->nfaces)
			break;


		/*it reaches any boundary, we should stop as well*/
		if(globalp[0]>=quadmesh->xend-1.e-8||globalp[0]<=quadmesh->xstart+1.e-8
			||globalp[1]>=quadmesh->yend-1.e-8||globalp[1]<=quadmesh->ystart+1.e-8)
		{
			if(globalp[0]>=quadmesh->xend-1.e-8) globalp[0]=quadmesh->xend;
			else if(globalp[0]<=quadmesh->xstart+1.e-8) globalp[0]=quadmesh->xstart;
			if(globalp[1]>=quadmesh->yend-1.e-8) globalp[1]=quadmesh->yend;
			else if(globalp[1]<=quadmesh->ystart+1.e-8) globalp[1]=quadmesh->ystart;
			break;
		}

		pre_face = cur_face;
		cur_face = trace_majRoad_in_quad(cur_face, globalp, type, dtest, loopdsep, dist2sing,
			Sample_interval, discsize, flag);
        if(flag==1 || flag==3 || flag==4)
            break;
		////if this is the first line segment, there may be a case that the start point is on the edge !!!
		if(pre_face == triangle && evenstreamlines->trajs[evenstreamlines->ntrajs]->nlinesegs>0)
		{
			cur_length = evenstreamlines->trajs[evenstreamlines->ntrajs]->linesegs[0].length;
		}
		cal_samplepts_when_tracing(evenstreamlines->ntrajs, Sample_interval, cur_line, movetonext, cur_length,
			samplepts[evenstreamlines->ntrajs]->samples, samplepts[evenstreamlines->ntrajs]->nsamples);
        if(evenstreamlines->trajs[evenstreamlines->ntrajs]->nlinesegs>5){
            if (evenstreamlines->trajs[evenstreamlines->ntrajs]->get_length()>m_tfCore->interFrameNum*LIEN_LENGTH_PER_FRAME){
                 break;
             }
        }
	}
    evenstreamlines->trajs[evenstreamlines->ntrajs]->traj_len=
        evenstreamlines->trajs[evenstreamlines->ntrajs]->get_length();

    evenstreamlines->ntrajs ++;

	return true;
}


bool EvenStreamlinePlace::grow_a_separatrix(double degpt_loc[2],double seed_p[2], int triangle, double dtest,
                                         double discsize, double Sample_interval,
                                         double loopdsep, double dist2sing,
                                         double streamlinelength,
                                         int type, icVector2 direction,int index,int sep_index)
{
    int i;
    int flag = -1;

    int pre_face, cur_face;
    double globalp[3] = {0.};
    int cur_line = 0;
    double cur_length = 0;
    int movetonext = 0;

    /*  variables for pixel level judgement of "inland" or not  */
    double xstart=quadmesh->xstart;
    double ystart=quadmesh->ystart;
    double xrang=quadmesh->xend-quadmesh->xstart;
    double yrang=quadmesh->yend-quadmesh->ystart;
    double dx=xrang/(NPIX-1);
    double dy=yrang/(NPIX-1);
    double dist_did;


    pre_face = cur_face = triangle;
    globalp[0] = seed_p[0];
    globalp[1] = seed_p[1];

    icMatrix2x2 ten;
    //double t[4]={0.};

    if(!is_in_cell(triangle, seed_p[0], seed_p[1]))
    {
        triangle = get_cellID_givencoords(seed_p[0], seed_p[1]);
    }

    if(triangle < 0 || triangle >= quadmesh->nfaces)
        return false;

    compute_tensor_at_quad(triangle, seed_p[0], seed_p[1], ten);

    double evalues[2] = {0.};
    icVector2 ev[2], startdir;
    m_tfCore->cal_eigen_vector_sym(ten, ev);

    if(type == 1) /*use minor eigen vector field*/
        ev[0] = ev[1];

    tenline_dir_global = startdir = direction;
    evenstreamlines->trajs[evenstreamlines->ntrajs]->degpt_id=index;
    evenstreamlines->trajs[evenstreamlines->ntrajs]->degpt_sep_index=sep_index;
    evenstreamlines->trajs[evenstreamlines->ntrajs]->nlinesegs = 1;
    evenstreamlines->trajs[evenstreamlines->ntrajs]->linesegs[0].gstart[0]=degpt_loc[0];
    evenstreamlines->trajs[evenstreamlines->ntrajs]->linesegs[0].gstart[1]=degpt_loc[1];
    evenstreamlines->trajs[evenstreamlines->ntrajs]->linesegs[0].gend[0]=seed_p[0];
    evenstreamlines->trajs[evenstreamlines->ntrajs]->linesegs[0].gend[1]=seed_p[1];

    evenstreamlines->trajs[evenstreamlines->ntrajs]->closed=false;


    samplepts[evenstreamlines->ntrajs]->samples[0]->gpt[0] = seed_p[0];
    samplepts[evenstreamlines->ntrajs]->samples[0]->gpt[1] = seed_p[1];
    samplepts[evenstreamlines->ntrajs]->samples[0]->triangle = triangle;
    samplepts[evenstreamlines->ntrajs]->nsamples=1;


    hstep = quadmesh->xinterval/2.;
    predict_stepsize = quadmesh->xinterval/2.;
    euler_stepsize = quadmesh->xinterval/10.;


    cur_line = 0;
    cur_length = 0;



    //////////////////////////////////////////////////////////////////////////

    ////Backward tracing
    int NUMTRACETRIS = (int)sqrt((double)quadmesh->nfaces);

    for(i = 0; i < 3*NUMTRACETRIS; i++)
    {

        ////The cell does not exist. Something is wrong!

        if(cur_face < 0 || cur_face >= quadmesh->nfaces)
            break;


        /*it reaches any boundary, we should stop as well*/
        if(globalp[0]>=quadmesh->xend-1.e-8||globalp[0]<=quadmesh->xstart+1.e-8
            ||globalp[1]>=quadmesh->yend-1.e-8||globalp[1]<=quadmesh->ystart+1.e-8)
        {
            if(globalp[0]>=quadmesh->xend-1.e-8) globalp[0]=quadmesh->xend;
            else if(globalp[0]<=quadmesh->xstart+1.e-8) globalp[0]=quadmesh->xstart;
            if(globalp[1]>=quadmesh->yend-1.e-8) globalp[1]=quadmesh->yend;
            else if(globalp[1]<=quadmesh->ystart+1.e-8) globalp[1]=quadmesh->ystart;
            break;
        }

        pre_face = cur_face;
        cur_face = trace_separatrix_in_quad(cur_face, globalp, type, dtest, loopdsep, dist2sing,
            Sample_interval, discsize, flag,index,sep_index);

        cal_samplepts_when_tracing(evenstreamlines->ntrajs, Sample_interval, cur_line, movetonext, cur_length,
            samplepts[evenstreamlines->ntrajs]->samples, samplepts[evenstreamlines->ntrajs]->nsamples);

        if(flag==1 || flag==3 || flag==4 || flag==5)
            break;
        //if(evenstreamlines->trajs[evenstreamlines->ntrajs]->nlinesegs>2000) break;

    }

    evenstreamlines->ntrajs ++;
    return true;
}


SampleListInTriangle * EvenStreamlinePlace::extend_cell_samplelist(SampleListInTriangle *samplepts, int nsamples)
{
	if(samplepts == NULL || nsamples == 0)
	{
		samplepts = (SampleListInTriangle*)malloc(sizeof(SampleListInTriangle));
		return samplepts;
	}

	SampleListInTriangle *temp = samplepts;
	samplepts = (SampleListInTriangle*)malloc(sizeof(SampleListInTriangle)*(nsamples+1));
	for(int i=0; i<nsamples; i++)
		samplepts[i] = temp[i];
	return samplepts;
}

void EvenStreamlinePlace::add_sample_to_cell(int triangle, int which_traj, 
											 int which_sample, bool fieldtype)
{
	if(triangle < 0 || triangle >= quadmesh->nfaces)
		return;

	QuadCell *face = quadmesh->quadcells[triangle];

	if(!fieldtype) /*major*/
	{
		face->maj_samplepts = extend_cell_samplelist(face->maj_samplepts, 
			face->maj_nsamplepts);

		face->maj_samplepts[face->maj_nsamplepts].which_traj = which_traj;
		face->maj_samplepts[face->maj_nsamplepts].which_sample = which_sample;

		face->maj_nsamplepts++;
	}
	else /*minor*/
	{
		face->min_samplepts = extend_cell_samplelist(face->min_samplepts, 
			face->min_nsamplepts);

		face->min_samplepts[face->min_nsamplepts].which_traj = which_traj;
		face->min_samplepts[face->min_nsamplepts].which_sample = which_sample;

		face->min_nsamplepts++;
	}
}



void EvenStreamlinePlace::reset_placement_quad()
{
	int i;
	QuadCell *face;

	for(i = 0; i < quadmesh->nfaces; i++)
	{
		face = quadmesh->quadcells[i];
		face->visited = false;

		//face->reset_sampleList();
	}
    init_samplelist_in_cell();

	for(i = 0; i < quadmesh->nverts; i++)
	{
		quadmesh->quad_verts[i]->distance = 1e49;
		quadmesh->quad_verts[i]->visited = false;
	}
}
void EvenStreamlinePlace::init_samplelist_in_cell()
{
	int i;
	QuadCell *face;
    for(i=0; i<quadmesh->nfaces; i++)
    {
        face = quadmesh->quadcells[i];
        if(face->maj_samplepts != NULL)
            free(face->maj_samplepts);
        face->maj_samplepts = NULL;
        face->maj_nsamplepts = 0;
        face->MAJMaxSampNum=0;
    }
}
void EvenStreamlinePlace::init_major_line_info()
{
	int i;
	QuadCell *face;
	for(i=0; i<quadmesh->nfaces; i++)
	{
		face = quadmesh->quadcells[i];
        if(face->majorlines != NULL)
        {
            delete face->majorlines;
            face->majorlines = NULL;
        }
        face->hasmajor = false;
	}
}
