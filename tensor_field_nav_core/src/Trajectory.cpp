#include "tensor_field_nav_core/Trajectory.h"
Trajectory::Trajectory(int index, int curMaxNum = 200){
	this->index = index;

	if(curMaxNum==0)
	{
		linesegs=NULL;
		curMaxNumLinesegs=0;
		return;
	}

	linesegs = (LineSeg *)malloc(sizeof(LineSeg)*curMaxNum);

	if(linesegs == NULL)
	{
		exit(-1);
	}

	int i;
	for(i=0;i<curMaxNum;i++)
	{
		linesegs[i].gend[0]=linesegs[i].end[0]=linesegs[i].gstart[0]=linesegs[i].start[0]=
			linesegs[i].gend[1]=linesegs[i].end[1]=linesegs[i].gstart[1]=linesegs[i].start[1]=0.;
		linesegs[i].length=0;
		linesegs[i].Triangle_ID=0;
	}
    linked_degpt_id=-1;
	curMaxNumLinesegs = curMaxNum;
	nlinesegs = 0;
    infoGain=0.0;
    is_reach_degpt=false;
    is_reach_unknown=false;
}

Trajectory::~Trajectory()
{
	if(curMaxNumLinesegs > 0)
	{
		free(linesegs);
		curMaxNumLinesegs=0;
	}
}

bool Trajectory::extend_line_segments(int add_size)
{

	LineSeg *extendlist=(LineSeg*)malloc(sizeof(LineSeg)*(curMaxNumLinesegs+add_size));

	if(extendlist == NULL)
		//if(linesegs == NULL)
	{
		return false;
	}

	int i;
	for(i = 0; i < curMaxNumLinesegs; i++)
	{
		extendlist[i].end[0] = linesegs[i].end[0];
		extendlist[i].end[1] = linesegs[i].end[1];

		extendlist[i].start[0] = linesegs[i].start[0];
		extendlist[i].start[1] = linesegs[i].start[1];

		extendlist[i].gend[0] = linesegs[i].gend[0];
		extendlist[i].gend[1] = linesegs[i].gend[1];

		extendlist[i].gstart[0] = linesegs[i].gstart[0];
		extendlist[i].gstart[1] = linesegs[i].gstart[1];

		extendlist[i].length = linesegs[i].length;
		extendlist[i].Triangle_ID = linesegs[i].Triangle_ID;

	}
	free(linesegs);

	linesegs = extendlist;

	for(i=curMaxNumLinesegs;i<curMaxNumLinesegs+add_size;i++)
	{
		linesegs[i].gend[0]=linesegs[i].end[0]=linesegs[i].gstart[0]=linesegs[i].start[0]=
			linesegs[i].gend[1]=linesegs[i].end[1]=linesegs[i].gstart[1]=linesegs[i].start[1]=0.;
		linesegs[i].length=0;
		linesegs[i].Triangle_ID=0;
	}

	curMaxNumLinesegs += add_size;
	return true;
}

/*get the flow length of the streamline*/
double Trajectory::get_length()
{
	int i;
	double len = 0;
	for(i = 0 ; i < nlinesegs; i++)
		len += linesegs[i].length;
	return len;
}


//remove the front n line segments
bool Trajectory::remove_front_nlines(int n)
{
	if(nlinesegs-n<0) return false;
	/*move the content forward*/
	int i;
	for(i=0;i<nlinesegs-n;i++)
	{
		linesegs[i].gstart[0]=linesegs[i+n].gstart[0];
		linesegs[i].gstart[1]=linesegs[i+n].gstart[1];
		linesegs[i].gend[0]=linesegs[i+n].gend[0];
		linesegs[i].gend[1]=linesegs[i+n].gend[1];

		linesegs[i].start[0]=linesegs[i+n].start[0];
		linesegs[i].start[1]=linesegs[i+n].start[1];
		linesegs[i].end[0]=linesegs[i+n].end[0];
		linesegs[i].end[1]=linesegs[i+n].end[1];

		linesegs[i].length=linesegs[i+n].length;
		linesegs[i].Triangle_ID=linesegs[i+n].Triangle_ID;
	}
	nlinesegs-=n;
	return true;
}

//add n new line segments in the front
bool Trajectory::add_front_nlines(LineSeg *otherlinesegs, int n)
{
	if(nlinesegs+n>=curMaxNumLinesegs)
	{
		if(!extend_line_segments(nlinesegs+n-curMaxNumLinesegs))
			exit(-1);
	}
	/*move backward n elements*/
	int i;
	if(nlinesegs>0)
	{
		for(i=nlinesegs-1;i>=0;i--)
		{
			linesegs[i+n].gstart[0]=linesegs[i].gstart[0];
			linesegs[i+n].gstart[1]=linesegs[i].gstart[1];
			linesegs[i+n].gend[0]=linesegs[i].gend[0];
			linesegs[i+n].gend[1]=linesegs[i].gend[1];

			linesegs[i+n].start[0]=linesegs[i].start[0];
			linesegs[i+n].start[1]=linesegs[i].start[1];
			linesegs[i+n].end[0]=linesegs[i].end[0];
			linesegs[i+n].end[1]=linesegs[i].end[1];

			linesegs[i+n].length=linesegs[i].length;
			linesegs[i+n].Triangle_ID=linesegs[i].Triangle_ID;
		}
	}

	/*copy the new n line segments to the front*/
	for(i=0;i<n;i++)
	{
		linesegs[i].gstart[0]=otherlinesegs[i].gstart[0];
		linesegs[i].gstart[1]=otherlinesegs[i].gstart[1];
		linesegs[i].gend[0]=otherlinesegs[i].gend[0];
		linesegs[i].gend[1]=otherlinesegs[i].gend[1];

		linesegs[i].start[0]=otherlinesegs[i].start[0];
		linesegs[i].start[1]=otherlinesegs[i].start[1];
		linesegs[i].end[0]=otherlinesegs[i].end[0];
		linesegs[i].end[1]=otherlinesegs[i].end[1];

		linesegs[i].length=otherlinesegs[i].length;
		linesegs[i].Triangle_ID=otherlinesegs[i].Triangle_ID;
	}
	nlinesegs+=n;
	return true;
}

//remove the last n line segments
bool Trajectory::remove_last_nlines(int n)
{
	if(nlinesegs-n<0) return false;
	nlinesegs-=n;
	return true;
}

//add n new line segments at the end
bool Trajectory::add_last_nlines(LineSeg *otherlinesegs, int n)
{
	if(nlinesegs+n>=curMaxNumLinesegs)
	{
		if(!extend_line_segments(nlinesegs+n-curMaxNumLinesegs))
			exit(-1);
	}

	/*copy the content of "linesegs" to the end of current list*/
	int i;
	for(i=nlinesegs;i<n+nlinesegs;i++)
	{
		linesegs[i].gstart[0]=otherlinesegs[i-nlinesegs].gstart[0];
		linesegs[i].gstart[1]=otherlinesegs[i-nlinesegs].gstart[1];
		linesegs[i].gend[0]=otherlinesegs[i-nlinesegs].gend[0];
		linesegs[i].gend[1]=otherlinesegs[i-nlinesegs].gend[1];

		linesegs[i].start[0]=otherlinesegs[i-nlinesegs].start[0];
		linesegs[i].start[1]=otherlinesegs[i-nlinesegs].start[1];
		linesegs[i].end[0]=otherlinesegs[i-nlinesegs].end[0];
		linesegs[i].end[1]=otherlinesegs[i-nlinesegs].end[1];

		linesegs[i].length=otherlinesegs[i-nlinesegs].length;
		linesegs[i].Triangle_ID=otherlinesegs[i-nlinesegs].Triangle_ID;
	}
	nlinesegs+=n;
	return true;
}

bool Trajectory::store_to_global_line_segs(CurvePoints *temp, int num)
{
	int i;
	int tempid = nlinesegs;
	icVector3 dis_vec;

	////if the number of the line segements over the maximum number of the line segments each trajectory can store
	////extend the space for each trajectory
	if(tempid + num - 1 >= curMaxNumLinesegs)
	{
		//if(curMaxNumLinesegs>1000) 
		//	return false; // possible bug here! 12/27/2007

		if(!extend_line_segments(200))
		{
			return false;
		}
	}

	/*save to the global list*/

	for( i = 0; i < num-1; i++)
	{
		////Build the line segment
		linesegs[tempid+i].gstart[0] = temp[i].gpx;
		linesegs[tempid+i].gstart[1] = temp[i].gpy;
		linesegs[tempid+i].start[0] = temp[i].lpx;
		linesegs[tempid+i].start[1] = temp[i].lpy;

		linesegs[tempid+i].gend[0] = temp[i+1].gpx;
		linesegs[tempid+i].gend[1] = temp[i+1].gpy;
		linesegs[tempid+i].end[0] = temp[i+1].lpx;
		linesegs[tempid+i].end[1] = temp[i+1].lpy;

		////Use local coordinates to calculate the length
		dis_vec.entry[0] = temp[i+1].gpx - temp[i].gpx;
		dis_vec.entry[1] = temp[i+1].gpy - temp[i].gpy;
		dis_vec.entry[2] = 0;

		linesegs[tempid+i].length = length(dis_vec);

		linesegs[tempid+i].Triangle_ID = temp[i].triangleid;
	}

	nlinesegs = tempid + num - 1;
	return true;
}






