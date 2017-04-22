#include "tensor_field_nav_core/TrajectoryList.h"

// The similar list operations
TrajectoryList::TrajectoryList(int initsize = 1000) //construction
{
	trajs = (Trajectory **)malloc(sizeof(Trajectory *)*initsize);
	curMaxNumTrajs = initsize;
	ntrajs = 0;

	if(trajs == NULL)
	{
		exit(-1);
	}

	for(int i = 0; i < initsize; i++)
		trajs[i] = NULL;
	curMaxNumTrajs = initsize;
} 

TrajectoryList::~TrajectoryList()
{
	int i, j;

	for(i = 0; i < curMaxNumTrajs; i++)
	{
		if(trajs[i] != NULL)
		{
			free(trajs[i]->linesegs);
		}
	}

	free(trajs);
}

//add a new vertex to the end of the list, if it succeeds, return true
 bool TrajectoryList::append(Trajectory *s)
{
	if(isFull ())
		if(!extend(100))
			return false;             //if not enough memory available, return false
	trajs[ntrajs] = s;
	//copyElem(s, polist[nporbits]);
	s->index=ntrajs;
	ntrajs++;
	return true;
} 

bool TrajectoryList::del_End() //delete the vertex at the end of the list
{
	if(isEmpty())  return false;
	ntrajs --;
	return true;
} 

void TrajectoryList::copy_Elem(Trajectory *s, Trajectory *d)
{
}

//delete the corresponding  vertex, if it succeeds, return true
bool TrajectoryList::del_Node(Trajectory *s) 
{
	if(isEmpty())  return false;

	//find the vertex, if find it, delete and move the following vertices forward
	//otherwise, return false;

	int i, pos = -1;

	for(i = 0; i < ntrajs; i++)
	{
		if(trajs[i] == s)
		{
			pos = i;
			break;
		}
	}

	if(pos == -1) return false;

	//delete it
	for(i = pos; i < ntrajs-1; i++)
	{
		//we need a copy function
		copy_Elem(trajs[i], trajs[i+1]);
	}

	ntrajs--;

	return true;
} 

bool TrajectoryList::isEmpty()  //judge whether the list is empty
{
	if(ntrajs == 0)   return true;
	return false;
}

bool TrajectoryList::isFull()
{
	if(ntrajs == curMaxNumTrajs) return true;
	return false;
}

//extend the original list, if it succeeds, return true
bool TrajectoryList::extend(int step = 100)
{
	Trajectory **temp = trajs;
	trajs = (Trajectory **) malloc(sizeof(Trajectory *) * (curMaxNumTrajs + step));
	if( trajs == NULL)
	{
		curMaxNumTrajs = 0;
		trajs = temp;
		exit(-1);

		return false;
	}

	int i;

	for(i = 0; i < curMaxNumTrajs; i++)
		trajs[i] = temp[i];
	for(i = curMaxNumTrajs; i < curMaxNumTrajs+step; i++)
		trajs[i] = NULL;

	curMaxNumTrajs += step;

	free(temp);
	return true;
}
void TrajectoryList::reset()
{
	ntrajs = 0;
}
