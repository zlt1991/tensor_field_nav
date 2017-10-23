/***
 basic structure for vertex in frame, used for contructing spatial-temporal constraints.
 ***/
#include "tensor_field_nav_core/VertTimeList.h"
VertTimeList::VertTimeList(int nverts, int initsize)
{
    if(initsize == 0)
    {
        vertsTime  = NULL;
        nvertsTime = curMaxNum    = 0;
        total_notconstraint_verts = 0;
        nvertsPerFrame = nverts;
        return;
    }

    vertsTime = (OneVertAtTime**)malloc(sizeof(OneVertAtTime*)*initsize);
    if(vertsTime == NULL) exit(-1);

    int i;
    for(i=0; i<initsize; i++)
        vertsTime[i]=NULL;

    nvertsTime = 0;
    curMaxNum  = initsize;
    total_notconstraint_verts = 0;
    nvertsPerFrame = nverts;
}

/*   destructor   */
VertTimeList::~VertTimeList()
{
    if(vertsTime == NULL)
        return;

    int i;
    for(i=0; i<curMaxNum; i++)
    {
        if(vertsTime[i]== NULL)
            continue;

        free(vertsTime[i]);
        vertsTime[i]=NULL;
    }

    free(vertsTime);
    vertsTime = NULL;
}

/*  list operations  */
void VertTimeList::add_new(int time_step, int which_vert, bool isConstraint)
{
    /*  allocate space for new element  */
    if(isFull())
    {
        if(!extend())
            exit(-1);
    }

    vertsTime[nvertsTime]=(OneVertAtTime*)malloc(sizeof(OneVertAtTime));
    vertsTime[nvertsTime]->time_step    = time_step;
    vertsTime[nvertsTime]->which_vert   = which_vert;
    vertsTime[nvertsTime]->isConstraint = isConstraint;
    vertsTime[nvertsTime]->variableID   = -1;

    nvertsTime++;
}

void VertTimeList::add_new(OneVertAtTime *oneVert)
{
    /*  allocate space for new element  */
    if(isFull())
    {
        if(!extend())
            exit(-1);
    }

    vertsTime[nvertsTime]=oneVert;
    nvertsTime++;
}

bool VertTimeList::isFull()
{
    if(nvertsTime >= curMaxNum) return true;
    return false;
}

bool VertTimeList::extend(int step)
{
    OneVertAtTime **temp=vertsTime;

    vertsTime=(OneVertAtTime**)malloc(sizeof(OneVertAtTime*)*(curMaxNum+step));

    if(vertsTime == NULL) return false;

    int i;
    for(i=0; i<curMaxNum; i++)
        vertsTime[i]=temp[i];

    for(i=curMaxNum; i<curMaxNum+step; i++)
        vertsTime[i]=NULL;

    free(temp);
    curMaxNum += step;

    return true;
}

void VertTimeList::set_constraint_at_vert(int time_step, int which_vert)
{
    vertsTime[time_step*nvertsPerFrame+which_vert]->isConstraint = true;
}

void VertTimeList::reset_all_constraints()
{
    int i;
    for(i=0; i<nvertsTime; i++)
        vertsTime[i]->isConstraint = false;
}

/*  return the slice index given the index of a variable   */
int VertTimeList::get_sliceID_given_varID(int variableID)
{
    int i;
    for(i=0; i<nvertsTime; i++)
    {
        if(vertsTime[i]->variableID == variableID)
            return vertsTime[i]->time_step;
    }

    return -1;
}
