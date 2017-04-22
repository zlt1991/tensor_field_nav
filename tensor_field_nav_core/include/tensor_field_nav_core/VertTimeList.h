#ifndef _VERTTIMELIST_H
#define _VERTTIMELIST_H
#include "dataStructure.h"
#include <stdio.h>
/*   Data structure for spatio-temporal relaxation   */
typedef struct OneVertAtTime{
    int time_step;
    int which_vert;
    int variableID;
    bool isConstraint;
}OneVertAtTime;


class VertTimeList{
public:
    OneVertAtTime **vertsTime;
    int nvertsTime;
    int curMaxNum;
    int total_notconstraint_verts;   // number of total nonconstrained vertices
    int nvertsPerFrame;              // number of vertices per frame

    /*  constructor   */
    VertTimeList(int nverts = 0, int initsize = 0);

    /*   destructor   */
    ~VertTimeList();
    /*  list operations  */
    void add_new(int time_step, int which_vert, bool isConstraint=false);

    void add_new(OneVertAtTime *oneVert);

    bool isFull();

    bool extend(int step=1000);

    void set_constraint_at_vert(int time_step, int which_vert);

    void reset_all_constraints();

    /*  return the slice index given the index of a variable   */
    int get_sliceID_given_varID(int variableID);
};


#endif
