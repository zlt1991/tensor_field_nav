/***
 Some useful functions
***/
#ifndef _UTIL_H
#define _UTIL_H
#include <stdlib.h>
#include <math.h>
#include <vector>
#include "icVector.h"
#include "nrtypes_nr.h"
typedef int32_t int32;
typedef uint32_t uint32;

struct Location {
    int32 x;
    int32 y;
    Location() { x=0, y=0; }
    Location(int32 a, int32 b) { x=a, y=b; }
};

void transform_point3D(float p[3], float rot_mat[16]);
double bilinear_interpolate(double a, double b, double f00, double f01, double f10, double f11);
bool is_repeated_elem(int *a, int b, int num);
inline double fabs (double x){return x>0?x:-x; }
inline double max (double x,double y){return x>y?x:y; }
inline double min (double x,double y){return x>y?y:x; }

int GetIntersection2(double PointA[2], double PointB[2], double PointC[2], double PointD[2], double t[2]);
double * MatrixInver(double A[],int m,int n);
double *MatrixOpp(double A[],int m,int n);
double Surplus(double A[],int m,int n) ;
int solve_ten_cubic(double a, double b, double c, double d, double solutions[4]);
int solve_ten_cubic_3(double a, double b, double c, double d, double solutions[4]);
int solve_ten_cubic_2(double a, double b, double c, double d, double solutions[4]);
int solve_ten_quadratic(double a, double b, double c, double solutions[2]);
int get_sign(double x);
bool SameSide(icVector3 A, icVector3 B, icVector3 C, icVector3 P);
bool PointInTriangle(icVector3 A, icVector3 B, icVector3 C, icVector3 P);
void linbcg(Vec_I_DP &b, Vec_IO_DP &x, Vec_INT *ija_p,Vec_DP *sa_p,const int itol, const DP tol,const int itmax, int &iter, DP &err);
void BubbleSorting(int *a, int size);
void atimes(Vec_I_DP &x, Vec_O_DP &r,Vec_INT *ija_p,Vec_DP *sa_p, const int itrnsp);
void asolve(Vec_I_DP &b, Vec_O_DP &x,Vec_DP *sa_p, const int itrnsp);
void sprsin(Mat_I_DP &a, const DP thresh, Vec_O_DP &sa, Vec_O_INT &ija);
DP  snrm(Vec_I_DP &sx, const int itol);
void  sprsax(Vec_I_DP &sa, Vec_I_INT &ija, Vec_I_DP &x, Vec_O_DP &b);
void  sprstx(Vec_I_DP &sa, Vec_I_INT &ija, Vec_I_DP &x, Vec_O_DP &b);
void Bresenham(int32 x1, int32 y1, int32 x2, int32 y2, vector<Location>& locationVec);
void CalcBresenhamLocs(const Location& startPos, const Location& endPos, vector<Location>& locationVec);
#endif
