#pragma once
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>
#include "nrtypes_nr.h"
#include "nr.h"
using namespace std;
////routines for solving Runge_Kutta integration
void rkck(Vec_I_DP &y, Vec_I_DP &dydx, const DP x,
          const DP h, Vec_O_DP &yout, Vec_O_DP &yerr,
          void derivs(const DP, Vec_I_DP &, Vec_O_DP &));

void rkqs(Vec_IO_DP &y, Vec_IO_DP &dydx, DP &x, const DP htry,
          const DP eps, Vec_I_DP &yscal, DP &hdid, DP &hnext,
          void derivs(const DP, Vec_I_DP &, Vec_O_DP &));

/////functions for solving sparse linear system, move to another file for reuse in future 05/23/05
void sprsin(Mat_I_DP &a, const DP thresh, Vec_O_DP &sa, Vec_O_INT &ija);
DP  snrm(Vec_I_DP &sx, const int itol);
void  sprsax(Vec_I_DP &sa, Vec_I_INT &ija, Vec_I_DP &x, Vec_O_DP &b);
void  sprstx(Vec_I_DP &sa, Vec_I_INT &ija, Vec_I_DP &x, Vec_O_DP &b);
