/***
 Some useful functions
***/
#include "tensor_field_nav_core/Util.h"

void transform_point3D(float p[3], float rot_mat[16])
{
    double tmp[3] = {0.};

    tmp[0] = rot_mat[0] * p[0] + rot_mat[4] * p[1] + rot_mat[8]  * p[2] + rot_mat[12];
    tmp[1] = rot_mat[1] * p[0] + rot_mat[5] * p[1] + rot_mat[9]  * p[2] + rot_mat[13];
    tmp[2] = rot_mat[2] * p[0] + rot_mat[6] * p[1] + rot_mat[10] * p[2] + rot_mat[14];

    p[0] = tmp[0];
    p[1] = tmp[1];
    p[2] = tmp[2];
}

double bilinear_interpolate(double a, double b,
                                     double f00, double f01, double f10, double f11)
{
    return (f00*(1-a)*(1-b)+f10*a*(1-b)+f01*(1-a)*b+f11*a*b);
}
bool is_repeated_elem(int *a, int b, int num)
{
    int i;
    for(i = 0; i < num; i++)
    {
        if(a[i] == b)
            return true;
    }
    return false;
}
/* meaning of return value
 0----Intersection dosn't exists
 1----Intersection exists.
 2----two line segments are parallel.
 3----two line segments are collinear, but not overlap.
 4----two line segments are collinear, and share one same end point.
 5----two line segments are collinear, and overlap.
*/

int GetIntersection2(double PointA[2], double PointB[2], double PointC[2], double PointD[2], double t[2])
{

    double delta;
    double t1,t2;
    double a,b,c,d;
    double xba,yba,xdc,ydc,xca,yca;

    xba=PointB[0]-PointA[0];    yba=PointB[1]-PointA[1];
    xdc=PointD[0]-PointC[0];    ydc=PointD[1]-PointC[1];
    xca=PointC[0]-PointA[0];    yca=PointC[1]-PointA[1];

    delta=xba*ydc-yba*xdc;
    t1=xca*ydc-yca*xdc;
    t2=xca*yba-yca*xba;

    if(delta!=0)
    {
        t[0]=t1/delta;   t[1]=t2/delta;
        /*two segments intersect (including intersect at end points)*/
        //if ( t[0]<=1 && t[0]>=0 && t[1]<=1 && t[1]>=0 ) return 1;
        if ( t[0]<=1 && (t[0]>=0 || fabs (t[0])<=1.e-8)
            && t[1]<=1 && (t[1]>=0|| fabs (t[1])<=1.e-8)) //set threshold to allow some numerical errors
            return 1;
        else return 0;
    }

    else
    {
        /* AB & CD are parallel. */
        if ( (t1!=0) && (t2!=0) ) return 2;

        /* when AB & CD are collinear */

        /*if AB isn't a vertical line segment, project to x-axis */
        if(PointA[0]!=PointB[0])
        {
            a=min(PointA[0],PointB[0]); b=max(PointA[0],PointB[0]);
            c=min(PointC[0],PointD[0]); d=max(PointC[0],PointD[0]);

            if ( (d<a) || (c>b) ) return  3;
            else if( (d==a) || (c==b) ) return 4;
            else return 5;
        }

        else         /* if AB is a vertical line segment, project to y-axis */
        {

            a=min(PointA[1],PointB[1]); b=max(PointA[1],PointB[1]);
            c=min(PointC[1],PointD[1]); d=max(PointC[1],PointD[1]);

            if( (d<a) || (c>b) ) return  3;
            else if( (d==a) || (c==b) ) return 4;
            else return 5;
        }
    }
}
double * MatrixInver(double A[],int m,int n) //zhuanzhi
{
    int i,j;
    double *B=NULL;
    B=(double *)malloc(m*n*sizeof(double));

    for(i=0;i<n;i++)
        for(j=0;j<m;j++)
            B[i*m+j]=A[j*n+i];

    return B;
}

double *MatrixOpp(double A[],int m,int n) //inverse
{
    int i,j,x,y,k;
    double *SP=NULL,*AB=NULL,*B=NULL,XX,*C;
    SP=(double *)malloc(m*n*sizeof(double));
    AB=(double *)malloc(m*n*sizeof(double));
    B=(double *)malloc(m*n*sizeof(double));

    XX=Surplus(A,m,n);
    XX=1/XX;

    for(i=0;i<m;i++)
        for(j=0;j<n;j++)
        {
            for(k=0;k<m*n;k++)
                B[k]=A[k];
            {
                for(x=0;x<n;x++)
                    B[i*n+x]=0;
                for(y=0;y<m;y++)
                    B[m*y+j]=0;
                B[i*n+j]=1;
                SP[i*n+j]=Surplus(B,m,n);
                AB[i*n+j]=XX*SP[i*n+j];
            }
        }

        C=MatrixInver(AB,m,n);

        free(SP);
        free(AB);
        free(B);

        return C;
}

double Surplus(double A[],int m,int n) //hanglieshi
{

    int i,j,k,p,r;
    double XX,temp=1,temp1=1,s=0,s1=0;

    if(n==2)
    {for(i=0;i<m;i++)
    for(j=0;j<n;j++)
        if((i+j)%2) temp1*=A[i*n+j];
        else temp*=A[i*n+j];
        XX=temp-temp1;}
    else{
        for(k=0;k<n;k++)
        {for(i=0,j=k;i<m,j<n;i++,j++)
        temp*=A[i*n+j];
        if(m-i)
        {for(p=m-i,r=m-1;p>0;p--,r--)
        temp*=A[r*n+p-1];}
        s+=temp;
        temp=1;
        }

        for(k=n-1;k>=0;k--)
        {for(i=0,j=k;i<m,j>=0;i++,j--)
        temp1*=A[i*n+j];
        if(m-i)
        {for(p=m-1,r=i;r<m;p--,r++)
        temp1*=A[r*n+p];}
        s1+=temp1;
        temp1=1;
        }

        XX=s-s1;}
    return XX;
}

int solve_ten_cubic(double a, double b, double c, double d, double solutions[4])
{
    if(fabs(a) < 1e-6)
        return solve_ten_quadratic(b, c, d, solutions);
    b /= a;
    c /= a;
    d /= a;
    double Q = (9.*b*c-27.*d-2.*b*b*b)/54.;
    double D = (3.*c-b*b)/9.;
    double R = D*D*D + Q*Q;
    double S, T;

    if(R>=0)
    {
        double s_R = sqrt(R);
        S = get_sign(Q+s_R)*pow(fabs(Q+s_R), 1./3);
        T = get_sign(Q-s_R)*pow(fabs(Q-s_R), 1./3);

        solutions[0] = -b/3.+(S+T);
        solutions[1] = -b/3.-(S+T)/2.;
        solutions[2] = -b/3.-(S+T)/2.;
        solutions[3] = fabs(sqrt(3.)/2.*(S-T));
        if(solutions[3] == 0) /*no complex roots*/
        {
            if(solutions[1] != solutions[0] )
                return 2;  /*two real roots*/
            else
                return 1;
        }
        else /*contains two complex roots*/
        {
            return 4;
        }
    }

    else   /*we have distinct real roots*/
    {
        double th = acos(Q/sqrt(-D*D*D));
        solutions[0] = 2*sqrt(-D)*cos(th/3.)-b/3;
        solutions[1] = 2*sqrt(-D)*cos((th+2*M_PI)/3.)-b/3;
        solutions[2] = 2*sqrt(-D)*cos((th+4*M_PI)/3.)-b/3.;
        return 3;
    }
    return 0;
}


int solve_ten_cubic_3(double a, double b, double c, double d, double solutions[4])
{
    if(fabs(a) < 1e-6)
        return solve_ten_quadratic(b, c, d, solutions);
    double Q = (9*a*b*c-27*a*a*d-2*b*b*b)/(54*a*a*a);
    double D = (3*a*c-b*b)/(9*a*a);
    double R = D*D*D + Q*Q;
    double S, T;

    if(R>=0)
    {
        double s_R = sqrt(R);
        S = get_sign(Q+s_R)*pow(fabs(Q+s_R), 1./3);
        T = get_sign(Q-s_R)*pow(fabs(Q-s_R), 1./3);

        solutions[0] = -b/(3*a)+(S+T);
        solutions[1] = -b/(3*a)-(S+T)/2.;
        solutions[2] = -b/(3*a)-(S+T)/2.;
        solutions[3] = fabs(sqrt(3.)/2.*(S-T));
        if(solutions[3] == 0) /*no complex roots*/
        {
            if(solutions[1] != solutions[0] )
                return 2;  /*two real roots*/
            else
                return 1;
        }
        else /*contains two complex roots*/
        {
            return 4;
        }
    }

    else   /*we have distinct real roots*/
    {
        double th = acos(Q/sqrt(-D*D*D));
        solutions[0] = 2*sqrt(-D)*cos(th/3.)-b/(3*a);
        solutions[1] = 2*sqrt(-D)*cos((th+2*M_PI)/3.)-b/(3*a);
        solutions[2] = 2*sqrt(-D)*cos((th+4*M_PI)/3.)-b/(3*a);
        return 3;
    }
    return 0;
}

/*   a*x3 + b*x2 + c*x + d = 0

Formula:
  Step 1: Calculate p and q
          p = ( 3*c/a - (b/a)2 ) / 3
          q = ( 2*(b/a)3 - 9*b*c/a/a + 27*d/a ) / 27
  Step 2: Calculate discriminant D
          D = (p/3)3 + (q/2)2
  Step 3: Depending on the sign of D, you follow different strategy.
          If D<0, three distinct real roots.
          If D=0, three real roots of which at least two are equal.
          If D>0, one real and two complex roots.
  Step 3a: For D>0 and D=0
          Calculate u and v
          u = cubic_root(-q/2 + sqrt(D))
          v = cubic_root(-q/2 - sqrt(D))
          Find the three transformed roots
          y1 = u + v
          y2 = -(u+v)/2 + i (u-v)*sqrt(3)/2
          y3 = -(u+v)/2 - i (u-v)*sqrt(3)/2
  Step 3b: Alternately, for D<0, a trigonometric formulation is more convenient
          y1 =  2 * sqrt(|p|/3) * cos(phi/3)
          y2 = -2 * sqrt(|p|/3) * cos((phi+pi)/3)
          y3 = -2 * sqrt(|p|/3) * cos((phi-pi)/3)
          where phi = acos(-q/2/sqrt(|p|3/27))
                pi  = 3.141592654...
  Step 4  Finally, find the three roots
          x = y - b/a/3

*/
int solve_ten_cubic_2(double a, double b, double c, double d, double solutions[4])
{
    double p = (3*c/a - (b/a)*(b/a))/3.;
    double q = (2*pow((b/a),3) - 9*b*c/a/a + 27*d/a)/27.;
    double D = pow((p/3),3) + (q/2)*(q/2);

    if(D < 0 )/*3 distinct real roots*/
    {
        double phi = acos(-q/2/sqrt(pow(fabs(p),3)/27));
        solutions[0] =  2 * sqrt(fabs(p)/3) * cos(phi/3);
        solutions[1] = -2 * sqrt(fabs(p)/3) * cos((phi+M_PI)/3);
        solutions[2] = -2 * sqrt(fabs(p)/3) * cos((phi-M_PI)/3);
    }
    else if(D>=0)
    {
        double u = pow((-q/2 + sqrt(D)), 1./3.);
        double v = pow((-q/2 - sqrt(D)), 1./3.);
        solutions[0] = u + v;

        if(D == 0)
        {
            solutions[1] = -(u+v)/2;
            if(solutions[0] == solutions[1])
                return 1;
            else
                return 2;
        }

        return 1;
    }
}


int solve_ten_quadratic(double a, double b, double c, double solutions[2])
{
    double D = b*b-4*a*c;
    if(D>=0)
    {
        if(D==0)
        {
            solutions[0] = -b/(2*a); return 1;
        }
        else
        {
            D=sqrt(D);
            solutions[0] = -b/(2*a)+D;
            solutions[1] = -b/(2*a)-D;
            return 2;
        }
    }
    return 0;  /*there is no real root*/
}



int get_sign(double x)
{
    if(x>=0.0) return 1;
    else return -1;
}

bool SameSide(icVector3 A, icVector3 B, icVector3 C, icVector3 P)
{
    icVector3 AB = B - A ;
    icVector3 AC = C - A ;
    icVector3 AP = P - A ;

    icVector3 v1 = cross(AB,AC) ;
    icVector3 v2 = cross(AB,AP) ;

    // v1 and v2 should point to the same direction
    return dot(v1,v2) >= 0 ;
}

// Same side method
// Determine whether point P in triangle ABC
bool PointInTriangle(icVector3 A, icVector3 B, icVector3 C, icVector3 P)
{
    return SameSide(A, B, C, P) &&
        SameSide(B, C, A, P) &&
        SameSide(C, A, B, P) ;
}

void BubbleSorting(int *a, int size){
    int i, j, temp;
    for ( i = 0; i < size; i++ )    // controls passes through the list
    {
        for ( j = 0; j < size - 1; j++ )   // performs adjacent comparisons
        {
            if(a[j] > a[j+1])      // determines if a swap should occur
            {
                temp = a[j];       // swap is performed
                a[j] = a[j+1];
                a[j+1] = temp;
            }
        }
    }
}
void linbcg(Vec_I_DP &b, Vec_IO_DP &x, Vec_INT *ija_p,Vec_DP *sa_p,const int itol, const DP tol,const int itmax, int &iter, DP &err){
    DP ak,akden,bk,bkden=1.0,bknum,bnrm,dxnrm,xnrm,zm1nrm,znrm;
    const DP EPS=1.0e-14;
    int j;

    int n=b.size();
    Vec_DP p(n),pp(n),r(n),rr(n),z(n),zz(n);
    iter=0;
    atimes(x,r,ija_p,sa_p,0);
    for (j=0;j<n;j++) {
        r[j]=b[j]-r[j];
        rr[j]=r[j];
    }
    //atimes(r,rr,0);
    if (itol == 1) {
        bnrm=snrm(b,itol);
        asolve(r,z,sa_p,0);
    }
    else if (itol == 2) {
        asolve(b,z,sa_p,0);
        bnrm=snrm(z,itol);
        asolve(r,z,sa_p,0);
    }
    else if (itol == 3 || itol == 4) {
        asolve(b,z,sa_p,0);
        bnrm=snrm(z,itol);
        asolve(r,z,sa_p,0);
        znrm=snrm(z,itol);
    } else printf("illegal itol in linbcg");
    //cout << fixed << setprecision(6);
    while (iter < itmax) {
        ++iter;
        asolve(rr,zz,sa_p,1);
        for (bknum=0.0,j=0;j<n;j++) bknum += z[j]*rr[j];
        if (iter == 1) {
            for (j=0;j<n;j++) {
                p[j]=z[j];
                pp[j]=zz[j];
            }
        } else {
            bk=bknum/bkden;
            for (j=0;j<n;j++) {
                p[j]=bk*p[j]+z[j];
                pp[j]=bk*pp[j]+zz[j];
            }
        }
        bkden=bknum;
        atimes(p,z,ija_p,sa_p,0);
        for (akden=0.0,j=0;j<n;j++) akden += z[j]*pp[j];
        if(akden != 0)
            ak=bknum/akden;
        else
            ak = bknum;
        atimes(pp,zz,ija_p,sa_p,1);
        for (j=0;j<n;j++) {
            x[j] += ak*p[j];
            r[j] -= ak*z[j];
            rr[j] -= ak*zz[j];
        }
        asolve(r,z,sa_p,0);
        if (itol == 1)
        {
            if(bnrm != 0)
                err=snrm(r,itol)/bnrm;
            else
                err=snrm(r,itol);
        }
        else if (itol == 2)
            err=snrm(z,itol)/bnrm;
        else if (itol == 3 || itol == 4) {
            zm1nrm=znrm;
            znrm=snrm(z,itol);
            if (fabs(zm1nrm-znrm) > EPS*znrm) {
                dxnrm=fabs(ak)*snrm(p,itol);
                err=znrm/fabs(zm1nrm-znrm)*dxnrm;
            } else {
                err=znrm/bnrm;
                continue;
            }
            xnrm=snrm(x,itol);
            if (err <= 0.5*xnrm) err /= xnrm;
            else {
                err=znrm/bnrm;
                continue;
            }
        }
       // cout << "iter=" << setw(4) << iter+1 << setw(12) << err << endl;
        if (err <= tol) break;
    }
}

void atimes(Vec_I_DP &x, Vec_O_DP &r,Vec_INT *ija_p,Vec_DP *sa_p, const int itrnsp)
{
    if (itrnsp) sprstx(*sa_p,*ija_p,x,r);
    else sprsax(*sa_p,*ija_p,x,r);
}


void asolve(Vec_I_DP &b, Vec_O_DP &x,Vec_DP *sa_p,const int itrnsp)
{
    int i;

    int n=b.size();
    for(i=0;i<n;i++) x[i]=((*sa_p)[i] != 0.0 ? b[i]/(*sa_p)[i] : b[i]);
}


/*--------------------------------------------------------------*/
/////functions for solving sparse linear system
void sprsin(Mat_I_DP &a, const DP thresh, Vec_O_DP &sa, Vec_O_INT &ija)
{
    int i,j,k;

    int n=a.nrows();
    int nmax=sa.size();
    for (j=0;j<n;j++) sa[j]=a[j][j];
    ija[0]=n+1;
    k=n;
    for (i=0;i<n;i++) {
        for (j=0;j<n;j++) {
            if (fabs(a[i][j]) >= thresh && i != j) {
                if (++k > nmax) printf("sprsin: sa and ija too small");
                sa[k]=a[i][j];
                ija[k]=j;
            }
        }
        ija[i+1]=k+1;
    }
}

DP snrm(Vec_I_DP &sx, const int itol)
{
    int i,isamax;
    DP ans;

    int n=sx.size();
    if (itol <= 3) {
        ans = 0.0;
        for (i=0;i<n;i++) ans += sx[i]*sx[i];
        return sqrt(ans);
    } else {
        isamax=0;
        for (i=0;i<n;i++) {
            if (fabs(sx[i]) > fabs(sx[isamax])) isamax=i;
        }
        return fabs(sx[isamax]);
    }
}



void sprsax(Vec_I_DP &sa, Vec_I_INT &ija, Vec_I_DP &x, Vec_O_DP &b)
{
    int i,k;

    int n=x.size();
    if (ija[0] != n+1)
        printf("sprsax: mismatched vector and matrix");
    for (i=0;i<n;i++) {
        b[i]=sa[i]*x[i];
        for (k=ija[i];k<ija[i+1];k++) {
            b[i] += sa[k]*x[ija[k]];
        }
    }
}


void sprstx(Vec_I_DP &sa, Vec_I_INT &ija, Vec_I_DP &x, Vec_O_DP &b)
{
    int i,j,k;

    int n=x.size();
    if (ija[0] != (n+1))
        printf("mismatched vector and matrix in sprstx");
    for (i=0;i<n;i++) b[i]=sa[i]*x[i];
    for (i=0;i<n;i++) {
        for (k=ija[i];k<ija[i+1];k++) {
            j=ija[k];
            b[j] += sa[k]*x[i];
        }
    }
}

void Bresenham(int32 x1, int32 y1, int32 x2, int32 y2, vector<Location>& locationVec)
{
        bool swapflag = false;
        if (x1 > x2){
                int32 tmpx = x1;
                int32 tmpy = y1;
                x1 = x2;
                y1 = y2;
                x2 = tmpx;
                y2 = tmpy;
                swapflag = true;
        }

        int32 dx = x2-x1;
        int32 dy = y2-y1;
        int32 x = x1;
        int32 y = y1;
        int32 sub = (dy<<1)-dx;
        locationVec.push_back(Location(x, y));
        while(x<x2){
                ++x;
                if (sub > 0){
                        sub += (dy<<1) - (dx<<1);
                        ++y;
                }else {
                        sub += (dy<<1);
                }
                locationVec.push_back(Location(x, y));
        }

        if (swapflag){
                uint32 size = locationVec.size();
                for (uint32 i = 0; i < size/2 ; ++i){
                        Location tmp = locationVec[i];
                        locationVec[i] = locationVec[size-i-1];
                        locationVec[size-i-1] = tmp;
                }
        }
}

void CalcShortestDistance(const Location& startPos, const Location& endPos, vector<Location>& locationVec)
{
    if (startPos.x==endPos.x && startPos.y==endPos.y)
        return ;

    if (endPos.x == startPos.x){ //x相同
        if (endPos.y > startPos.y){
            for (uint32 i = 0; i < (uint32)(endPos.y-startPos.y); ++i){
                locationVec.push_back(Location(startPos.x, startPos.y+i+1));
            }
        }else{
            for (uint32 i = 0; i < (uint32)(startPos.y-endPos.y); ++i){
                locationVec.push_back(Location(startPos.x, startPos.y-i-1));
            }
        }
        return ;
    }

    float k = (float)(endPos.y-startPos.y)/(endPos.x-startPos.x);

    if (k >= 0 && k <= 1){ //斜率为0~1

        Bresenham(startPos.x,startPos.y,endPos.x,endPos.y,locationVec);

    }else if (k > 1){ //斜率为1~无穷大

        Bresenham(startPos.y,startPos.x,endPos.y,endPos.x,locationVec);
        for (vector<Location>::iterator it = locationVec.begin(); it!=locationVec.end(); ++it){
            int tmp = (*it).x;
            (*it).x = (*it).y;
            (*it).y = tmp;
        }

    }else if (k >= -1 && k < 0){ //斜率为-1~0

        Bresenham(startPos.x,-startPos.y,endPos.x,-endPos.y,locationVec);
        for (vector<Location>::iterator it = locationVec.begin(); it!=locationVec.end(); ++it)
            (*it).y = -(*it).y;

    }else if (k < -1){ //斜率为无穷小~-1

        Bresenham(-startPos.y,startPos.x,-endPos.y,endPos.x,locationVec);
        for (vector<Location>::iterator it = locationVec.begin(); it!=locationVec.end(); ++it){
            int tmp = (*it).x;
            (*it).x = (*it).y;
            (*it).y = tmp;
            (*it).y = -(*it).y;
        }
    }

    locationVec.erase(locationVec.begin());
}


void CalcBresenhamLocs(const Location& startPos, const Location& endPos, vector<Location>& locationVec)
{
    if (startPos.x==endPos.x && startPos.y==endPos.y)
        return ;

    if (endPos.x == startPos.x){ //x相同
        if (endPos.y > startPos.y){
            for (uint32 i = 0; i < (uint32)(endPos.y-startPos.y); ++i){
                locationVec.push_back(Location(startPos.x, startPos.y+i+1));
            }
        }else{
            for (uint32 i = 0; i < (uint32)(startPos.y-endPos.y); ++i){
                locationVec.push_back(Location(startPos.x, startPos.y-i-1));
            }
        }
        return ;
    }

    float k = (float)(endPos.y-startPos.y)/(endPos.x-startPos.x);

    if (k >= 0 && k <= 1){ //斜率为0~1

        Bresenham(startPos.x,startPos.y,endPos.x,endPos.y,locationVec);

    }else if (k > 1){ //斜率为1~无穷大

        Bresenham(startPos.y,startPos.x,endPos.y,endPos.x,locationVec);
        for (vector<Location>::iterator it = locationVec.begin(); it!=locationVec.end(); ++it){
            int tmp = (*it).x;
            (*it).x = (*it).y;
            (*it).y = tmp;
        }

    }else if (k >= -1 && k < 0){ //斜率为-1~0

        Bresenham(startPos.x,-startPos.y,endPos.x,-endPos.y,locationVec);
        for (vector<Location>::iterator it = locationVec.begin(); it!=locationVec.end(); ++it)
            (*it).y = -(*it).y;

    }else if (k < -1){ //斜率为无穷小~-1

        Bresenham(-startPos.y,startPos.x,-endPos.y,endPos.x,locationVec);
        for (vector<Location>::iterator it = locationVec.begin(); it!=locationVec.end(); ++it){
            int tmp = (*it).x;
            (*it).x = (*it).y;
            (*it).y = tmp;
            (*it).y = -(*it).y;
        }
    }

    locationVec.erase(locationVec.begin());
}

