#ifndef icMatrix_is_defined
#define icMatrix_is_defined

class icMatrix2x2;
class icMatrix3x3;

extern "C" {
#include <math.h>
#include <stdlib.h>
}
#include "icVector.h"

// start for class icMatrix2x2
class icMatrix2x2 {
public:
  inline icMatrix2x2();
  inline icMatrix2x2(double x);
  inline icMatrix2x2(const icMatrix2x2 &that);

  inline icMatrix2x2(double M00, double M01, 
		     double M10, double M11);
  inline icMatrix2x2(double M[2][2]);

  inline icMatrix2x2 &set      (const double d);
  inline icMatrix2x2 &operator=(const double d);

  inline icMatrix2x2 &set      (const icMatrix2x2 &that);  
  inline icMatrix2x2 &operator=(const icMatrix2x2 &that); 

	inline icMatrix2x2 &set			 (double M[2][2]);
  inline icMatrix2x2 &operator=(double M[2][2]); 

  inline int operator!=(const icMatrix2x2 &that)const; 
  inline int operator==(const icMatrix2x2 &that)const; 

  inline int operator==(double d) const;
  inline int operator!=(double d) const;
  
  inline icMatrix2x2 &operator+=(double d);
  inline icMatrix2x2 &operator-=(double d);
  inline icMatrix2x2 &operator*=(double d);

  // component-wise operations.
  inline icMatrix2x2 &operator+=(const icMatrix2x2 &that);
  inline icMatrix2x2 &operator-=(const icMatrix2x2 &that);
  inline icMatrix2x2 &operator*=(const icMatrix2x2 &that);

  // Left : this = that x this  
  // Right: this = this x that
  icMatrix2x2 &leftMultiply (const icMatrix2x2 &that);
  icMatrix2x2 &rightMultiply(const icMatrix2x2 &that);

  inline icMatrix2x2 &setIdentity     ();

public:
  double entry[2][2];

};

inline icMatrix2x2 operator+(const icMatrix2x2 &a, double b);
inline icMatrix2x2 operator-(const icMatrix2x2 &a, double b);
inline icMatrix2x2 operator*(const icMatrix2x2 &a, double b);

inline icMatrix2x2 operator+(const icMatrix2x2 &a, const icMatrix2x2 &b);
inline icMatrix2x2 operator-(const icMatrix2x2 &a, const icMatrix2x2 &b);
inline icMatrix2x2 operator*(const icMatrix2x2 &a, const icMatrix2x2 &b); 

inline icMatrix2x2 multiply(const icMatrix2x2 &a, const icMatrix2x2 &b); 
inline icVector2   operator*(const icMatrix2x2 &a, const icVector2   &b);
inline icVector2   operator*(const icVector2   &a, const icMatrix2x2 &b);

inline double determinant(const icMatrix2x2 &a);

inline icMatrix2x2 transpose(const icMatrix2x2 &a);
inline icMatrix2x2   inverse(const icMatrix2x2 &a);

inline icMatrix2x2::icMatrix2x2() {
  entry[0][0] = 1;
  entry[0][1] = 0;
  entry[1][0] = 0;
  entry[1][1] = 1;
}

inline icMatrix2x2::icMatrix2x2(double x) {
  entry[0][0] = x;
  entry[0][1] = x;
  entry[1][0] = x;
  entry[1][1] = x;
}

inline icMatrix2x2::icMatrix2x2(double M00, double M01, 
				double M10, double M11) {
  entry[0][0] = M00;
  entry[0][1] = M01;
  entry[1][0] = M10;
  entry[1][1] = M11;
};

inline icMatrix2x2::icMatrix2x2(const icMatrix2x2 &that) {
  entry[0][0] = that.entry[0][0];
  entry[0][1] = that.entry[0][1];
  entry[1][0] = that.entry[1][0];
  entry[1][1] = that.entry[1][1];
};

inline icMatrix2x2 &icMatrix2x2::set(const double d) {
  return (*this)=d;
}

inline icMatrix2x2 &icMatrix2x2::operator=(const double d) {
  entry[0][0] = d;
  entry[0][1] = d;

  entry[1][0] = d;
  entry[1][1] = d;
  return (*this);
};

inline icMatrix2x2 &icMatrix2x2::set(const icMatrix2x2 &that) {
  return (*this)=that;
}

inline icMatrix2x2 &icMatrix2x2::operator=(const icMatrix2x2 &that) {
  entry[0][0] = that.entry[0][0];
  entry[0][1] = that.entry[0][1];

  entry[1][0] = that.entry[1][0];
  entry[1][1] = that.entry[1][1];
  return (*this);
};

inline icMatrix2x2 &icMatrix2x2::set(double M[2][2]) {
  return (*this)=M;
}

inline icMatrix2x2 &icMatrix2x2::operator=(double M[2][2]) {
  entry[0][0] = M[0][0];
  entry[0][1] = M[0][1];

  entry[1][0] = M[1][0];
  entry[1][1] = M[1][1];
  return (*this);
};

inline int icMatrix2x2::operator==(double d) const {
  return  ( (entry[0][0] == d) &&
	    (entry[0][1] == d) &&
	    (entry[1][0] == d) &&
	    (entry[1][1] == d) );
}

inline int icMatrix2x2::operator!=(double d) const {
  return  ( (entry[0][0] != d) ||
	    (entry[0][1] != d) ||
	    (entry[1][0] != d) ||
	    (entry[1][1] != d) );
}
  
inline int icMatrix2x2::operator==(const icMatrix2x2 &that)const {
  return ( (entry[0][0] == that.entry[0][0]) &&
	   (entry[0][1] == that.entry[0][1]) &&
	   (entry[1][0] == that.entry[1][0]) &&
	   (entry[1][1] == that.entry[1][1]) );
}

inline int icMatrix2x2::operator!=(const icMatrix2x2 &that)const {
  return ( (entry[0][0] != that.entry[0][0]) ||
	   (entry[0][1] != that.entry[0][1]) ||
	   (entry[1][0] != that.entry[1][0]) ||
	   (entry[1][1] != that.entry[1][1]) );
}

inline icMatrix2x2 &icMatrix2x2::operator+=(double d) {
  entry[0][0] += d; entry[1][0] += d; 
  entry[0][1] += d; entry[1][1] += d; 
  return (*this);
}

inline icMatrix2x2 &icMatrix2x2::operator-=(double d) {
  entry[0][0] -= d; entry[1][0] -= d; 
  entry[0][1] -= d; entry[1][1] -= d; 
  return (*this);
}

inline icMatrix2x2 &icMatrix2x2::operator*=(double d) {
  entry[0][0] *= d; entry[1][0] *= d; 
  entry[0][1] *= d; entry[1][1] *= d; 
  return (*this);
}

inline icMatrix2x2 &icMatrix2x2::operator+=(const icMatrix2x2 &that) {
  entry[0][0] += that.entry[0][0]; entry[1][0] += that.entry[1][0]; 
  entry[0][1] += that.entry[0][1]; entry[1][1] += that.entry[1][1]; 
  return (*this);
}
  
inline icMatrix2x2 &icMatrix2x2::operator-=(const icMatrix2x2 &that) {
  entry[0][0] -= that.entry[0][0]; entry[1][0] -= that.entry[1][0]; 
  entry[0][1] -= that.entry[0][1]; entry[1][1] -= that.entry[1][1]; 
  return (*this);
}

inline icMatrix2x2 &icMatrix2x2::operator*=(const icMatrix2x2 &that) {
  entry[0][0] *= that.entry[0][0]; entry[1][0] *= that.entry[1][0]; 
  entry[0][1] *= that.entry[0][1]; entry[1][1] *= that.entry[1][1]; 
  return (*this);
}

inline icMatrix2x2 &icMatrix2x2::leftMultiply (const icMatrix2x2 &that){
	icMatrix2x2 tmp(entry[0][0], entry[0][1], entry[1][0], entry[1][1]);
	
	entry[0][0] = that.entry[0][0] * tmp.entry[0][0] + that.entry[0][1] * tmp.entry[1][0];
	entry[0][1] = that.entry[0][0] * tmp.entry[0][1] + that.entry[0][1] * tmp.entry[1][1];
	entry[1][0] = that.entry[1][0] * tmp.entry[0][0] + that.entry[1][1] * tmp.entry[1][0];
	entry[1][1] = that.entry[1][0] * tmp.entry[0][1] + that.entry[1][1] * tmp.entry[1][1];
	return (*this);
};

inline icMatrix2x2 &icMatrix2x2::rightMultiply(const icMatrix2x2 &that){
	icMatrix2x2 tmp(entry[0][0], entry[0][1], entry[1][0], entry[1][1]);

	entry[0][0] = tmp.entry[0][0] * that.entry[0][0] + tmp.entry[0][1] * that.entry[1][0];
	entry[0][1] = tmp.entry[0][0] * that.entry[0][1] + tmp.entry[0][1] * that.entry[1][1];
	entry[1][0] = tmp.entry[1][0] * that.entry[0][0] + tmp.entry[1][1] * that.entry[1][0];
	entry[1][1] = tmp.entry[1][0] * that.entry[0][1] + tmp.entry[1][1] * that.entry[1][1];
	return (*this);
};

inline icMatrix2x2 &icMatrix2x2::setIdentity() {
  entry[0][0] = 1; entry[0][1] = 0; 
  entry[1][0] = 0; entry[1][1] = 1; 
  return (*this);
};

inline icMatrix2x2 operator+(const icMatrix2x2 &a,double b) {
  return (icMatrix2x2(a)+=b);
}

inline icMatrix2x2 operator-(const icMatrix2x2 &a,double b) {
  return (icMatrix2x2(a)-=b);
}

inline icMatrix2x2 operator*(const icMatrix2x2 &a,double b) {
  return (icMatrix2x2(a)*=b);
}
 
inline icMatrix2x2 operator+(double a, const icMatrix2x2 &b) {
return b+a;
}

inline icMatrix2x2 operator-(double a, const icMatrix2x2 &b) {
  return icMatrix2x2(a-b.entry[0][0],a-b.entry[0][1],
		     a-b.entry[1][0],a-b.entry[1][1]);
}

inline icMatrix2x2 operator*(double a, const icMatrix2x2 &b) {
  return b*a;
}
 
inline icMatrix2x2 operator+(const icMatrix2x2 &a,const icMatrix2x2 &b) {
  return (icMatrix2x2(a)+=b);
}
 
inline icMatrix2x2 operator-(const icMatrix2x2 &a,const icMatrix2x2 &b) {
  return (icMatrix2x2(a)-=b);
}

inline icMatrix2x2 operator*(const icMatrix2x2 &a,const icMatrix2x2 &b) {
  return (icMatrix2x2(a)*=b);
}

inline icMatrix2x2 multiply(const icMatrix2x2 &a,const icMatrix2x2 &b) {
  icMatrix2x2 tmp(a);
  tmp.rightMultiply(b);
  return tmp;
}

inline icVector2 operator*(const icMatrix2x2 &a,const icVector2 &b) {
  return icVector2(b.entry[0]*a.entry[0][0] + b.entry[1]*a.entry[0][1],
		   b.entry[0]*a.entry[1][0] + b.entry[1]*a.entry[1][1]);
}

inline icVector2 operator*(const icVector2 &a,const icMatrix2x2 &b) {
  return icVector2(a.entry[0]*b.entry[0][0] + a.entry[1]*b.entry[1][0],
		   a.entry[0]*b.entry[0][1] + a.entry[1]*b.entry[1][1]);
}

inline double determinant(const icMatrix2x2 &a) {
  return ( a.entry[0][0] * a.entry[1][1] - a.entry[0][1] * a.entry[1][0] );
}

inline icMatrix2x2 transpose(const icMatrix2x2 &a) {
  icMatrix2x2 tmp(a);

	tmp.entry[0][1] = a.entry[1][0];
	tmp.entry[1][0] = a.entry[0][1];
  return tmp;
}

inline icMatrix2x2 inverse(const icMatrix2x2 &a) {
	icMatrix2x2 tmp;
	double dmt;
	
	if ((dmt=determinant(a))!= 0.0) {
		tmp.entry[0][0] = a.entry[1][1]/dmt;
		tmp.entry[0][1] = -a.entry[0][1]/dmt;
		tmp.entry[1][0] = -a.entry[1][0]/dmt;
		tmp.entry[1][1] = a.entry[0][0]/dmt;
	}
	return tmp;
}

// start for class icMatrix3x3
class icMatrix3x3 {
public:
  inline icMatrix3x3();
  inline icMatrix3x3(double x);
  inline icMatrix3x3(const icMatrix3x3 &that);
	inline icMatrix3x3(const icVector3 &v1, const icVector3 &v2, const icVector3 &v3);

  inline icMatrix3x3(double M00, double M01, double M02,
										 double M10, double M11, double M12,
										 double M20, double M21, double M22);
  inline icMatrix3x3(double M[3][3]);

  inline icMatrix3x3 &set      (const double d);
  inline icMatrix3x3 &operator=(const double d);

  inline icMatrix3x3 &set      (const icMatrix3x3 &that);  
  inline icMatrix3x3 &operator=(const icMatrix3x3 &that); 

	inline icMatrix3x3 &set			 (double M[3][3]);
  inline icMatrix3x3 &operator=(double M[3][3]); 

	inline icMatrix3x3 &set     (const icVector3 &v1, const icVector3 &v2, const icVector3 &v3);
  inline icMatrix3x3 &set			(double M00, double M01, double M02,
					      							 double M10, double M11, double M12,
															 double M20, double M21, double M22);
  inline int operator!=(const icMatrix3x3 &that)const; 
  inline int operator==(const icMatrix3x3 &that)const; 

  inline int operator==(double d) const;
  inline int operator!=(double d) const;
  
  inline icMatrix3x3 &operator+=(double d);
  inline icMatrix3x3 &operator-=(double d);
  inline icMatrix3x3 &operator*=(double d);

  // component-wise operations.
  inline icMatrix3x3 &operator+=(const icMatrix3x3 &that);
  inline icMatrix3x3 &operator-=(const icMatrix3x3 &that);
  inline icMatrix3x3 &operator*=(const icMatrix3x3 &that);

  // Left : this = that x this  
  // Right: this = this x that
  icMatrix3x3 &leftMultiply (const icMatrix3x3 &that);
  icMatrix3x3 &rightMultiply(const icMatrix3x3 &that);

  inline icMatrix3x3 &setIdentity     ();

public:
  double entry[3][3];

};

inline icMatrix3x3 operator+(const icMatrix3x3 &a, double b);
inline icMatrix3x3 operator-(const icMatrix3x3 &a, double b);
inline icMatrix3x3 operator*(const icMatrix3x3 &a, double b);

inline icMatrix3x3 operator+(const icMatrix3x3 &a, const icMatrix3x3 &b);
inline icMatrix3x3 operator-(const icMatrix3x3 &a, const icMatrix3x3 &b);
inline icMatrix3x3 operator*(const icMatrix3x3 &a, const icMatrix3x3 &b); 

inline icMatrix3x3 multiply(const icMatrix3x3 &a, const icMatrix3x3 &b); 
inline icMatrix3x3 conjugate(const icMatrix3x3 &a, const icMatrix3x3 &b); 
inline icMatrix3x3 othoconjugate(const icMatrix3x3 &a, const icMatrix3x3 &b); 
inline icVector3   operator*(const icMatrix3x3 &a, const icVector3   &b);
inline icVector3   operator*(const icVector3   &a, const icMatrix3x3 &b);

inline double determinant(const icMatrix3x3 &a);

inline icMatrix3x3 transpose(const icMatrix3x3 &a);
inline icMatrix3x3   inverse(const icMatrix3x3 &a);

inline icMatrix3x3::icMatrix3x3() {
  entry[0][0] = 1;
  entry[0][1] = 0;
  entry[0][2] = 0;
  entry[1][0] = 0;
  entry[1][1] = 1;
  entry[1][2] = 0;
  entry[2][0] = 0;
  entry[2][1] = 0;
  entry[2][2] = 1;
}

inline icMatrix3x3::icMatrix3x3(double x) {
  entry[0][0] = x;
  entry[0][1] = x;
  entry[0][2] = x;
  entry[1][0] = x;
  entry[1][1] = x;
  entry[1][2] = x;
  entry[2][0] = x;
  entry[2][1] = x;
  entry[2][2] = x;
}

inline icMatrix3x3::icMatrix3x3(double M00, double M01, double M02,
																double M10, double M11, double M12,
																double M20, double M21, double M22) {
  entry[0][0] = M00;
  entry[0][1] = M01;
  entry[0][2] = M02;
  entry[1][0] = M10;
  entry[1][1] = M11;
  entry[1][2] = M12;
  entry[2][0] = M20;
  entry[2][1] = M21;
  entry[2][2] = M22;
};

inline icMatrix3x3::icMatrix3x3(const icMatrix3x3 &that) {
  entry[0][0] = that.entry[0][0];
  entry[0][1] = that.entry[0][1];
  entry[0][2] = that.entry[0][2];
  entry[1][0] = that.entry[1][0];
  entry[1][1] = that.entry[1][1];
  entry[1][2] = that.entry[1][2];
  entry[2][0] = that.entry[2][0];
  entry[2][1] = that.entry[2][1];
  entry[2][2] = that.entry[2][2];
};

inline icMatrix3x3::icMatrix3x3(const icVector3 &v1, const icVector3 &v2, const icVector3 &v3) {
	entry[0][0] = v1.entry[0];
	entry[0][1] = v1.entry[1];
	entry[0][2] = v1.entry[2];
	entry[1][0] = v2.entry[0];
	entry[1][1] = v2.entry[1];
	entry[1][2] = v2.entry[2];
	entry[2][0] = v3.entry[0];
	entry[2][1] = v3.entry[1];
	entry[2][2] = v3.entry[2];
}

inline icMatrix3x3 &icMatrix3x3::set(const double d) {
  return (*this)=d;
}

inline icMatrix3x3 &icMatrix3x3::operator=(const double d) {
  entry[0][0] = d;
  entry[0][1] = d;
  entry[0][2] = d;

  entry[1][0] = d;
  entry[1][1] = d;
  entry[1][2] = d;

  entry[2][0] = d;
  entry[2][1] = d;
  entry[2][2] = d;

  return (*this);
};

inline icMatrix3x3 &icMatrix3x3::set(const icMatrix3x3 &that) {
  return (*this)=that;
}

inline icMatrix3x3 &icMatrix3x3::operator=(const icMatrix3x3 &that) {
  entry[0][0] = that.entry[0][0];
  entry[0][1] = that.entry[0][1];
  entry[0][2] = that.entry[0][2];
  entry[1][0] = that.entry[1][0];
  entry[1][1] = that.entry[1][1];
  entry[1][2] = that.entry[1][2];
  entry[2][0] = that.entry[2][0];
  entry[2][1] = that.entry[2][1];
  entry[2][2] = that.entry[2][2];
  return (*this);
};

inline icMatrix3x3 &icMatrix3x3::set(double M[3][3]) {
  return (*this)=M;
}

inline icMatrix3x3 &icMatrix3x3::operator=(double M[3][3]) {
  entry[0][0] = M[0][0];
  entry[0][1] = M[0][1];
  entry[0][2] = M[0][2];

  entry[1][0] = M[1][0];
  entry[1][1] = M[1][1];
  entry[1][2] = M[1][2];

  entry[2][0] = M[2][0];
  entry[2][1] = M[2][1];
  entry[2][2] = M[2][2];
return (*this);
};

inline icMatrix3x3 &icMatrix3x3::set(const icVector3 &v1, const icVector3 &v2, const icVector3 &v3) {
	entry[0][0] = v1.entry[0];
	entry[0][1] = v1.entry[1];
	entry[0][2] = v1.entry[2];
	entry[1][0] = v2.entry[0];
	entry[1][1] = v2.entry[1];
	entry[1][2] = v2.entry[2];
	entry[2][0] = v3.entry[0];
	entry[2][1] = v3.entry[1];
	entry[2][2] = v3.entry[2];
	return (*this);
}

inline icMatrix3x3 &icMatrix3x3::set			(double M00, double M01, double M02,
				      							 double M10, double M11, double M12,
														 double M20, double M21, double M22)
{
	entry[0][0] = M00;
	entry[0][1] = M01;
	entry[0][2] = M02;
	entry[1][0] = M10;
	entry[1][1] = M11;
	entry[1][2] = M12;
	entry[2][0] = M20;
	entry[2][1] = M21;
	entry[2][2] = M22;
	return (*this);
}

inline int icMatrix3x3::operator==(double d) const {
  return  ( (entry[0][0] == d) && (entry[0][1] == d) && (entry[0][2] == d) &&
						(entry[1][0] == d) && (entry[1][1] == d) && (entry[1][2] == d) && 
						(entry[2][0] == d) && (entry[2][1] == d) && (entry[2][2] == d));
}

inline int icMatrix3x3::operator!=(double d) const {
  return  ( (entry[0][0] != d) || (entry[0][1] != d) || (entry[0][2] != d) ||
						(entry[1][0] != d) || (entry[1][1] != d) || (entry[1][2] != d) ||
						(entry[2][0] != d) || (entry[2][1] != d) || (entry[2][2] != d));
}
  
inline int icMatrix3x3::operator==(const icMatrix3x3 &that)const {
  return ( (entry[0][0] == that.entry[0][0]) && (entry[0][1] == that.entry[0][1]) && (entry[0][2] == that.entry[0][2]) &&
					 (entry[1][0] == that.entry[1][0]) && (entry[1][1] == that.entry[1][1]) && (entry[1][2] == that.entry[1][2]) &&
					 (entry[2][0] == that.entry[2][0]) && (entry[2][1] == that.entry[2][1]) && (entry[2][2] == that.entry[2][2]));
}

inline int icMatrix3x3::operator!=(const icMatrix3x3 &that)const {
  return ( (entry[0][0] != that.entry[0][0]) || (entry[0][1] != that.entry[0][1]) || (entry[0][2] != that.entry[0][2]) ||
					 (entry[1][0] != that.entry[1][0]) || (entry[1][1] != that.entry[1][1]) || (entry[1][2] != that.entry[1][2]) ||
					 (entry[2][0] != that.entry[2][0]) || (entry[2][1] != that.entry[2][1]) || (entry[2][2] != that.entry[2][2]));
}

inline icMatrix3x3 &icMatrix3x3::operator+=(double d) {
  entry[0][0] += d; entry[0][1] += d; entry[0][2] += d; 
  entry[1][0] += d; entry[1][1] += d; entry[1][2] += d; 
  entry[2][0] += d; entry[2][1] += d; entry[2][2] += d; 
  return (*this);
}

inline icMatrix3x3 &icMatrix3x3::operator-=(double d) {
  entry[0][0] -= d; entry[0][1] -= d; entry[0][2] -= d; 
  entry[1][0] -= d; entry[1][1] -= d; entry[1][2] -= d;
  entry[2][0] -= d; entry[2][1] -= d; entry[2][2] -= d;
  return (*this);
}

inline icMatrix3x3 &icMatrix3x3::operator*=(double d) {
  entry[0][0] *= d; entry[0][1] *= d; entry[0][2] *= d; 
  entry[1][0] *= d; entry[1][1] *= d; entry[1][2] *= d; 
  entry[2][0] *= d; entry[2][1] *= d; entry[2][2] *= d; 
  return (*this);
}

inline icMatrix3x3 &icMatrix3x3::operator+=(const icMatrix3x3 &that) {
  entry[0][0] += that.entry[0][0]; entry[0][1] += that.entry[0][1]; entry[0][2] += that.entry[0][2]; 
  entry[1][0] += that.entry[1][0]; entry[1][1] += that.entry[1][1]; entry[1][2] += that.entry[1][2]; 
  entry[2][0] += that.entry[2][0]; entry[2][1] += that.entry[2][1]; entry[2][2] += that.entry[2][2]; 
  return (*this);
}
  
inline icMatrix3x3 &icMatrix3x3::operator-=(const icMatrix3x3 &that) {
  entry[0][0] -= that.entry[0][0]; entry[0][1] -= that.entry[0][1]; entry[0][2] -= that.entry[0][2]; 
  entry[1][0] -= that.entry[1][0]; entry[1][1] -= that.entry[1][1]; entry[1][2] -= that.entry[1][2]; 
  entry[2][0] -= that.entry[2][0]; entry[2][1] -= that.entry[2][1]; entry[2][2] -= that.entry[2][2]; 
  return (*this);
}

inline icMatrix3x3 &icMatrix3x3::operator*=(const icMatrix3x3 &that) {
  entry[0][0] *= that.entry[0][0]; entry[0][1] *= that.entry[0][1]; entry[0][2] *= that.entry[0][2]; 
  entry[1][0] *= that.entry[1][0]; entry[1][1] *= that.entry[1][1]; entry[1][2] *= that.entry[1][2]; 
  entry[2][0] *= that.entry[2][0]; entry[2][1] *= that.entry[2][1]; entry[2][2] *= that.entry[2][2]; 
  return (*this);
}

inline icMatrix3x3 &icMatrix3x3::leftMultiply (const icMatrix3x3 &that){
	icMatrix3x3 tmp(entry[0][0], entry[0][1], entry[0][2], 
									entry[1][0], entry[1][1], entry[1][2],
									entry[2][0], entry[2][1], entry[2][2]);
	
	entry[0][0] = that.entry[0][0] * tmp.entry[0][0] + that.entry[0][1] * tmp.entry[1][0] + that.entry[0][2] * tmp.entry[2][0];
	entry[0][1] = that.entry[0][0] * tmp.entry[0][1] + that.entry[0][1] * tmp.entry[1][1] + that.entry[0][2] * tmp.entry[2][1];
	entry[0][2] = that.entry[0][0] * tmp.entry[0][2] + that.entry[0][1] * tmp.entry[1][2] + that.entry[0][2] * tmp.entry[2][2];

	entry[1][0] = that.entry[1][0] * tmp.entry[0][0] + that.entry[1][1] * tmp.entry[1][0] + that.entry[1][2] * tmp.entry[2][0];
	entry[1][1] = that.entry[1][0] * tmp.entry[0][1] + that.entry[1][1] * tmp.entry[1][1] + that.entry[1][2] * tmp.entry[2][1];
	entry[1][2] = that.entry[1][0] * tmp.entry[0][2] + that.entry[1][1] * tmp.entry[1][2] + that.entry[1][2] * tmp.entry[2][2];

	entry[2][0] = that.entry[2][0] * tmp.entry[0][0] + that.entry[2][1] * tmp.entry[1][0] + that.entry[2][2] * tmp.entry[2][0];
	entry[2][1] = that.entry[2][0] * tmp.entry[0][1] + that.entry[2][1] * tmp.entry[1][1] + that.entry[2][2] * tmp.entry[2][1];
	entry[2][2] = that.entry[2][0] * tmp.entry[0][2] + that.entry[2][1] * tmp.entry[1][2] + that.entry[2][2] * tmp.entry[2][2];
	return (*this);
};

inline icMatrix3x3 &icMatrix3x3::rightMultiply(const icMatrix3x3 &that){
	icMatrix3x3 tmp(entry[0][0], entry[0][1], entry[0][2], 
									entry[1][0], entry[1][1], entry[1][2],
									entry[2][0], entry[2][1], entry[2][2]);

	entry[0][0] = tmp.entry[0][0] * that.entry[0][0] + tmp.entry[0][1] * that.entry[1][0] + tmp.entry[0][2] * that.entry[2][0];
	entry[0][1] = tmp.entry[0][0] * that.entry[0][1] + tmp.entry[0][1] * that.entry[1][1] + tmp.entry[0][2] * that.entry[2][1];
	entry[0][2] = tmp.entry[0][0] * that.entry[0][2] + tmp.entry[0][1] * that.entry[1][2] + tmp.entry[0][2] * that.entry[2][2];

	entry[1][0] = tmp.entry[1][0] * that.entry[0][0] + tmp.entry[1][1] * that.entry[1][0] + tmp.entry[1][2] * that.entry[2][0];
	entry[1][1] = tmp.entry[1][0] * that.entry[0][1] + tmp.entry[1][1] * that.entry[1][1] + tmp.entry[1][2] * that.entry[2][1];
	entry[1][2] = tmp.entry[1][0] * that.entry[0][2] + tmp.entry[1][1] * that.entry[1][2] + tmp.entry[1][2] * that.entry[2][2];

	entry[2][0] = tmp.entry[2][0] * that.entry[0][0] + tmp.entry[2][1] * that.entry[1][0] + tmp.entry[2][2] * that.entry[2][0];
	entry[2][1] = tmp.entry[2][0] * that.entry[0][1] + tmp.entry[2][1] * that.entry[1][1] + tmp.entry[2][2] * that.entry[2][1];
	entry[2][2] = tmp.entry[2][0] * that.entry[0][2] + tmp.entry[2][1] * that.entry[1][2] + tmp.entry[2][2] * that.entry[2][2];
	return (*this);
};

inline icMatrix3x3 &icMatrix3x3::setIdentity() {
  entry[0][0] = 1; entry[0][1] = 0; entry[0][2] = 0; 
  entry[1][0] = 0; entry[1][1] = 1; entry[1][2] = 0; 
  entry[2][0] = 0; entry[2][1] = 0; entry[2][2] = 1; 
  return (*this);
};

inline icMatrix3x3 operator+(const icMatrix3x3 &a,double b) {
  return (icMatrix3x3(a)+=b);
}

inline icMatrix3x3 operator-(const icMatrix3x3 &a,double b) {
  return (icMatrix3x3(a)-=b);
}

inline icMatrix3x3 operator*(const icMatrix3x3 &a,double b) {
  return (icMatrix3x3(a)*=b);
}
 
inline icMatrix3x3 operator+(double a, const icMatrix3x3 &b) {
return b+a;
}

inline icMatrix3x3 operator-(double a, const icMatrix3x3 &b) {
  return icMatrix3x3(a-b.entry[0][0],a-b.entry[0][1],a-b.entry[0][2],
										 a-b.entry[1][0],a-b.entry[1][1],a-b.entry[1][2],
										 a-b.entry[2][0],a-b.entry[2][1],a-b.entry[2][2]);
}

inline icMatrix3x3 operator*(double a, const icMatrix3x3 &b) {
  return b*a;
}
 
inline icMatrix3x3 operator+(const icMatrix3x3 &a,const icMatrix3x3 &b) {
  return (icMatrix3x3(a)+=b);
}
 
inline icMatrix3x3 operator-(const icMatrix3x3 &a,const icMatrix3x3 &b) {
  return (icMatrix3x3(a)-=b);
}

inline icMatrix3x3 operator*(const icMatrix3x3 &a,const icMatrix3x3 &b) {
  return (icMatrix3x3(a)*=b);
}

inline icMatrix3x3 multiply(const icMatrix3x3 &a,const icMatrix3x3 &b) {
  icMatrix3x3 tmp(a);
  tmp.rightMultiply(b);
  return tmp;
}

inline icMatrix3x3 conjugate(const icMatrix3x3 a, const icMatrix3x3 &b) {
  icMatrix3x3 tmp(a);
	icMatrix3x3 c = inverse(b);
  tmp.rightMultiply(b);
	tmp.leftMultiply(c);
  return tmp;
}

inline icMatrix3x3 othoconjugate(const icMatrix3x3 a, const icMatrix3x3 &b) {
  icMatrix3x3 tmp(a);
	icMatrix3x3 c = transpose(b);
  tmp.rightMultiply(b);
	tmp.leftMultiply(c);
  return tmp;
}

inline icVector3 operator*(const icMatrix3x3 &a,const icVector3 &b) {
  return icVector3(b.entry[0]*a.entry[0][0] + b.entry[1]*a.entry[0][1] + b.entry[2]*a.entry[0][2], 
									 b.entry[0]*a.entry[1][0] + b.entry[1]*a.entry[1][1] + b.entry[2]*a.entry[1][2],
									 b.entry[0]*a.entry[2][0] + b.entry[1]*a.entry[2][1] + b.entry[2]*a.entry[2][2]);
}

inline icVector3 operator*(const icVector3 &a,const icMatrix3x3 &b) {
  return icVector3(a.entry[0]*b.entry[0][0] + a.entry[1]*b.entry[1][0] + a.entry[2]*b.entry[2][0],
									 a.entry[0]*b.entry[0][1] + a.entry[1]*b.entry[1][1] + a.entry[2]*b.entry[2][1],
									 a.entry[0]*b.entry[0][2] + a.entry[1]*b.entry[1][2] + a.entry[2]*b.entry[2][2]);
}

inline double determinant(const icMatrix3x3 &a) {
  return ( a.entry[0][0] * a.entry[1][1] * a.entry[2][2] - a.entry[2][0] * a.entry[1][1] * a.entry[0][2]
		     + a.entry[1][0] * a.entry[2][1] * a.entry[0][2] - a.entry[0][0] * a.entry[2][1] * a.entry[1][2]
				 + a.entry[2][0] * a.entry[0][1] * a.entry[1][2] - a.entry[1][0] * a.entry[0][1] * a.entry[2][2]);
}

inline icMatrix3x3 transpose(const icMatrix3x3 &a) {
  icMatrix3x3 tmp(a);

	tmp.entry[0][1] = a.entry[1][0];
	tmp.entry[1][0] = a.entry[0][1];

	tmp.entry[0][2] = a.entry[2][0];
	tmp.entry[2][0] = a.entry[0][2];

	tmp.entry[2][1] = a.entry[1][2];
	tmp.entry[1][2] = a.entry[2][1];
  return tmp;
}

inline icMatrix3x3 inverse(const icMatrix3x3 &a) {
	icMatrix3x3 tmp;
	double dmt;
	
	if ((dmt=determinant(a))!= 0.0) {
		tmp.entry[0][0] = (a.entry[1][1] * a.entry[2][2] - a.entry[2][1] * a.entry[1][2])/dmt;
		tmp.entry[0][1] = (a.entry[2][1] * a.entry[0][2] - a.entry[0][1] * a.entry[2][2])/dmt;
		tmp.entry[0][2] = (a.entry[0][1] * a.entry[1][2] - a.entry[1][1] * a.entry[0][2])/dmt;

		tmp.entry[1][0] = (a.entry[1][2] * a.entry[2][0] - a.entry[2][2] * a.entry[1][0])/dmt;
		tmp.entry[1][1] = (a.entry[2][2] * a.entry[0][0] - a.entry[0][2] * a.entry[2][0])/dmt;
		tmp.entry[1][2] = (a.entry[0][2] * a.entry[1][0] - a.entry[1][2] * a.entry[0][0])/dmt;

		tmp.entry[2][0] = (a.entry[1][0] * a.entry[2][1] - a.entry[2][0] * a.entry[1][1])/dmt;
		tmp.entry[2][1] = (a.entry[2][0] * a.entry[0][1] - a.entry[0][0] * a.entry[2][1])/dmt;
		tmp.entry[2][2] = (a.entry[0][0] * a.entry[1][1] - a.entry[1][0] * a.entry[0][1])/dmt;
	}
	return tmp;
}

#endif
