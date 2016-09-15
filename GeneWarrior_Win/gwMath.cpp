#include "stdafx.h"
#include "gwMath.h"
#include "math.h"
#include <stdio.h>
#include <string.h>

Vec3d::Vec3d()
{

}

Vec3d::Vec3d(double x,double y, double z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

Vec3d Vec3d::operator=(const Vec3d& other)
{
	memcpy(this,&other,sizeof(Vec3d));
	return *this;
}

Vec3d::~Vec3d()
{
}

bool Vec3d::operator==(const Vec3d& other)const
{
	if (x != other.x)
		return false;

	if (y != other.y)
		return false;

	if (z != other.z)
		return false;

	return true;
}

double& Vec3d::operator()(const unsigned int e)
{
	return data[e];
}
double sign(double x)
{
    //Returns 1.0 if x is positive, -1.0 if x is negative or zero.
    if (x > 0.0) 
		return 1.0;
	else 
		return -1.0;
}

double len3(Vec3d& v)
{
    //Returns the length of 3-vector v.
    //return sqrt(v(0)^2 + v(1)**2 + v(2)**2);

	return sqrt( (v(0)*v(0)) + (v(1)*v(1)) + (v(2)*v(2)) );
}

Vec3d neg3(Vec3d& v)
{
    //Returns the negation of 3-vector v.
    return Vec3d(-v(0), -v(1), -v(2));
}

Vec3d add3(Vec3d& a, Vec3d& b)
{
	//Returns the sum of 3-vectors a and b.
	return Vec3d(a(0) + b(0), a(1) + b(1), a(2) + b(2));
}

Vec3d sub3(Vec3d& a, Vec3d& b)
{
    //Returns the difference between 3-vectors a and b.
    return Vec3d(a(0) - b(0), a(1) - b(1), a(2) - b(2));
}

Vec3d mul3(Vec3d& v,double s)
{
    //Returns 3-vector v multiplied by scalar s.
    return Vec3d(v(0) * s, v(1) * s, v(2) * s);
}

Vec3d div3(Vec3d& v, double s)
{
    //Returns 3-vector v divided by scalar s.
    return Vec3d(v(0) / s, v(1) / s, v(2) / s);
}

double dist3(Vec3d& a, Vec3d& b)
{
    //Returns the distance between point 3-vectors a and b.
    return len3(sub3(a, b));
}

Vec3d norm3(Vec3d& v)
{
    //Normalize
    double l = len3(v);
    if (l > 0.0) 
		return Vec3d(v(0) / l, v(1) / l, v(2) / l);
    else  // if it's zero length vector, normal is meaningless
		return Vec3d(0.0, 0.0, 0.0);
}

double dot3(Vec3d& a,Vec3d&  b)
{
    //Returns the dot product of 3-vectors a and b.
    return (a(0) * b(0)) + (a(1) * b(1)) + (a(2) * b(2));
}

Vec3d  cross(Vec3d& a,Vec3d&  b)
{
    //Returns the cross product of 3-vectors a and b.
    return Vec3d(a(1) * b(2) - a(2) * b(1), a(2) * b(0) - a(0) * b(2),
        a(0) * b(1) - a(1) * b(0));
}

Vec3d project3(Vec3d& v,Vec3d& d)
{
    //Returns projection of 3-vector v onto unit 3-vector d.
    return mul3(v, dot3(norm3(v), d));
}

double acosdot3(Vec3d& a,Vec3d& b)
{
    //Returns the angle between unit 3-vectors a and b.
    double x = dot3(a, b);
    if (x < -1.0 )
		return GW_PI;
	else if (x > 1.0 )
		return 0.0;
    else 
		return acos(x);
}


Mat4x4::Mat4x4(
		double a,double  b,double  c,double  d,
		double e,double  f,double  g,double  h,
		double i,double  j,double  k,double  l,
		double m,double  n,double  o,double  p)
{
	data[0] = a;
	data[1] = b;
	data[2] = c;
	data[3] = d;
	data[4] = e;
	data[5] = f;
	data[6] = g;
	data[7] = h;
	data[8] = i;
	data[9] = j;
	data[10] = k;
	data[11] = l;
	data[12] = m;
	data[13] = n;
	data[14] = o;
	data[15] = p;
}

Mat4x4::~Mat4x4()
{
}

Mat4x4 Mat4x4::operator=(const Mat4x4& other)
{
	memcpy(data,other.data,sizeof(Mat4x4));
	return *this;
}

double& Mat4x4::operator()(const unsigned int e)
{
	return data[e];
}


Mat3x3::Mat3x3()
{

}

Mat3x3::Mat3x3(
		double a,double  b,double  c,
		double d,double  e,double  f,
		double g,double  h,double  i)
{
	data[0] = a;
	data[1] = b;
	data[2] = c;
	data[3] = d;
	data[4] = e;
	data[5] = f;
	data[6] = g;
	data[7] = h;
	data[8] = i;
}

Mat3x3::~Mat3x3()
{
}

Mat3x3 Mat3x3::operator=(const Mat3x3& other)
{
	memcpy(data,other.data,sizeof(Mat3x3));
	return *this;
}

double& Mat3x3::operator()(const unsigned int e)
{
	return data[e];
}

Vec3d rotate3(Mat3x3& m,Vec3d& v)
{
    //Returns the rotation of 3-vector v by 3x3 (row major) matrix m.
    return Vec3d(v(0) * m(0) + v(1) * m(1) + v(2) * m(2),
        v(0) * m(3) + v(1) * m(4) + v(2) * m(5),
        v(0) * m(6) + v(1) * m(7) + v(2) * m(8));
}

Mat3x3 invert3x3(Mat3x3& m)
{
    //Returns the inversion (transpose) of 3x3 rotation matrix m.
    return Mat3x3(m(0), m(3), m(6), m(1), m(4), m(7), m(2), m(5), m(8));
}

Vec3d zaxis(Mat3x3& m)
{
    //Returns the z-axis vector from 3x3 (row major) rotation matrix m.
    return Vec3d(m(2), m(5), m(8));
}

Mat3x3 calcRotMatrix(Vec3d axis,double angle)
{
    //Returns the row-major 3x3 rotation matrix defining a rotation around axis by
    //angle.
    
    double cosTheta = cos(angle);
    double sinTheta = sin(angle);
    double t = 1.0 - cosTheta;
    return Mat3x3(
        t * axis(0)*axis(0) + cosTheta,
        t * axis(0) * axis(1) - sinTheta * axis(2),
        t * axis(0) * axis(2) + sinTheta * axis(1),
        t * axis(0) * axis(1) + sinTheta * axis(2),
        t * axis(1)*axis(1) + cosTheta,
        t * axis(1) * axis(2) - sinTheta * axis(0),
        t * axis(0) * axis(2) - sinTheta * axis(1),
        t * axis(1) * axis(2) + sinTheta * axis(0),
        t * axis(2)*axis(2) + cosTheta);
}

Mat4x4 makeOpenGLMatrix(const double* r,const double* p)
{
    //Returns an OpenGL compatible (column-major, 4x4 homogeneous) transformation
    //matrix from ODE compatible (row-major, 3x3) rotation matrix r and position
    //vector p.

	return Mat4x4(
	  r[0], r[1], r[2], r[3],
	  r[4], r[5], r[6], r[7],
	  r[8], r[9], r[10], r[11],
      p[0],p[1],p[2],1);
}



Vec3d getBodyRelVec(Mat3x3 r, Vec3d& v)
{
    
    //Returns the 3-vector v transformed into the local coordinate system of ODE
    //body b.

    //return rotate3(invert3x3(b.getRotation()), v);
	return rotate3(invert3x3(r), v);
}
