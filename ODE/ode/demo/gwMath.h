// Based on Python Code made by Matt Heinzen

#pragma once

#ifndef GW_MATH_H
#define GW_MATH_H

#define GW_PI 3.1415926535897931


struct Vec3d
{

	Vec3d();
	Vec3d(double x,double y, double z);
	~Vec3d();	

	double& operator()(const unsigned int e);
	Vec3d operator=(const Vec3d& other);
	bool operator==(const Vec3d& other)const;

	union 
	{
		double data[3]; // 3 components - x,y,z

		struct
		{
			double x;
			double y;
			double z;
		};
	};
};


double sign(double x);
double len3(Vec3d& v);
Vec3d neg3(Vec3d& v);
Vec3d add3(Vec3d& a, Vec3d& b);
Vec3d sub3(Vec3d& a, Vec3d& b);
Vec3d mul3(Vec3d& v,double s);
Vec3d div3(Vec3d& v, double s);
double dist3(Vec3d& a, Vec3d& b);
Vec3d norm3(Vec3d& v);
double dot3(Vec3d& a,Vec3d&  b);
Vec3d  cross(Vec3d& a,Vec3d&  b);
Vec3d project3(Vec3d& v,Vec3d& d);
double acosdot3(Vec3d& a,Vec3d& b);

class Mat4x4
{
public:
	Mat4x4(
		double a,double  b,double  c,double  d,
		double e,double  f,double  g,double  h,
		double i,double  j,double  k,double  l,
		double m,double  n,double  o,double  p);
	

	~Mat4x4();

	double& operator()(const unsigned int e);
	Mat4x4 operator=(const Mat4x4& other);

	double data[16];
};

class Mat3x3
{
public:
	Mat3x3();
	Mat3x3(
		double a,double  b,double  c,
		double d,double  e,double  f,
		double g,double  h,double  i);

	~Mat3x3();

	double& operator()(const unsigned int e);
	Mat3x3 operator=(const Mat3x3& other);

	double data[9];
};

Vec3d rotate3(Mat3x3& m,Vec3d& v);
Mat3x3 invert3x3(Mat3x3& m);
Vec3d zaxis(Mat3x3& m);
Mat3x3 calcRotMatrix(Vec3d axis,double angle);
Mat4x4 makeOpenGLMatrix(Mat3x3& r,Vec3d& p);



Vec3d getBodyRelVec(Mat3x3 r, Vec3d& v);



#endif