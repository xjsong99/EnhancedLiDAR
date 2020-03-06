#pragma once

#include <cmath>

const double eps = 1e-8;
inline int sgn(double x)
{
    if(fabs(x) < eps) return 0;
    if(x < 0) return -1;
    else return 1;
}

struct Point3{
    double x, y, z;
    Point3 (double _x = 0,double _y = 0,double _z = 0)
    {
        x = _x;
        y = _y;
        z = _z;
    }
    Point3 operator -(const Point3 &b)const
    {
        return Point3(x - b.x, y - b.y, z - b.z);
    }
    Point3 operator +(const Point3 &b)const
    {
        return Point3(x + b.x, y + b.y, z + b.z);
    }
    Point3 operator *(const double &k)const
    {
        return Point3(k * x, k * y, k * z);
    }
    Point3 operator /(const double &k)const
    {
        return Point3(x / k, y / k, z / k);
    }
    //点乘
    double operator *(const Point3 &b)const
    {
        return x * b.x + y * b.y + z * b.z;
    }
    //叉乘
    Point3 operator ^(const Point3 &b)const
    {
        return Point3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
    }
};

struct Line3
{
    Point3 s, e;//线段的起终点
    Line3(){}
    Line3(Point3 _s,Point3 _e)
    {
        s = _s;
        e = _e;
    }
};

struct Plane
{
    Point3 a, b, c, o;//平面上3个点及法向量
    Plane(){}
    Plane(Point3 _a,Point3 _b,Point3 _c)
    {
        a = _a;
        b = _b;
        c = _c;
        o = pvec();
    }
    Point3 pvec()
    {
        return (b-a)^(c-a);
    }
    //求平面和直线的交点，返回值是交点个数
    int crossline(Line3 u,Point3 &p)
    {
        double x = o * (u.e - a);
        double y = o * (u.s - a);
        double d = x - y;
        if(sgn(d) == 0) return 0;
        p = ((u.s * x) - (u.e * y)) / d;
        return 1;
    }
    //判断点在三角形内部
    bool PointInTriangle(Point3 p)
    {
        Point3 v0 = c - a;
        Point3 v1 = b - a;
        Point3 v2 = p - a;

        float dot00 = v0 * v0;
        float dot01 = v0 * v1;
        float dot02 = v0 * v2;
        float dot11 = v1 * v1;
        float dot12 = v1 * v2;

        float inverDeno = dot00 * dot11 - dot01 * dot01;
        
        if(sgn(inverDeno) == 0)
            return false;

        float x = dot11 * dot02 - dot01 * dot12;
        float y = dot00 * dot12 - dot01 * dot02;

        return sgn(x) >= 0 && sgn(y) >= 0 && sgn(x + y - inverDeno) <= 0;

        /*
        float u = (dot11 * dot02 - dot01 * dot12) / inverDeno;
        if(u < 0 || u > 1) return false;

        float v = (dot00 * dot12 - dot01 * dot02) / inverDeno ;
        if(v < 0 || v > 1) return false;

        return sgn(u + v - 1) <= 0;
        */

    }
};