#ifndef TYPEDEFS_H
#define TYPEDEFS_H



struct point3D
{
    double x,y,z;
};

struct approachVector
{
    double theta,psi;
};
struct machineCartesianCoordinates
{
    point3D p1,p1pr,p2,p2pr,p3,p4,p5;
};

struct machineCoordinates
{
    double fi1,fi2,fi3,fi4,fi5;
    machineCartesianCoordinates cartesian;
};

struct robotParamsRegional
{
    double l1,l2,l3,d,e;
};

struct robotParamsLocal
{
    double l4,l5,l6;
};

struct Deltas
{
    int d1,d2,d5;
};

#endif // TYPEDEFS_H
