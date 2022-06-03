
#ifndef MY_PROJECT_GEOMETRIC_PRIMITIVES_H
#define MY_PROJECT_GEOMETRIC_PRIMITIVES_H

struct Point {
    double x;
    double y;
    Point(double x_, double y_ ): x(x_), y(y_)
    {
    }
};

struct Line {
    int p1;
    int p2;
    Line(int p1_, int p2_ ): p1(p1_), p2(p2_)
    {
    }
};

#endif //MY_PROJECT_GEOMETRIC_PRIMITIVES_H
