#ifndef featureExtraction_H
#define featureExtraction_H

#include <emc/data.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

struct Point2d {
    double x, y;
    Point2d(double x_, double y_) : x(x_), y(y_) {};
};

struct Line2d {
    Point2d p1, p2;
    Line2d(Point2d p1_, Point2d p2_) : p1(p1_), p2(p2_) {};
};

class FeatureExtraction
{
private:
    std::vector<Point2d> filteredCoords_;
    std::vector<int> splitIndices_;
    std::vector<Line2d> lines_;
    std::vector<Point2d> corners_;

public:
    FeatureExtraction() {}

    // Method to pre-process the LRF data
    void LRF2Points(emc::LaserData* laser);

    // Method that returns the distance between a point and a line
    double distPointLine(double px, double py, double x1, double y1, double x2, double y2);

    // Method to split a data segment, if possible
    void splitSegment(int i_start, int i_end);

    // Visualization
    void plotFeatures();

    // Call this method in your main file to execute feature extraction
    void findLinesAndCorners(emc::LaserData* laser);

    // Method that returns lines (e.g. walls)
    std::vector<Line2d> getLines();

    // Method that returns corners
    std::vector<Point2d> getCorners();
};

#endif //featureExtraction_H