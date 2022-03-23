#include "featureExtraction.h" 
#include "config.h"

void FeatureExtraction::LRF2Points(emc::LaserData* laser)
{
    double theta;

    // For all laser measurements
	for(int i = 0; i < laser->ranges.size(); ++i) {
        // Check if the measurement is valid
        if(laser->ranges[i] > laser->range_min && laser->ranges[i] < laser->range_max) {
            // Calculate the angle of the measurement
            theta = laser->angle_min + i * laser->angle_increment;
            // Convert to points in local robot frame
            filteredCoords_.push_back(Point2d(laser->ranges[i] * cos(theta), 
                                              laser->ranges[i] * sin(theta)));
        }
    }
}

double FeatureExtraction::distPointLine(double px, double py, double x1, double y1, double x2, double y2) {
    return std::abs( (y2-y1)*px - (x2-x1)*py + x2*y1 - y2*x1 ) / std::sqrt( std::pow(y2-y1,2) + std::pow(x2-x1,2) );
}

void FeatureExtraction::splitSegment(int i_start, int i_end)
{
    double dist = -1;
    double distMax = -1;
    double splitIndex = -1;

    // For each point in the data segment,
    for(int i = i_start; i < i_end; ++i) {
        // calculate the distance to a line through the start + end point.
        dist = distPointLine(filteredCoords_[i].x, filteredCoords_[i].y,
                             filteredCoords_[i_start].x, filteredCoords_[i_start].y,
                             filteredCoords_[i_end].x, filteredCoords_[i_end].y);
        // Update the maximum of the distances
        if(dist > distMax) {
            distMax = dist;
            splitIndex = i;
        }
    }

    // If the segment is splittable,
    if(distMax > SEGMENT_DIST_THRESHOLD) {
        // save the split index,
        splitIndices_.push_back(splitIndex);
        // and recursively try to split the new segments again.
        splitSegment(i_start, splitIndex);
        splitSegment(splitIndex, i_end);
    }
}

void FeatureExtraction::plotFeatures()
{
    const double pix_per_m = 100;
    const double range_min = -2.5;
    const double range_max = 2.5;

    cv::Mat image = cv::Mat::zeros( (range_max - range_min) * pix_per_m, 
                                    (range_max - range_min) * pix_per_m, CV_8UC3 );

    // Laser points
    for(auto point : filteredCoords_) {
        cv::circle(image, cv::Point( pix_per_m*(point.y-range_min), pix_per_m*(point.x-range_min)),
                                     2, cv::Scalar(150,100,0), 2, 8 );
    }

    // Corners
    for(auto point : corners_) {
        cv::circle(image, cv::Point( pix_per_m*(point.y-range_min), pix_per_m*(point.x-range_min)),
                                     6, cv::Scalar(0,0,255), 2, 8 );
    }

    // Lines
    for(auto line : lines_) {
        std::vector<cv::Point> p;
        cv::Point point1(std::floor(pix_per_m*(line.p1.y-range_min)),
                         std::floor(pix_per_m*(line.p1.x-range_min)));
        p.push_back(point1);
        cv::Point point2(std::floor(pix_per_m*(line.p2.y-range_min)),
                         std::floor(pix_per_m*(line.p2.x-range_min)));
        p.push_back(point2);
        cv::polylines(image, p, true, cv::Scalar(100,255,0), 2);
    }

    cv::Mat image_corrected;
    cv::flip(image, image_corrected, -1);
    cv::imshow("visualization", image_corrected);
    cv::waitKey(1);
}

void FeatureExtraction::findLinesAndCorners(emc::LaserData* laser)
{
    // Convert LRF data to coordinates in local robot frame
    filteredCoords_.clear();
    LRF2Points(laser);

    // Data segmentation
    splitIndices_.clear();
    splitIndices_.push_back(0);
    splitIndices_.push_back(filteredCoords_.size()-1);
    splitSegment(0, filteredCoords_.size()-1);

    // Sort the split indices
    std::sort(splitIndices_.begin(), splitIndices_.end());

    // Return resulting lines and corners
    lines_.clear();
    corners_.clear();
    for(int i = 0; i < splitIndices_.size()-1; ++i) {
        // If the segment has enough data points to be considered reliable,
        if(splitIndices_[i+1] - splitIndices_[i] > SEGMENT_MIN_NR_POINTS) {
            // save line,
            lines_.push_back(Line2d(filteredCoords_[splitIndices_[i]],
                                    filteredCoords_[splitIndices_[i+1]]));
            // and (second) end point.
            corners_.push_back(filteredCoords_[splitIndices_[i+1]]);
        }
    }

    // Plotting
    if(PLOT_FEATURES == 1) {
        plotFeatures();
    }
}

std::vector<Line2d> FeatureExtraction::getLines()
{
    return lines_;
}

std::vector<Point2d> FeatureExtraction::getCorners() 
{
    return corners_;
}