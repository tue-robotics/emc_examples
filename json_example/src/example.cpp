#include "map_visualization.h"
#include "geometric_primitives.h"

#include <algorithm>

int main()
{

    // Read the json object
    std::ifstream i("../src/map.json");
    nlohmann::json doc = nlohmann::json::parse(i);

    // Make a vector of points and a vector of lines to hold the points, walls and cabinets (see geometric_primitives.h)
    std::vector<Point> points;
    std::vector<Line> walls;

    typedef std::vector<Line> Cabinet;
    typedef std::vector<Line> Door;
    typedef std::vector<Line> Area;

    std::vector<Cabinet> cabinets;
    std::vector<Door> doors;
    std::vector<Area> startArea;

    // Read the json objects into the c++ object (NOTE: The _comment fields are not used, they are for convenience)
    for (const auto& p : doc.at("points") )
    {
        assert(p.is_object());
        assert(p.find("x") != p.end());  // check for key x
        assert(p.find("y") != p.end());  // check for key y
        points.push_back(Point(p["x"], p["y"]));
    }

    for (const auto& l : doc.at("walls") )
    {
        walls.push_back(Line(l[0], l[1]));
    }

    for (const auto& cab : doc.at("cabinets") )
    {
        Cabinet cabinet;
        for (const auto& l : cab){
            cabinet.push_back(Line(l[0], l[1]));
        }
        cabinets.push_back(cabinet);
    }

    for (const auto& dr : doc.at("doors") )
    {
        Door door;
        for (const auto& l : dr){
            door.push_back(Line(l[0], l[1]));
        }
        doors.push_back(door);
    }

     for (const auto& sa : doc.at("start_area") )
    {
        Area area;
        for (const auto& l : sa){
            area.push_back(Line(l[0], l[1]));
        }
        startArea.push_back(area);
    }

    // Find the min/max x/y values
    auto compx = [](Point &p1, Point &p2){return p1.x < p2.x;};
    double xmin = std::min_element(points.begin(), points.end() ,compx)->x;
    double xmax = std::max_element(points.begin(), points.end() ,compx)->x;

    auto compy = [](Point &p1, Point &p2){return p1.y < p2.y;};
    double ymin = std::min_element(points.begin(), points.end() ,compy)->y;
    double ymax = std::max_element(points.begin(), points.end() ,compy)->y;

    // Visualize the environment
    visualization vis("vector map of the hospital",xmin,xmax,ymin,ymax); // visualization object with title and plot dimensions in m
    vis.set_points(points); // Set all the points that are referenced by the walls and cabinets
    vis.plot_lines(walls,cv::Scalar(255,100,0)); // Draw all the lines that make up the walls

    for(auto& area : startArea){
        vis.plot_lines(area, cv::Scalar(0, 255, 255)); // Draw all the lines that make up the start area
    }

    for(auto& cabinet : cabinets){
        vis.plot_lines(cabinet, cv::Scalar(255, 255, 255)); // Draw all the lines that make up the cabinets
    }

    for(auto& door : doors){
        vis.plot_lines(door, cv::Scalar(255, 0, 255)); // Draw all the lines that make up the doors
    }

    vis.show_always();

    return 0;
}

