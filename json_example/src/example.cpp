#include "map_visualization.h"
#include "geometric_primitives.h"

int main()
{

    // Read the json object
    std::ifstream i("../src/map.json");
    nlohmann::json doc = nlohmann::json::parse(i);

    // Make a vector of points and a vector of lines to hold the points, walls and cabinets (see geometric_primitives.h)
    std::vector<Point> points;
    std::vector<Line> walls;
    typedef std::vector<Line> Cabinet;
    std::vector<Cabinet> cabinets;

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

    // Visualize the environment
    visualization vis("vector map of the hospital",-1,4,-1,4); // visualization object with title and plot dimensions in m
    vis.set_points(points); // Set all the points that are referenced by the walls and cabinets
    vis.plot_lines(walls,cv::Scalar(255,100,0)); // Draw all the lines that make up the walls

    for(auto& cabinet : cabinets){
        vis.plot_lines(cabinet, cv::Scalar(255, 255, 255)); // Draw all the lines that make up the cabinets
    }

    vis.show_always();

    return 0;
}

