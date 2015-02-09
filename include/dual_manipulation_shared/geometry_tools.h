#ifndef GEOMETRY_TOOLS_H
#define GEOMETRY_TOOLS_H
#include <vector>
#include <algorithm>

struct Point
{
    Point(double x,double y)
    {this->x=x;this->y=y;}
    double x,y;
    bool operator <(const Point&  p) const;
};


class geometry_tools
{
public:
    inline void order_points(std::vector<Point>& polygon)
    {
        std::sort(polygon.begin(), polygon.end());
    }
    
    inline bool point_in_ordered_polygon(Point p, const std::vector<Point>& polygon)
    {
        return point_in_ordered_polygon(p.x,p.y,polygon);
    }
    
    bool point_in_ordered_polygon(double x, double y, const std::vector<Point>& polygon);
    
};

#endif // GEOMETRY_TOOLS_H
