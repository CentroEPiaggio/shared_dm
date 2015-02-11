#include "../include/dual_manipulation_shared/geometry_tools.h"
#include <iostream> 

// bool Point::operator <(const Point &p) const {
//     return x < p.x || (x == p.x && y < p.y);
// }

bool geometry_tools::point_in_ordered_polygon(double x, double y, const std::vector< Point >& polygon)
{
    bool result = false;
    if (polygon.size() < 3) return false;
    int j = polygon.size() - 1;
    for (int i = 0; i < polygon.size(); i++)
    {
        if (((polygon[i].y <= y) && (y < polygon[j].y)) || 
            ((polygon[j].y <= y) && (y < polygon[i].y)))
        {
            /* compute the edge-ray intersect @ the x-coordinate */
            if (x - polygon[i].x < ((polygon[j].x - polygon[i].x) * (y - polygon[i].y) / (polygon[j].y - polygon[i].y)))
            {
                result = !result;
            }
        }
        j = i;
    }
    return result;
}



