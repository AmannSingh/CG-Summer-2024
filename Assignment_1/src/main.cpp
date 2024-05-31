////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
// Shortcut to avoid  everywhere, DO NOT USE IN .h
using namespace Eigen;
////////////////////////////////////////////////////////////////////////////////

const std::string root_path = "../data";

// Computes the determinant of the matrix whose columns are the vector u and v
double inline det(const Vector2d &u, const Vector2d &v)
{
    // TODO 
    double determinant;
    determinant = u.x() * v.y() - u.y()*v.x();

    return determinant;
}

// Return true iff [a,b] intersects [c,d]
bool intersect_segment(const Vector2d &a, const Vector2d &b, const Vector2d &c, const Vector2d &d)

{    // https://www.youtube.com/watch?v=l_UlG-oVxus 
    // used above resource to determine how to calculate intersection 
    // don't understand how to use the determinant to determine if there is an intersection or not 

    // TODO

    // if the point is in between the two y coordinants of the polygon
    // and is less than the intersection point of the x axis   

    if( ((a.y() < c.y()) != (a.y() < d.y())) && a.x() < (d.x()-c.x()*(a.y()-c.y())/(d.y()-c.y()) + c.x()))
    {
        return true;
    }

    else
    {
        return false;
    }

}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const std::vector<Vector2d> &poly, const Vector2d &query)
{
    size_t n = poly.size(); 
    int intersections = 0; 

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    // 1. Compute bounding box and set coordinate of a point outside the polygon
    // TODO
    
   for(auto &point: poly ){
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x,point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y,point.y());
   }
    Vector2d outside(max_x +1, max_y +1);

    // 2. Cast a ray from the query point to the 'outside' point, count number of intersections
    // TODO

    for(auto i = 1; i<n-1; i++){
        
        Vector2d v1 = poly[i];
        Vector2d v2 = poly[i+1];  // next vertex
        if(intersect_segment(query, outside, v1, v2))
        {
            intersections++;
        }
    }
   
    if((intersections % 2) == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Vector2d> load_xyz(const std::string &filename)
{
    std::vector<Vector2d> points;
    std::ifstream in(filename);
    // TODO
    double x, y, z; 

    //error checking 

    if (!in.is_open()) 
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return points;
    }
    
    //read file and store values into x y z 
    while( in >> x >> y >> z)
    {   
        // placing y and z because code seems to put x coordinant as 0 (z coordinant)
        // essentially coordinants have shifted 

         points.push_back(Vector2d(y,z));
         
        //uncomment below line if that doesn't occur for you 
        //points.push_back(Vector2d(x,y));

       
    }

    in.close();

    return points;
}

void save_xyz(const std::string &filename, const std::vector<Vector2d> &points)
{
    // TODO
    std::ofstream out(filename);

    //check to see if file open then save 

    if(out.is_open())
    {
        for(const auto &point: points)
        {   
            out << point.x()<< " " <<point.y() << "0\n";
        }
        
        out.close();
    }
    else
    {
        std::cerr << "Error opening file\n";
    }
}

std::vector<Vector2d> load_obj(const std::string &filename)
{
    std::ifstream in(filename);
    std::vector<Vector2d> points;
    std::vector<Vector2d> poly;
    char key;
    while (in >> key)
    {   

        if (key == 'v')
        {
            double x, y, z;
            in >> x >> y >> z;
            points.push_back(Vector2d(x, y));
        }
        else if (key == 'f')
        {
            std::string line;
            std::getline(in, line);
            std::istringstream ss(line);
            int id;
            while (ss >> id)
            {
                poly.push_back(points[id - 1]);
            }
        }
    }
    return poly;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    const std::string points_path = root_path + "/points.xyz";
    const std::string poly_path = root_path + "/polygon.obj";
    std::vector<Vector2d> points = load_xyz(points_path);

    ////////////////////////////////////////////////////////////////////////////////
    //Point in polygon
    std::vector<Vector2d> poly = load_obj(poly_path);
    std::vector<Vector2d> result;
    for (size_t i = 0; i < points.size(); ++i)
    { 
        if (is_inside(poly, points[i]))
        {
            result.push_back(points[i]);
        }
    }

    save_xyz("output.xyz", result);

    return 0;
}
