// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

bool intersect_ray_triangle(const Vector3d& ray_origin, const Vector3d& ray_direction, const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, Vector3d& ray_intersection){

/* using Möller Trumbore ray triangle intersection algorithm 
*  Resources:
*   1. https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection.html
*   2. https://www.youtube.com/watch?v=fK1RPmF_zjQ
*   3. https://www.realtimerendering.com/intersections.html
*  
*/
    Vector3d edge1 = v1 - v0;
    Vector3d edge2 = v2 - v0; 
    Vector3d ray_direction_cross_e2 = ray_direction.cross(edge2);
    double det = edge1.dot(ray_direction_cross_e2);

    if( fabs(det) < __DBL_EPSILON__)
    {
        return false;  //ray is parallel to triangle 
    }

    // calculate distance from v0 to ray_origin
    Vector3d v0_to_ray_origin = ray_origin - v0;
    
    double inverted_det = 1.0/det; 

    //calculate u and check bounds
    double u = inverted_det * v0_to_ray_origin.dot(ray_direction_cross_e2);

    if(u< 0.0 || u > 1.0)
    {
        return false;
    }

    //calculate v and check bounds 
    Vector3d q = v0_to_ray_origin.cross(edge1);
    double v = inverted_det * ray_direction.dot(q);

    if (v < 0.0 || u + v > 1.0)
    {
        return false;
    }

    //calculate intersection point

    double t = inverted_det * edge2.dot(q);

    if(t> __DBL_EPSILON__)
    {
        ray_intersection = ray_origin + t*ray_direction; 
        return true;
    }

    return false; 
}

bool parallelogram_into_triangles_intersection(const Vector3d& ray_origin, const Vector3d& ray_direction, const Vector3d& pgram_origin, const Vector3d& pgram_u, const Vector3d& pgram_v, Vector3d& ray_intersection)
{ 
    
    // split parallelogram into two triangles 
    Vector3d v0 = pgram_origin;
    Vector3d v1 = pgram_origin + pgram_u;
    Vector3d v2 = pgram_origin + pgram_v;
    Vector3d v3 = pgram_origin + pgram_u + pgram_v;

    // Check intersection with the triangles
    if(intersect_ray_triangle(ray_origin, ray_direction, v0, v1, v2, ray_intersection))
    {
        return true;
    }
    if(intersect_ray_triangle(ray_origin, ray_direction, v1, v2, v3, ray_intersection))
    {
        return true;
    }

    return false;

}

bool intersect_sphere(const Vector3d& ray_origin, const Vector3d& ray_direction, const Vector3d& sphere_center, double sphere_radius, double& t)
{
    Vector3d center_to_origin =  ray_origin - sphere_center; 

    // define quadratic formula from lecture 
    double a = ray_direction.dot(ray_direction); 
    double b = 2.0 * center_to_origin.dot(ray_direction);
    double c = center_to_origin.dot(center_to_origin) - (sphere_radius * sphere_radius); 

    double x = ((b*b) - (4*a*c) ); 

    // check to see if quadratic has a solution 
    if(x < 0)
    {
        return false; 
    }

    x = sqrt(x); 
    double t1 =(-b + x)/(2*a);
    double t2 =(-b - x)/(2*a);

    //determine closest solution to sphere
    if(t1 < 0 && t2 < 0){
        return false; 
    }
    else if( t1 < 0 || t1 >= t2 && t2> 0){
        t = t2; 
    }
    else if(t2< 0 || t2>=t1 && t1>0){
        t = t1;
    }

    return true; 



}

void raytrace_sphere()
{
    std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

    const std::string filename("sphere_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
            Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
            const double sphere_radius = 0.9;

            if (ray_on_xy.norm() < sphere_radius)
            {
                // The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection(
                    ray_on_xy(0), ray_on_xy(1),
                    sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_parallelogram()
{
    std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

    const std::string filename("plane_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;
            Vector3d ray_intersection;

            // TODO: Check if the ray intersects with the parallelogram
            if(parallelogram_into_triangles_intersection(ray_origin,ray_direction, pgram_origin, pgram_u, pgram_v, ray_intersection))
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection point
                /*stored in intersect_ray_triangle() function*/

                // TODO: Compute normal at the intersection point

                Vector3d ray_normal = -(pgram_u.cross(pgram_v).normalized());
                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
    std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

    const std::string filename("plane_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction) 
            const Vector3d ray_direction = (pixel_center - camera_origin).normalized();
            Vector3d ray_intersection;

            // TODO: Check if the ray intersects with the parallelogram
            if (parallelogram_into_triangles_intersection(camera_origin,ray_direction, pgram_origin, pgram_u, pgram_v, ray_intersection))
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection point
                /*stored in intersect_ray_triangle() function*/

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = -(pgram_u.cross(pgram_v).normalized());

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;
                
                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_shading()
{
    std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

    const std::string filename("shading.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / A.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / A.rows(), 0);

    //Sphere setup
    const Vector3d sphere_center(0, 0, 0);
    const double sphere_radius = 0.9;

    //material params
    const Vector3d diffuse_color(1, 0, 1);
    const double specular_exponent = 100;
    const Vector3d specular_color(0., 0, 1);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    double ambient = 0.1;

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // Intersect with the sphere
            // TODO: implement the generic ray sphere intersection
            double t;
            if (intersect_sphere(pixel_center, ray_direction, sphere_center, sphere_radius, t))
            {
                // TODO: The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection = ray_origin + t* ray_direction;

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = (ray_intersection - sphere_center).normalized();

                // TODO: Add shading parameter here
                const double diffuse = (light_position - ray_intersection).normalized().dot(ray_normal);
                const double specular = (light_position - ray_intersection).normalized().dot(ray_normal);

                // Simple diffuse model
                C(i, j) = ambient + diffuse + specular;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

int main()
{
    raytrace_sphere();
    raytrace_parallelogram();
    raytrace_perspective();
    raytrace_shading();

    return 0;
}
