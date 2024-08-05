////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh

private:
    // builds the bvh recursively
    int build_recursive(const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, int from, int to, int parent, std::vector<int> &triangles);
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "bunny.off");

// Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; // 45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

// spheres
std::vector<Vector3d> sphere_centers;
std::vector<double> sphere_radii;

// parallelograms
std::vector<Matrix3d> parallelograms;

// Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

// Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
// Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

// Fills the different arrays
void setup_scene()
{
    // Loads file
    std::ifstream in(mesh_filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    // setup tree
    bvh = AABBTree(vertices, facets);

    // Spheres
    sphere_centers.emplace_back(10, 0, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(7, 0.05, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(4, 0.1, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(1, 0.2, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-2, 0.4, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-5, 0.8, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-8, 1.6, 1);
    sphere_radii.emplace_back(1);

    // parallelograms
    parallelograms.emplace_back();
    parallelograms.back() << -100, 100, -100,
        -1.25, 0, -1.2,
        -100, -100, 100;

    // Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    // Vector containing the list of tringle indices
    std::vector<int> triangles(F.rows());
    std::iota(triangles.begin(), triangles.end(), 0);

    root = build_recursive(V, F, centroids, 0, triangles.size(), -1, triangles);
}

int AABBTree::build_recursive(const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, int from, int to, int parent, std::vector<int> &triangles)
{
    // Scene is empty, so is the aabb tree
    if (to - from == 0)
    {
        return -1;
    }

    // If there is only 1 triangle left, then we are at a leaf
    if (to - from == 1)
    {
        // TODO create leaf node and retun correct left index
        Node leaf;
        leaf.bbox.extend(V.row(F(triangles[from], 0)).transpose());
        leaf.bbox.extend(V.row(F(triangles[from], 1)).transpose());
        leaf.bbox.extend(V.row(F(triangles[from], 2)).transpose());
        leaf.left = -1;
        leaf.right = -1;
        leaf.parent = parent;
        leaf.triangle = triangles[from];
        nodes.push_back(leaf);
        return nodes.size() - 1;
    }

    AlignedBox3d centroid_box;

    // TODO Use AlignedBox3d to find the box around the current centroids
    for (int i = from; i < to; ++i)
    {
        centroid_box.extend(centroids.row(triangles[i]).transpose());
    }

    // Diagonal of the box
    Vector3d extent = centroid_box.diagonal();

    // TODO find the largest dimension
    int longest_dim = 0;
    if (extent[1] > extent[0])
    {
        longest_dim = 1;
    }
    if (extent[2] > extent[longest_dim])
    {
        longest_dim = 2;
    }

    // TODO sort centroids along the longest dimension
    std::sort(triangles.begin() + from, triangles.begin() + to, [&](int f1, int f2)
              {
                  // TODO sort the **triangles** along the centroid largest dimension
                  //  return true if triangle f1 comes before triangle f2
                  return centroids(f1, longest_dim) < centroids(f2, longest_dim); });

    // TODO Create a new internal node and do a recursive call to build the left and right part of the tree
    // TODO finally return the correct index
    Node internal;
    internal.bbox = centroid_box;
    internal.parent = parent;
    internal.triangle = -1;
    nodes.push_back(internal);
    int node_index = nodes.size() - 1;

    // Recursively build left and right children
    int mid = (from + to) / 2;
    int left_child = build_recursive(V, F, centroids, from, mid, node_index, triangles);
    int right_child = build_recursive(V, F, centroids, mid, to, node_index, triangles);

    nodes[node_index].left = left_child;
    nodes[node_index].right = right_child;

    return node_index;
}

////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // TODO
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.
    Vector3d edge1, edge2, ray_direction_cross, a_to_ray_origin, q;
    double det, inverted_det, u, v;
    edge1 = b - a;
    edge2 = c - a;
    ray_direction_cross = ray_direction.cross(edge2);
    det = edge1.dot(ray_direction_cross);

    if (fabs(det) < __DBL_EPSILON__)
    {
        return -1;
    }

    inverted_det = 1.0 / det;
    a_to_ray_origin = ray_origin - a;
    u = inverted_det * a_to_ray_origin.dot(ray_direction_cross);

    if (u < 0.0 || u > 1.0)
    {
        return -1;
    }

    q = a_to_ray_origin.cross(edge1);
    v = inverted_det * ray_direction.dot(q);

    if (v < 0.0 || u + v > 1.0)
    {
        return -1;
    }

    double t = inverted_det * edge2.dot(q);

    if (t < 0)
    {
        return -1;
    }

    p = ray_origin + (t * ray_direction);
    N = (edge1.cross(edge2)).normalized();
    return t;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // TODO
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.

    // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html

    Vector3d min = box.min();
    Vector3d max = box.max();

    double xMin = (min[0] - ray_origin[0]) / ray_direction[0];
    double xMax = (max[0] - ray_origin[0]) / ray_direction[0];

    if (xMin > xMax)
    {
        std::swap(xMin, xMax);
    }
    double yMin = (min[1] - ray_origin[1]) / ray_direction[1];
    double yMax = (max[1] - ray_origin[1]) / ray_direction[1];

    if (yMin > yMax)
    {
        std::swap(yMin, yMax);
    }
    if ((xMin > yMax) || (yMin > xMax))
    {
        return false;
    }
    if (yMin > xMin)
    {
        xMin = yMin;
    }
    if (yMax < xMax)
    {
        xMax = yMax;
    }
    double zMin = (min[2] - ray_origin[2]) / ray_direction[2];
    double zMax = (max[2] - ray_origin[2]) / ray_direction[2];

    if (xMin > zMax)
    {
        std::swap(zMin, zMax);
    }
    if ((xMin > zMax) || (zMin > xMax))
    {
        return false;
    }
    if (zMin > xMin)
    {
        xMin = zMin;
    }
    if (zMax < xMax)
    {
        xMax = zMax;
    }
    return true;
}

double parallelogram_into_triangles_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &pgram_origin, const Vector3d &pgram_u, const Vector3d &pgram_v, Vector3d &p, Vector3d &N)
{

    // split parallelogram into two triangles
    Vector3d v0 = pgram_origin;
    Vector3d v1 = pgram_origin + pgram_u;
    Vector3d v2 = pgram_origin + pgram_v;
    Vector3d v3 = pgram_origin + pgram_u + pgram_v;

    // Check intersection with the triangles
    double t = ray_triangle_intersection(ray_origin, ray_direction, v0, v1, v2, p, N);
    if (t != -1)
    {
        return t;
    }
    t = ray_triangle_intersection(ray_origin, ray_direction, v1, v2, v3, p, N);
    if (t != -1)
    {
        return t;
    }

    return -1;
}

double intersect_sphere(const Vector3d& ray_origin, const Vector3d& ray_direction, const Vector3d& sphere_center, double sphere_radius, Vector3d &p, Vector3d &N)
{
    Vector3d center_to_origin =  ray_origin - sphere_center; 
    double t;
    // define quadratic formula from lecture 
    double a = ray_direction.dot(ray_direction); 
    double b = 2.0 * center_to_origin.dot(ray_direction);
    double c = center_to_origin.dot(center_to_origin) - (sphere_radius * sphere_radius); 

    double x = ((b*b) - (4*a*c) ); 

    // check to see if quadratic has a solution 
    if(x < 0)
    {
        return -1; 
    }

    if (b*b - 4.0*a*c < 0.0)
    {
        return -1;
    }
    else
    {
    
    t = (-b - sqrt((b*b) - 4.0*a*c))/(2.0*a);
    
    p = ray_origin + (t * ray_direction);
    N = (p - sphere_center).normalized(); 
    }    
    

    return t;   
}

// Finds the closest intersecting object returns its index
// In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;

    // TODO
    // Method (1): Traverse every triangle and return the closest hit.
    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.
    bool is_intersection = false;
    double nearest_obj = std::numeric_limits<double>::max();

    int total_rows = facets.rows();
    for (int i = 0; i <= total_rows; ++i)
    {
        Vector3d v0 = vertices.row(facets(i, 0));
        Vector3d v1 = vertices.row(facets(i, 1));
        Vector3d v2 = vertices.row(facets(i, 2));

        const double t = ray_triangle_intersection(ray_origin, ray_direction, v0, v1, v2, tmp_p, tmp_N);
        if (t >= 0)
        {
            if (t < nearest_obj)
            {
                nearest_obj = t;
                p = tmp_p;
                N = tmp_N;
                is_intersection = true;
            }
        }
    }

    for (int i = 0; i < parallelograms.size(); ++i)
    {
        const Vector3d pgram_origin = parallelograms[i].col(0);
        const Vector3d A = parallelograms[i].col(1);
        const Vector3d B = parallelograms[i].col(2);
        const Vector3d pgram_v = A - pgram_origin;
        const Vector3d pgram_u = B - pgram_origin;

        const double t = parallelogram_into_triangles_intersection(ray_origin, ray_direction, pgram_origin, pgram_u,pgram_v, tmp_p,tmp_N);
        
        if (t >= 0)
        {
            if (t < nearest_obj)
            {
                nearest_obj = t;
                p = tmp_p;
                N = tmp_N;
                is_intersection = true;
            }
        }
    }

    for(int i = 0; i <sphere_centers.size(); i++){
        const Vector3d sphere_center = sphere_centers[i];
        const double sphere_radius = sphere_radii[i];
        const double t = intersect_sphere(ray_origin,ray_direction, sphere_center, sphere_radius, tmp_p, tmp_N);

        if (t >= 0)
        {
            if (t < nearest_obj)
            {
                nearest_obj = t;
                p = tmp_p;
                N = tmp_N;
                is_intersection = true;
            }
        }
    }

    return is_intersection;
}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction)
{
    // Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];

        Vector4d diff_color = obj_diffuse_color;

        // Diffuse contribution
        const Vector3d Li = (light_position - p).normalized();
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    // Rendering equation
    Vector4d C = ambient_color + lights_color;

    // Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    // TODO
    double image_y = (tan(field_of_view / 2) * focal_length);
    double image_x = image_y * aspect_ratio;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    setup_scene();

    raytrace_scene();
    return 0;
}
