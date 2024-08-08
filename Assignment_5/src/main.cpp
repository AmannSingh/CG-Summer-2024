// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"

#include <gif.h>
#include <fstream>

#include <Eigen/Geometry>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

// Image height
const int H = 480;

// Camera settings
const double near_plane = 1.5; // AKA focal length
const double far_plane = near_plane * 100;
const double field_of_view = 0.7854; // 45 degrees
const double aspect_ratio = 1.5;
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 3);
const Vector3d camera_gaze(0, 0, -1);
const Vector3d camera_top(0, 1, 0);

// Object
const std::string data_dir = DATA_DIR;
const std::string mesh_filename(data_dir + "bunny.off");
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)

// Material for the object
const Vector3d obj_diffuse_color(0.5, 0.5, 0.5);
const Vector3d obj_specular_color(0.2, 0.2, 0.2);
const double obj_specular_exponent = 256.0;

// Lights
std::vector<Vector3d> light_positions;
std::vector<Vector3d> light_colors;
// Ambient light
const Vector3d ambient_light(0.3, 0.3, 0.3);

// Fills the different arrays
void setup_scene()
{
    // Loads file
    std::ifstream in(mesh_filename);
    if (!in.good())
    {
        std::cerr << "Invalid file " << mesh_filename << std::endl;
        exit(1);
    }
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

    // Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16);
}

void build_uniform(UniformAttributes &uniform)
{
    // TODO: setup uniform
    double top, bottom, right, left, near, far;

    top = near_plane * std::tan(field_of_view / 2);
    bottom = -top;
    right = top * aspect_ratio;
    left = -(top * aspect_ratio);
    near = -near_plane;
    far = -far_plane;

    Matrix4d orth_matrix;
    orth_matrix << 2 / (right - left), 0, 0, -(right + left) / (right - left),
        0, 2 / (top - bottom), 0, -(top + bottom) / (top - bottom),
        0, 0, 2 / (near - far), -(near + far) / (near - far),
        0, 0, 0, 1;

    // TODO: setup camera, compute w, u, v

    Vector3d w_camera = -(camera_gaze).normalized();
    Vector3d u_camera = camera_top.cross(w_camera).normalized();
    Vector3d v_camera = w_camera.cross(u_camera);

    // TODO: compute the camera transformation
    Matrix4d temp_camera_matrix;
    Matrix4d camera_matrix;

    temp_camera_matrix << u_camera(0), v_camera(0), w_camera(0), camera_position(0),
        u_camera(1), v_camera(1), w_camera(1), camera_position(1),
        u_camera(2), v_camera(2), w_camera(2), camera_position(2),
        0, 0, 0, 1;

    camera_matrix << temp_camera_matrix.inverse();

    // TODO: setup projection matrix

    Matrix4d P;
    if (is_perspective)
    {
        // TODO setup prespective camera
        P << near, 0, 0, 0,
            0, near, 0, 0,
            0, 0, (near + far), -(far * near),
            0, 0, 1, 0;

        uniform.view = orth_matrix * P * camera_matrix;
    }
    else
    {
        uniform.view = orth_matrix * camera_matrix;
    }
}

void simple_render(Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: fill the shader
        VertexAttributes out;
        out.position = uniform.view * va.position;
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous)
    {
        // TODO: fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;
    // TODO: build the vertex attributes from vertices and facets

    VertexAttributes v1;
    VertexAttributes v2;
    VertexAttributes v3;

    // *************************** IMPORTANT!! *******************************//


    // don't know why my bunny doesn't turn red

    for (int i = 0; i < facets.rows(); i++)
    {
        v1 = VertexAttributes(vertices(facets(i, 0), 0), vertices(facets(i, 0), 1), vertices(facets(i, 0), 2));
        v2 = VertexAttributes(vertices(facets(i, 1), 0), vertices(facets(i, 1), 1), vertices(facets(i, 1), 2));
        v3 = VertexAttributes(vertices(facets(i, 2), 0), vertices(facets(i, 2), 1), vertices(facets(i, 2), 2));

        vertex_attributes.push_back(v1);
        vertex_attributes.push_back(v2);
        vertex_attributes.push_back(v3);
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

Matrix4d compute_rotation(const double alpha)
{
    // TODO: Compute the rotation matrix of angle alpha on the y axis around the object barycenter
    Matrix4d res;

    res << std::cos(alpha), 0, std::sin(alpha), 0,
        0, 1, 0, 0,
        -std::sin(alpha), 0, std::cos(alpha), 0,
        0, 0, 0, 1;
    return res;
}

void wireframe_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    Matrix4d trafo = compute_rotation(alpha);
    uniform.view = uniform.view * trafo;
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: fill the shader
        VertexAttributes out;
        out.position = uniform.view * va.position;
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous)
    {
        // TODO: fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;

    // TODO: generate the vertex attributes for the edges and rasterize the lines
    // TODO: use the transformation matrix

    VertexAttributes v1;
    VertexAttributes v2;
    VertexAttributes v3;

    // IMPORTANT: I dont know why this wont turn red
    for (int i = 0; i < facets.rows(); i++)
    {
        v1 = VertexAttributes(vertices(facets(i, 0), 0), vertices(facets(i, 0), 1), vertices(facets(i, 0), 2));
        v2 = VertexAttributes(vertices(facets(i, 1), 0), vertices(facets(i, 1), 1), vertices(facets(i, 1), 2));
        v3 = VertexAttributes(vertices(facets(i, 2), 0), vertices(facets(i, 2), 1), vertices(facets(i, 2), 2));

        // a-b
        vertex_attributes.push_back(v1);
        vertex_attributes.push_back(v2);
        // b-c
        vertex_attributes.push_back(v2);
        vertex_attributes.push_back(v3);
        // c-a
        vertex_attributes.push_back(v3);
        vertex_attributes.push_back(v1);
    }

    rasterize_lines(program, uniform, vertex_attributes, 0.5, frameBuffer);
}

void get_shading_program(Program &program)
{
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: transform the position and the normal
        VertexAttributes out;
        out.position = uniform.view * va.position;
        out.normal = va.normal;

        Vector3d light_colors2(0, 0, 0);
        Vector3d p(out.position[0], out.position[1], out.position[2]);
        Vector3d N(va.normal[0], va.normal[1], va.normal[2]);

        // TODO: compute the correct lighting
        for (int i = 0; i < light_positions.size(); i++)
        {
            const Vector3d light_position = light_positions[i];
            const Vector3d light_color = light_colors[i];
            const Vector3d li = (light_position - p).normalized();
            const Vector3d diffuse = obj_diffuse_color * std::max(li.dot(N), 0.0);
            const Vector3d hi = (li - p).normalized();
            const Vector3d specular = obj_specular_color * std::pow(std::max(N.dot(hi), 0.0), obj_specular_exponent);

            const Vector3d D = light_position - p;
            light_colors2 += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
        }

        light_colors2 += ambient_light;

        Vector4d C(light_colors2[0], light_colors2[1], light_colors2[2], 1);
        out.color = C;
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: create the correct fragment
        FragmentAttributes out(va.color(0), va.color(1), va.color(2), va.color(3));
        out.position = va.position;

        if (is_perspective)
        {
            out.position[2] = va.position[2];
        }
        else
        {
            out.position[2] = -va.position[2];
        }

        return out;
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous)
    {
        // TODO: implement the depth check
        if (fa.position[2] < previous.depth)
        {
            FrameBufferAttributes out(fa.color[0], fa.color[1], fa.color[2], fa.color[3]);
            out.depth = fa.position[2];
            return out;
        }
        else
            return previous;
    };
}

void flat_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);
    Eigen::Matrix4d trafo = compute_rotation(alpha);

    std::vector<VertexAttributes> vertex_attributes;
    // TODO: compute the normals
    uniform.view = uniform.view * trafo;
    VertexAttributes v1;
    VertexAttributes v2;
    VertexAttributes v3;

    for (int i = 0; i < facets.rows(); i++)
    {
        v1 = VertexAttributes(vertices(facets(i, 0), 0), vertices(facets(i, 0), 1), vertices(facets(i, 0), 2));
        v2 = VertexAttributes(vertices(facets(i, 1), 0), vertices(facets(i, 1), 1), vertices(facets(i, 1), 2));
        v3 = VertexAttributes(vertices(facets(i, 2), 0), vertices(facets(i, 2), 1), vertices(facets(i, 2), 2));

        Vector3d a(vertices(facets(i, 0), 0), vertices(facets(i, 0), 1), vertices(facets(i, 0), 2));
        Vector3d b(vertices(facets(i, 1), 0), vertices(facets(i, 1), 1), vertices(facets(i, 1), 2));
        Vector3d c(vertices(facets(i, 2), 0), vertices(facets(i, 2), 1), vertices(facets(i, 2), 2));

        Vector3d edge1 = b - a;
        Vector3d edge2 = c - a;
        Vector3d n = edge1.cross(edge2).normalized();

        v1.normal = n;
        v2.normal = n;
        v3.normal = n;

        vertex_attributes.push_back(v1);
        vertex_attributes.push_back(v2);
        vertex_attributes.push_back(v3);
    }

    // TODO: set material colors

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

void pv_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    Eigen::Matrix4d trafo = compute_rotation(alpha);

    // TODO: compute the vertex normals as vertex normal average
    std::vector<VertexAttributes> vertex_attributes;
    // TODO: create vertex attributes
    // TODO: set material colors
    uniform.view = uniform.view * trafo;
    VertexAttributes v1;
    VertexAttributes v2;
    VertexAttributes v3;
    MatrixXd norms(facets.rows(), vertices.cols());

    for (int i = 0; i < facets.rows(); i++)
    {

        Vector3d a(vertices(facets(i, 0), 0), vertices(facets(i, 0), 1), vertices(facets(i, 0), 2));
        Vector3d b(vertices(facets(i, 1), 0), vertices(facets(i, 1), 1), vertices(facets(i, 1), 2));
        Vector3d c(vertices(facets(i, 2), 0), vertices(facets(i, 2), 1), vertices(facets(i, 2), 2));

        Vector3d edge1 = b - a;
        Vector3d edge2 = c - a;
        Vector3d n = edge1.cross(edge2).normalized();

        for (int j = 0; j < 3; j++)
        {
            norms(facets(i, 0), j) += n(j);
            norms(facets(i, 1), j) += n(j);
            norms(facets(i, 2), j) += n(j);
        }
       
    }
    for (int i = 0; i < facets.rows(); i++)
    {
        v1 = VertexAttributes(vertices(facets(i, 0), 0), vertices(facets(i, 0), 1), vertices(facets(i, 0), 2));
        v2 = VertexAttributes(vertices(facets(i, 1), 0), vertices(facets(i, 1), 1), vertices(facets(i, 1), 2));
        v3 = VertexAttributes(vertices(facets(i, 2), 0), vertices(facets(i, 2), 1), vertices(facets(i, 2), 2));

        Vector3d a(vertices(facets(i, 0), 0), vertices(facets(i, 0), 1), vertices(facets(i, 0), 2));
        Vector3d b(vertices(facets(i, 1), 0), vertices(facets(i, 1), 1), vertices(facets(i, 1), 2));
        Vector3d c(vertices(facets(i, 2), 0), vertices(facets(i, 2), 1), vertices(facets(i, 2), 2));

        v1.normal << norms(facets(i, 0), 0), norms(facets(i, 0), 1), norms(facets(i, 0), 2);
        v2.normal << norms(facets(i, 1), 0), norms(facets(i, 1), 1), norms(facets(i, 1), 2);
        v3.normal << norms(facets(i, 2), 0), norms(facets(i, 2), 1), norms(facets(i, 2), 2);

        v1.normal = v1.normal.normalized();
        v2.normal = v2.normal.normalized();
        v3.normal = v3.normal.normalized();

        vertex_attributes.push_back(v1);
        vertex_attributes.push_back(v2);
        vertex_attributes.push_back(v3);
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

int main(int argc, char *argv[])
{
    setup_scene();

    int W = H * aspect_ratio;
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(W, H);
    
    vector<uint8_t> image;

    simple_render(frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("simple.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());
    wireframe_render(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("wireframe.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());
    flat_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("flat_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());
    pv_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("pv_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    // TODO: add the animation
    const char* fileName = "wireframe.gif";
    const char* fileName2 = "flat_shading.gif";
    const char* fileName3 = "pv_shading.gif";
    int delay = 25;
    GifWriter g;
    GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);

    for (float i = 1; i < 25; i += 0.5)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        wireframe_render(i, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    GifEnd(&g);

    GifBegin(&g, fileName2, frameBuffer.rows(), frameBuffer.cols(), delay);
    for (double i = 1; i < 25; i += 0.5)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        flat_shading(i, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    GifEnd(&g);

    GifBegin(&g, fileName3, frameBuffer.rows(), frameBuffer.cols(), delay);
    for (double i = 1; i < 25; i += 0.5)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        pv_shading(i, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    GifEnd(&g);
    

    return 0;
}
