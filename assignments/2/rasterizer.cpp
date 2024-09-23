// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // Vector3f
    // float x = _x; float y = _y;
    Vector3f v1(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), _v[1].z() - _v[0].z());
    Vector3f p1(x - _v[0].x(), y - _v[0].y(), 0 - _v[0].z());

    Vector3f v2(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), _v[2].z() - _v[1].z());
    Vector3f p2(x - _v[1].x(), y - _v[1].y(), 0 - _v[1].z());

    Vector3f v3(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), _v[0].z() - _v[2].z());
    Vector3f p3(x - _v[2].x(), y - _v[2].y(), 0 - _v[2].z());

    auto z1 = v1.cross(p1)[2], z2 = v2.cross(p2)[2], z3 = v3.cross(p3)[2];
    return (z1 > 0 && z2 > 0 && z3 > 0) ||
           (z1 < 0 && z2 < 0 && z3 < 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    const auto minx = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    const auto miny = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    const auto maxx = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    const auto maxy =std::max(std::max(v[0].y(), v[1].y()), v[2].y());
#define VERSION1
#ifndef VERSION1
    const ushort sample_times = 2;
    const float sample_rate = 1.0 / sample_times;
    static std::vector<float> sample_depth_buf;
    sample_depth_buf.resize(width * height * sample_times * sample_times);
#endif

    const auto get_z_interpolated = [v,t](const float& x, const float& y){
        const auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
        const float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
        z_interpolated *= w_reciprocal;
        return z_interpolated;
    };

    for (size_t x = minx; x < maxx; x++)
    {
        for (size_t y = miny; y < maxy; y++)
        {
#ifdef VERSION1
            if (insideTriangle(x + 0.5, y + 0.5, t.v)){
                const auto z_interpolated = get_z_interpolated(x,y);
                const auto depth_index = get_index(x,y);
                if (z_interpolated < depth_buf[depth_index]){
                    set_pixel(Vector3f(x, y,0),t.getColor());
                    depth_buf[depth_index] = z_interpolated;
                }
            }
#else
            float bias = 0;
            bool need_to_set = false;
            for (size_t i = 0; i < sample_times; i++)
            {
                for (size_t j = 0; j < sample_times; j++)
                {
                    float xx = x + i * sample_rate + sample_rate * .5;
                    float yy = y + j * sample_rate + sample_rate * .5;
                    if (insideTriangle(xx, yy, t.v))
                    {
                        bias += 1;
                        const auto z_interpolated = get_z_interpolated(xx,yy);
                        const auto depth_index = (height * sample_times -1-y)*width * sample_times + x;
                        if (z_interpolated < sample_depth_buf[depth_index]){
                            need_to_set = true;
                            sample_depth_buf[depth_index] = z_interpolated;
                        }
                    }
                }
            }
            if (bias > 0 && need_to_set)
                set_pixel(Vector3f(x, y,0),(bias /  (sample_times * sample_times)) * t.getColor());
#endif
        }
    }

    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    const auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

// clang-format on