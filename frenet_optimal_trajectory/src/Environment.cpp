#include "Environment.h"

void Environment::generate_global_map()
{
    VEC_1D theta;
    double cx, cy, cr;
    double curve_step = 0.1, line_step = 4;

    for (int i = 75; i > 30; i -= line_step)
    {
        reference_line_x.push_back(10);
        reference_line_y.push_back(i);
    }
    for (int i = 75; i > 30; i -= line_step)
    {
        lower_boundary_x.push_back(10 + road_width);
        lower_boundary_y.push_back(i);
        upper_boundary_x.push_back(10 - road_width);
        upper_boundary_y.push_back(i);
    }

    cx = 30, cy = 30, cr = 20;
    for (double i = M_PI; i <= 1.5 * M_PI; i += curve_step)
    {
        theta.push_back(i);
    }
    for (const double itheta : theta)
    {
        reference_line_x.push_back(cx + cr * cos(itheta));
        reference_line_y.push_back(cy + cr * sin(itheta));
    }
    for (int i = 30; i < 80; i += line_step)
    {
        reference_line_x.push_back(i);
        reference_line_y.push_back(10);
    }

    theta.resize(0);
    for (double i = M_PI; i <= 1.5 * M_PI; i += curve_step)
    {
        theta.push_back(i);
    }
    for (const double itheta : theta)
    {
        lower_boundary_x.push_back(cx + (cr - road_width) * cos(itheta));
        lower_boundary_y.push_back(cy + (cr - road_width) * sin(itheta));
        upper_boundary_x.push_back(cx + (cr + road_width) * cos(itheta));
        upper_boundary_y.push_back(cy + (cr + road_width) * sin(itheta));
    }
    for (int i = 30; i <= 80; i += line_step)
    {
        lower_boundary_x.push_back(i);
        lower_boundary_y.push_back(10 + road_width);
        upper_boundary_x.push_back(i);
        upper_boundary_y.push_back(10 - road_width);
    }

    theta.resize(0);
    cx = 80, cy = 30, cr = 20;
    for (double i = -M_PI / 2.0; i <= M_PI / 2.0; i += curve_step)
    {
        theta.push_back(i);
    }
    for (const double itheta : theta)
    {
        reference_line_x.push_back(cx + cr * cos(itheta));
        reference_line_y.push_back(cy + cr * sin(itheta));
    }
    for (int i = 80; i > 60; i -= line_step)
    {
        reference_line_x.push_back(i);
        reference_line_y.push_back(50);
    }

    theta.resize(0);
    for (double i = -M_PI / 2.0; i <= M_PI / 2.0; i += curve_step)
    {
        theta.push_back(i);
    }
    for (const double itheta : theta)
    {
        lower_boundary_x.push_back(cx + (cr - road_width) * cos(itheta));
        lower_boundary_y.push_back(cy + (cr - road_width) * sin(itheta));
        upper_boundary_x.push_back(cx + (cr + road_width) * cos(itheta));
        upper_boundary_y.push_back(cy + (cr + road_width) * sin(itheta));
    }
    for (int i = 80; i >= 60; i -= line_step)
    {
        lower_boundary_x.push_back(i);
        lower_boundary_y.push_back(50 - road_width);
        upper_boundary_x.push_back(i);
        upper_boundary_y.push_back(50 + road_width);
    }

    theta.resize(0);
    cx = 60, cy = 70, cr = 20;
    for (double i = -M_PI / 2.0; i >= -M_PI; i -= 0.1)
    {
        theta.push_back(i);
    }
    for (const double itheta : theta)
    {
        reference_line_x.push_back(cx + cr * cos(itheta));
        reference_line_y.push_back(cy + cr * sin(itheta));
    }

    theta.resize(0);
    for (double i = -M_PI / 2.0; i >= -M_PI; i -= 0.1)
    {
        theta.push_back(i);
    }
    for (const double itheta : theta)
    {
        lower_boundary_x.push_back(cx + (cr + road_width) * cos(itheta));
        lower_boundary_y.push_back(cy + (cr + road_width) * sin(itheta));
        upper_boundary_x.push_back(cx + (cr - road_width) * cos(itheta));
        upper_boundary_y.push_back(cy + (cr - road_width) * sin(itheta));
    }

    theta.resize(0);
    cx = 10, cy = 70, cr = 30;
    for (double i = 0; i <= M_PI; i += curve_step)
    {
        theta.push_back(i);
    }
    for (const double itheta : theta)
    {
        reference_line_x.push_back(cx + cr * cos(itheta));
        reference_line_y.push_back(cy + cr * sin(itheta));
    }
    for (int i = 60; i > -40; i -= line_step)
    {
        reference_line_x.push_back(-20);
        reference_line_y.push_back(i);
    }

    theta.resize(0);
    for (double i = 0; i <= M_PI; i += curve_step)
    {
        theta.push_back(i);
    }
    for (const double itheta : theta)
    {
        lower_boundary_x.push_back(cx + (cr - road_width) * cos(itheta));
        lower_boundary_y.push_back(cy + (cr - road_width) * sin(itheta));
        upper_boundary_x.push_back(cx + (cr + road_width) * cos(itheta));
        upper_boundary_y.push_back(cy + (cr + road_width) * sin(itheta));
    }
    for (int i = 60; i >= -40; i -= line_step)
    {
        lower_boundary_x.push_back(-20 + road_width);
        lower_boundary_y.push_back(i);
        upper_boundary_x.push_back(-20 - road_width);
        upper_boundary_y.push_back(i);
    }
}