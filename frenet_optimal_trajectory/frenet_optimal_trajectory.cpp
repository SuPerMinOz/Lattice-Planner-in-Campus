#include <signal.h>

#include "Environment.h"
#include "CarModel.h"
#include "Spline.h"
#include "Frenet_Path.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

#define MAX_SPEED 50.0 / 3.6 // maximum speed [m/s]
#define MAX_ACCEL 2.0        // maximum acceleration [m/ss]
#define MAX_CURVATURE 2.5    // maximum curvature [1/m]
#define MAX_ROAD_WIDTH 8.0   // maximum road width [m]
#define D_ROAD_W 0.5         // road width sampling length [m]

// cruising
#define DT 0.2                  // time tick [s]
#define MAXT 5.0                // max prediction time [m]
#define MINT 4.0                // min prediction time [m]
#define TARGET_SPEED 30.0 / 3.6 // target speed [m/s]
#define D_T_S 5.0 / 3.6         // target speed sampling length [m/s]
#define N_S_SAMPLE 1            // sampling number of target speed
#define ROBOT_RADIUS 2.0        // robot radius [m]

#define KJ 0.1
#define KT 0.1
#define KD 1.0
#define KLAT 1.0
#define KLON 1.0

// stopping
#define STOP_DiSTANCE ((TARGET_SPEED * TARGET_SPEED) / MAX_ACCEL)
#define STOP_DT 0.4
#define STOP_MINT 1.0
#define STOP_MAXT 10.0
#define STOP_KD 100

std::string vehicle_state;

double sum_of_power(VEC_1D &value_list)
{
    double sum = 0;
    for (double value : value_list)
    {
        sum += value * value;
    }
    return sum;
};

VEC_FrenetPath calc_frenet_paths_for_stopping(double c_speed, double c_accel, double c_d, double c_d_d, double c_d_dd, double c_s, double target_s)
{
    VEC_FrenetPath fp_list;
    // generate path to each offset goal
    // std::vector<double> d_offet_candidate = {-1.0, -0.5, 0.0, 0.5, 1.0};
    std::vector<double> d_offet_candidate = {0.0};
    for (const double di : d_offet_candidate)
    {
        for (double Ti = STOP_MINT; Ti < STOP_MAXT + 0.1; Ti += STOP_DT)
        {
            // Lateral motion planning
            FrenetPath fp_d;
            QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
            for (double t = 0; t < Ti + 0.1; t += STOP_DT)
            {
                fp_d.t.emplace_back(t);
                fp_d.d.emplace_back(lat_qp.Evaluate(0, t));
                fp_d.d_d.emplace_back(lat_qp.Evaluate(1, t));
                fp_d.d_dd.emplace_back(lat_qp.Evaluate(2, t));
                fp_d.d_ddd.emplace_back(lat_qp.Evaluate(3, t));
            }
            // Longitudinal motion planning (Stopping)
            // std::vector<double> speed_candidate = {0.0, 0.5, 1.0, 2.0};
            std::vector<double> speed_candidate = {0.0};
            for (const double tv : speed_candidate)
            {
                FrenetPath fp_d_s = fp_d;
                QuinticPolynomial lon_qp(c_s, c_speed, c_accel, target_s, tv, 0.0, Ti);

                fp_d_s.max_speed = std::numeric_limits<double>::min();
                fp_d_s.max_accel = std::numeric_limits<double>::min();
                for (double t : fp_d.t)
                {
                    fp_d_s.s.emplace_back(lon_qp.Evaluate(0, t));
                    fp_d_s.s_d.emplace_back(lon_qp.Evaluate(1, t));
                    fp_d_s.s_dd.emplace_back(lon_qp.Evaluate(2, t));
                    fp_d_s.s_ddd.emplace_back(lon_qp.Evaluate(3, t));
                    if (fp_d_s.s_d.back() > fp_d_s.max_speed)
                    {
                        fp_d_s.max_speed = fp_d_s.s_d.back();
                    }
                    if (fp_d_s.s_dd.back() > fp_d_s.max_accel)
                    {
                        fp_d_s.max_accel = fp_d_s.s_dd.back();
                    }
                }

                double Jp = sum_of_power(fp_d.d_ddd);
                double Js = sum_of_power(fp_d_s.s_ddd);
                double ds = target_s - fp_d_s.s.back();

                fp_d_s.cd = KJ * Jp + KT * Ti;
                fp_d_s.cv = KJ * Js + KT * Ti + STOP_KD * ds * ds;
                fp_d_s.cf = KLAT * fp_d_s.cd + KLON * fp_d_s.cv;

                fp_list.emplace_back(fp_d_s);
            }
        }
    }
    return fp_list;
};

VEC_FrenetPath calc_frenet_paths_for_cruising(double c_speed, double c_accel, double c_d, double c_d_d, double c_d_dd, double c_s)
{
    VEC_FrenetPath fp_list;
    // generate path to each offset goal
    for (double di = -1 * MAX_ROAD_WIDTH; di < MAX_ROAD_WIDTH + 0.1; di += D_ROAD_W)
    {
        for (double Ti = MINT; Ti < MAXT + 0.1; Ti += DT)
        {
            // Lateral motion planning
            FrenetPath fp_d;
            QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
            for (double t = 0; t < Ti + 0.1; t += DT)
            {
                fp_d.t.emplace_back(t);
                fp_d.d.emplace_back(lat_qp.Evaluate(0, t));
                fp_d.d_d.emplace_back(lat_qp.Evaluate(1, t));
                fp_d.d_dd.emplace_back(lat_qp.Evaluate(2, t));
                fp_d.d_ddd.emplace_back(lat_qp.Evaluate(3, t));
            }
            // Longitudinal motion planning (Velocity keeping)
            // for (double tv = TARGET_SPEED - D_T_S * N_S_SAMPLE; tv < TARGET_SPEED + D_T_S * N_S_SAMPLE + 0.1; tv += D_T_S)
            for (double tv = 0.5 * TARGET_SPEED; tv < 1.5 * TARGET_SPEED + 0.1; tv += 0.1 * TARGET_SPEED)
            {
                FrenetPath fp_d_s = fp_d;
                QuarticPolynomial lon_qp(c_s, c_speed, c_accel, tv, 0.0, Ti);

                fp_d_s.max_speed = std::numeric_limits<double>::min();
                fp_d_s.max_accel = std::numeric_limits<double>::min();
                for (double t : fp_d.t)
                {
                    fp_d_s.s.emplace_back(lon_qp.Evaluate(0, t));
                    fp_d_s.s_d.emplace_back(lon_qp.Evaluate(1, t));
                    fp_d_s.s_dd.emplace_back(lon_qp.Evaluate(2, t));
                    fp_d_s.s_ddd.emplace_back(lon_qp.Evaluate(3, t));
                    if (fp_d_s.s_d.back() > fp_d_s.max_speed)
                    {
                        fp_d_s.max_speed = fp_d_s.s_d.back();
                    }
                    if (fp_d_s.s_dd.back() > fp_d_s.max_accel)
                    {
                        fp_d_s.max_accel = fp_d_s.s_dd.back();
                    }
                }

                double Jp = sum_of_power(fp_d.d_ddd);
                double Js = sum_of_power(fp_d_s.s_ddd);
                double ds = (TARGET_SPEED - fp_d_s.s_d.back()); // square of diff from target speed

                fp_d_s.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_d_s.d.back(), 2);
                fp_d_s.cv = KJ * Js + KT * Ti + KD * ds * ds;
                fp_d_s.cf = KLAT * fp_d_s.cd + KLON * fp_d_s.cv;

                fp_list.emplace_back(fp_d_s);
            }
        }
    }
    return fp_list;
};

void calc_global_paths(VEC_FrenetPath &path_list, Spline2D &trajectory)
{
    for (VEC_FrenetPath::iterator current_path = path_list.begin(); current_path != path_list.end(); current_path++)
    {
        for (size_t i = 0; i < current_path->s.size(); i++)
        {
            if (current_path->s[i] >= trajectory.s.back())
            {
                break;
            }
            POINT_2D poi = trajectory.calc_postion(current_path->s[i]);
            double di = current_path->d[i];
            double yaw = trajectory.calc_yaw(current_path->s[i]);
            double x = poi[0] - sin(yaw) * di;
            double y = poi[1] + cos(yaw) * di;
            current_path->x.emplace_back(x);
            current_path->y.emplace_back(y);
        }

        for (size_t i = 0; i < current_path->x.size() - 1; i++)
        {
            double dx = current_path->x[i + 1] - current_path->x[i];
            double dy = current_path->y[i + 1] - current_path->y[i];
            current_path->yaw.emplace_back(std::atan2(dy, dx));
            current_path->ds.emplace_back(std::sqrt(dx * dx + dy * dy));
        }
        current_path->yaw.emplace_back(current_path->yaw.back());
        current_path->ds.emplace_back(current_path->ds.back());

        current_path->max_curvature = std::numeric_limits<double>::min();
        for (size_t i = 0; i < current_path->x.size() - 1; i++)
        {
            current_path->curvature.emplace_back((current_path->yaw[i + 1] - current_path->yaw[i]) / current_path->ds[i]);
            if (current_path->curvature.back() > current_path->max_curvature)
            {
                current_path->max_curvature = current_path->curvature.back();
            }
        }
    }
};

bool check_collision(FrenetPath &path, const VEC_2D &obstacle_sum)
{
    for (auto point : obstacle_sum)
    {
        for (size_t i = 0; i < path.x.size(); i++)
        {
            double dist = std::pow((path.x[i] - point[0]), 2) + std::pow((path.y[i] - point[1]), 2);
            if (dist <= ROBOT_RADIUS * ROBOT_RADIUS)
            {
                return false;
            }
        }
    }
    return true;
};

VEC_FrenetPath check_paths(VEC_FrenetPath path_list, const VEC_2D &obstacle_sum)
{
    VEC_FrenetPath fp_list;
    for (FrenetPath path : path_list)
    {
        if (path.max_speed < MAX_SPEED && path.max_accel < MAX_ACCEL && path.max_curvature < MAX_CURVATURE && check_collision(path, obstacle_sum))
        {
            fp_list.emplace_back(path);
        }
    }
    return fp_list;
};

FrenetPath frenet_optimal_planning(Spline2D trajectory, double c_s, double c_speed, double c_accel, double c_d, double c_d_d, double c_d_dd, const VEC_2D &obstacle_sum, double target_s)
{
    if (target_s - c_s >= STOP_DiSTANCE + 0.1)
    {
        vehicle_state = "[CRUISING]";

        VEC_FrenetPath fp_list = calc_frenet_paths_for_cruising(c_speed, c_accel, c_d, c_d_d, c_d_dd, c_s);
        calc_global_paths(fp_list, trajectory);
        VEC_FrenetPath final_path_list = check_paths(fp_list, obstacle_sum);
        double min_cost = std::numeric_limits<double>::max();
        FrenetPath final_path;
        for (auto path : final_path_list)
        {
            if (min_cost >= path.cf)
            {
                min_cost = path.cf;
                final_path = path;
            }
        }
        return final_path;
    }
    else
    {
        vehicle_state = "[STOPPING]";

        VEC_FrenetPath fp_list = calc_frenet_paths_for_stopping(c_speed, c_accel, c_d, c_d_d, c_d_dd, c_s, target_s);
        calc_global_paths(fp_list, trajectory);
        VEC_FrenetPath final_path_list = check_paths(fp_list, obstacle_sum);
        double min_cost = std::numeric_limits<double>::max();
        FrenetPath final_path;
        for (auto path : final_path_list)
        {
            if (min_cost >= path.cf)
            {
                min_cost = path.cf;
                final_path = path;
            }
        }
        return final_path;
    }
};

void signal_handler(int signum)
{
    if (signum == SIGINT)
    {
        exit(1);
    }
}

int main()
{
    Environment env;
    env.generate_global_map();

    CarModel carmodel;

    // VEC_1D WayPoint_x({0.0, 10.0, 20.5, 35.0, 70.5});
    // VEC_1D WayPoint_y({0.0, -6.0, 5.0, 6.5, 0.0});
    // Spline2D Trajectory(WayPoint_x, WayPoint_y);
    // VEC_1D Obstacle_Sum_x({20.0, 30.0, 30.0, 35.0, 50.0});
    // VEC_1D Obstacle_Sum_y({10.0, 6.0, 8.0, 8.0, 3.0});
    // VEC_2D Obstacle_Sum{{{20.0, 10.0}}, {{30.0, 6.0}}, {{30.0, 8.0}}, {{35.0, 8.0}}, {{50.0, 3.0}}};

    Spline2D Trajectory(env.reference_line_x, env.reference_line_y);
    VEC_1D r_x, r_y, r_s, r_yaw, r_curvature;
    for (double i = 0.1; i < Trajectory.s.back(); i += 0.1)
    {
        POINT_2D point = Trajectory.calc_postion(i);
        r_x.emplace_back(point[0]);
        r_y.emplace_back(point[1]);
        r_s.emplace_back(i);
        r_yaw.emplace_back(Trajectory.calc_yaw(i));
        r_curvature.emplace_back(Trajectory.calc_curvature(i));
    }
    VEC_1D Obstacle_Sum_x({50.0, 96.0, 70.0, 40.0, 0.0, 20.0});
    VEC_1D Obstacle_Sum_y({10.0, 25.0, 50.0, 75.0, 96.0, 96.0});
    VEC_2D Obstacle_Sum{{{50.0, 10.0}}, {{96.0, 25.0}}, {{70.0, 50.0}}, {{40.0, 75.0}}, {{0.0, 96.0}}, {{20.0, 96.0}}};

    double c_speed = 0.0 / 3.6, c_accel = 0.0, c_d = 0.0, c_d_d = 0.0, c_d_dd = 0.0, c_s = 0.0;
    double target_s = r_s.back();

    std::cout << "Start" << std::endl;

    signal(SIGINT, signal_handler); // ctrl+c中断

    while (true)
    {
        FrenetPath final_path = frenet_optimal_planning(Trajectory, c_s, c_speed, c_accel, c_d, c_d_d, c_d_dd, Obstacle_Sum, target_s);

        c_s = final_path.s[1];
        c_d = final_path.d[1];
        c_d_d = final_path.d_d[1];
        c_d_dd = final_path.d_dd[1];
        c_speed = final_path.s_d[1];
        c_accel = final_path.s_dd[1];

        plt::cla();
        std::string vehicle_information = vehicle_state + " Speed: " + std::to_string(c_speed * 3.6).substr(0, 4) + "km/h" +
                                          " Accelerate: " + std::to_string(c_accel).substr(0, 4) + "m/ss";
        plt::title(vehicle_information);

        plt::plot(r_x, r_y, "r--");
        plt::plot(env.lower_boundary_x, env.lower_boundary_y, "C7-");
        plt::plot(env.upper_boundary_x, env.upper_boundary_y, "C7-");
        plt::plot(Obstacle_Sum_x, Obstacle_Sum_y, "ok");

        // VEC_1D car_x({final_path.x[0]});
        // VEC_1D car_y({final_path.y[0]});
        // plt::plot(car_x, car_y, "vc");

        carmodel.update(final_path.x[0], final_path.y[0], final_path.yaw[0]);
        plt::plot(carmodel.car_x, carmodel.car_y, "k");
        plt::plot(final_path.x, final_path.y, ".g:");
        plt::axis("equal");

        if (std::pow((final_path.x[1] - r_x.back()), 2) + std::pow((final_path.y[1] - r_y.back()), 2) <= 0.5)
        {
            std::cout << "Goal" << std::endl;
            break;
        }

        plt::pause(0.01);
    }
    std::cout << "c_speed: " << c_speed << " c_accel: " << c_accel << " c_d: " << c_d << std::endl;
    std::cout << "Finish" << std::endl;
    plt::pause(0.01);
    plt::show();

    return 0;
}