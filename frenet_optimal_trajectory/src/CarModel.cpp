#include "CarModel.h"

void CarModel::update(double x, double y, double yaw)
{
    car_x.resize(0);
    car_y.resize(0);

    Eigen::MatrixXd car(2, 5);
    car << -RB, -RB, RF, RF, -RB,
        W / 2, -W / 2, -W / 2, W / 2, W / 2;

    Eigen::Matrix2d Rot_car;
    Rot_car << cos(yaw), -sin(yaw),
        sin(yaw), cos(yaw);

    car = Rot_car * car;

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            if (i == 0)
            {
                car(i, j) += x;
                car_x.push_back(car(i, j));
            }
            if (i == 1)
            {
                car(i, j) += y;
                car_y.push_back(car(i, j));
            }
        }
    }
}