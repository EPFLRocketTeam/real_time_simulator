/*
* Code from goromal, at https://github.com/goromal/wind-dynamics 
*/

#pragma once

#include <random>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>

using namespace Eigen;

namespace dryden_model {

class GustModelBase
{
private:
    std::default_random_engine random_generator_;
    std::uniform_real_distribution<double> gust_dist_;
    
    bool initialized_;
    double dt_;

    double alpha_;
    double beta_;
    double delta_;
    double gamma_;

    double u_km1;
    double u_km2;
    double y_km1;
    double y_km2;

    double run(const double &dt)
    {
        if (initialized_)
        {
            double C1 = 1.0 + 2 * delta_ / dt + 4 * gamma_ / dt / dt;
            double C2 = 2.0 - 8 * gamma_ / dt / dt;
            double C3 = 1.0 - 2 * delta_ / dt + 4 * gamma_ / dt / dt;
            double C4 = alpha_ + 2 * beta_ / dt;
            double C5 = 2 * alpha_;
            double C6 = alpha_ - 2 * beta_ / dt;

            double u_k = gust_dist_(random_generator_);
            double y_k = (C4 * u_k + C5 * u_km1 + C6 * u_km2 - C2 * y_km1 - C3 * y_km2) / C1;

            u_km2 = u_km1;
            u_km1 = u_k;
            y_km2 = y_km1;
            y_km1 = y_k;

            return y_k;
        }
        else
            return 0.0;
    }

public:
    GustModelBase() : dt_(0.05)
    {
        random_generator_ = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
        gust_dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);
        initialized_ = false;
    }

    void initializeParameters(const double &V, 
                              const double &L, 
                              const double &sigma)
    {
        double b = 2 * sqrt(3) * L / V;
        double c = 2 * L / V;

        alpha_ = sigma * sqrt(2 * L / M_PI / V);
        beta_ = alpha_ * b;
        delta_ = 2 * c;
        gamma_ = c * c;
        
        u_km1 = 0;
        u_km2 = 0;
        y_km1 = 0;
        y_km2 = 0;

        initialized_ = true;
    }

    double integrate(const double &dt)
    {
        if (dt > dt_)
        {
            double t = 0;
            double y = 0;
            while (t < dt)
            {
                double t_inc = std::min(dt_, dt - t);
                y = run(t_inc);
                t += t_inc;
            }
            return y;
        }
        else
        {
            return run(dt);
        }
    }
};

class DrydenWind
{
public:
    DrydenWind() : initialized_(false) {}

    void initialize(const double &wx_nominal, const double &wy_nominal, const double &wz_nominal,
                    const double &wx_sigma,   const double &wy_sigma,   const double &wz_sigma,
                    const double &altitude=2.0)
    {
        wx_nominal_ = wx_nominal;
        wy_nominal_ = wy_nominal;
        wz_nominal_ = wz_nominal;

        double Lz_ft = 3.281 * altitude;
        double Lx_ft = Lz_ft / pow(0.177 + 0.000823 * Lz_ft, 1.2);
        double Ly_ft = Lx_ft;

        wx_gust_.initializeParameters(1.0, Lx_ft / 3.281, wx_sigma);
        wy_gust_.initializeParameters(1.0, Ly_ft / 3.281, wy_sigma);
        wz_gust_.initializeParameters(1.0, Lz_ft / 3.281, wz_sigma);

        initialized_ = true;
    }

    Vector3d getWind(const double &dt)
    {
        if (initialized_)
            return Vector3d(wx_nominal_, wy_nominal_, wz_nominal_) 
                 + Vector3d(wx_gust_.integrate(dt), wy_gust_.integrate(dt), wz_gust_.integrate(dt));
        else
            return Vector3d(0., 0., 0.);
    }

private:
    GustModelBase wx_gust_;
    GustModelBase wy_gust_;
    GustModelBase wz_gust_;

    double wx_nominal_;
    double wy_nominal_;
    double wz_nominal_;

    bool initialized_;
};

} // end namespace dryden_model
