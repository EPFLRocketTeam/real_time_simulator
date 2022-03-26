#ifndef SRC_ATMOSPHERE_HPP
#define SRC_ATMOSPHERE_HPP

#include <cmath>

namespace atmosphere_models {

constexpr double METER_TO_FT{3.2808399};
constexpr double METER_PER_SECOND_TO_KTS{1.94384449};

namespace random_gen {
class UniformRandGen
{
public:
    UniformRandGen() { distribution = std::uniform_real_distribution<double>(0, 1); }
    double getValue() { return distribution(generator); }
    double getValueBetween(const double &lower_bound, const double &upper_bound)
    {
        return lower_bound + distribution(generator) * (upper_bound - lower_bound);
    }
private:
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution;
};

class WhiteNoiseGen
{
public:
    WhiteNoiseGen() { distribution = std::normal_distribution<double>(0, 1); }
    double getValue() { return distribution(generator); }
    double getValueBetween(double lower, double upper)
    { // Returns samples from the normal distribution, s.t. lower/higher match with -/+ 2 * standard error
        const double mean = 0.5 * (lower + upper);
        return mean + distribution(generator) * (upper - mean) / 2.0;
    }
private:
    std::default_random_engine generator;
    std::normal_distribution<double> distribution;
};
}

class HorizontalWindModel
{
public:
    void setup(double sim_dt, double mean_windFrom_deg, double mean_windspeed)
    {
        psi = psi_mean = mean_windFrom_deg * M_PI / 180.0;
        V_mean = mean_windspeed;

        dpsi_max = sim_dt * 5.0 * M_PI / 180.0; // Max change of wind direction: 2.0 deg/s
    }
    double get_direction()
    {
        update();
        return psi;
    }
    void get_windVelocityNED(Eigen::Vector3d &n_vW)
    {
        update();
        n_vW = {-V_mean * cos(psi), -V_mean * sin(psi), 0};
    }
private:
    double psi_mean{0};
    double V_mean{0};
    double psi{psi_mean};
    random_gen::WhiteNoiseGen randGen;
    double dpsi_max{0};
    void update()
    {
        //double psi_err = psi_mean - psi;
        //if (psi_err > M_PI) psi_err -= 2 * M_PI;
        //if (psi_err < -M_PI) psi_err += 2 * M_PI;

        const double dpsi = randGen.getValueBetween(-dpsi_max, dpsi_max);
        psi += dpsi;
        //std::cout << "psim: " << psi_mean << ", psi: " << psi << ", dpsi:" << dpsi << "\n";
    }
};

class WindShearModel
{
public:
    void setup(double windspeed_at_6m)
    {
        W20ft = windspeed_at_6m;
    }
    double get_windspeed(double height)
    {
        if (height < 3.0) return W20ft;
        return W20ft * log(height * METER_TO_FT / 0.15) / log20;
    }
private:
    const double log20{log(20.0/ 0.15)};
    double W20ft{5};
};

namespace turbulence_models {

namespace discrete {
class DiscreteGustModel
{
public:
    void setup(double dt_, double windFrom_deg, double mean_windspeed)
    {
        dt = dt_;
        windFrom = windFrom_deg * M_PI / 180.0;
        gust_amplitude_max = 0.7 * mean_windspeed;

        gust_duration_max =  (8 - 1.5 * mean_windspeed) + 1; // @0m/s: 9s max, @5./s: 1s max
        gust_duration_max = std::min(std::max(gust_duration_max, 1.0), 9.0); // keep between 1 and 10s
    }

    void getGustVelNED(Eigen::Vector3d &n_vGust)
    {
        if (gust_isActive)
        {
            update_gust();
        }
        else if (gust_isScheduled)
        { // Check if gust gets active
            if (t_now >= t_gust_in)
            {
                gust_isActive = true;
                update_gust();
            }
        }
        else
        { // Schedule next gust
            schedule_gust();
            gust_isScheduled = true;
        }

        n_vGust = {-Vgust * cos(windFrom), -Vgust * sin(windFrom), 0};

        t_now += dt;
    }
private:
    double dt{0.1}, t_now{0}; // Absolute time
    double windFrom{0};

    double gust_duration_max{5};

    bool gust_isActive{false};
    bool gust_isScheduled{false};
//    random_gen::UniformRandGen randGen;
    random_gen::WhiteNoiseGen randGen;
    double t_gust_in{1}; // Absolute time of next gust
    double duration_in{1}, duration_out{1}; // Durations
    double tau_gust_main{1}, tau_gust_out{2}, tau_gust_end{3}; // Relative time from begin of gust
    double gust_amplitude{1}, gust_amplitude_max{2};

    double Vgust{0};

    void schedule_gust()
    {
        const double duration_total = randGen.getValueBetween(1.0 * gust_duration_max/5.0, 5.0 * gust_duration_max/5.0);
        const double duration_main = 0.3 * duration_total;
        duration_in = 0.35 * duration_total;
        duration_out = 0.35 * duration_total;

        t_gust_in = t_now + randGen.getValueBetween(0.5, 4.0);
        tau_gust_main = duration_in;
        tau_gust_out = tau_gust_main + duration_main;
        tau_gust_end = tau_gust_out + duration_out;

        gust_amplitude = randGen.getValueBetween(0.5 * gust_amplitude_max, gust_amplitude_max);
        //std::cout << "Scheduled gust. Duration total: " << duration_total << ", start in: " << t_gust_in - t_now << "\n";
    }
    void update_gust()
    {
        Vgust = gust_amplitude * get_gust_profile(t_now - t_gust_in);
    }
    double get_gust_profile(double age_gust)
    {
        if (age_gust <= tau_gust_main)
            return 0.5 * (1 - cos(M_PI * age_gust / duration_in));
        else if (age_gust <= tau_gust_out)
            return 1;
        else if (age_gust <= tau_gust_end)
            return 1 - 0.5 * (1 - cos(M_PI * (age_gust - tau_gust_out) / duration_out));
        else
        {
            t_now = 0;
            gust_isActive = false;
            gust_isScheduled = false;
            return 0;
        }
    }

};
}

namespace psd {
// Power Spectral Density / Frequency Space models
class DrydenTurbulenceModel
{
private:
    template<int nx, int nu, int ny>
    class StateSpaceModel
    {
    public:
        explicit StateSpaceModel(double dt_) : dt(dt_)
        {
            x.setZero();
            A.setZero();
            B.setZero();
            C.setZero();
            D.setZero();

            Ad.setZero();
            Bd.setZero();
            Identity.setIdentity();
        }
        void set_dt(double dt_) { dt = dt_; }
        double dt{0.01};
        Eigen::Matrix<double, nx, nx> A;
        Eigen::Matrix<double, nx, nu> B;
        Eigen::Matrix<double, ny, nx> C;
        Eigen::Matrix<double, ny, nu> D;
        void discretize()
        {
            Ad = Identity + A * dt;
            Bd = B * dt;
        }
        void run(const Eigen::Matrix<double, nu, 1> &u, Eigen::Matrix<double, ny, 1> &y)
        {
//        std::cout << "Ad " << Ad << ", Bd " << Bd << ", C " << C << ", D " << D << "\n";
//        std::cout << "x " << x << ", u " << u << "\n";
            x = Ad * x + Bd * u;
            y = C * x + D * u;
//        std::cout << "x+ " << x << ", y " << y << "\n";
        }
    private:
        Eigen::Matrix<double, nx, 1> x;
        Eigen::Matrix<double, nx, nx> Identity;
        Eigen::Matrix<double, nx, nx> Ad;
        Eigen::Matrix<double, nx, nu> Bd;
    };

    template<typename Scalar, typename Derived>
    Eigen::Quaternion<Scalar> T1quat(const Derived &rotAng)
    {
        Eigen::Quaternion<Scalar> q;
        q = Eigen::AngleAxis<Scalar>(-rotAng, Eigen::Matrix<Scalar, 3, 1>(1, 0, 0));
        return q;
    }
    template<typename Scalar, typename Derived>
    Eigen::Quaternion<Scalar> T2quat(const Derived &rotAng)
    {
        Eigen::Quaternion<Scalar> q;
        q = Eigen::AngleAxis<Scalar>(-rotAng, Eigen::Matrix<Scalar, 3, 1>(0, 1, 0));
        return q;
    }
    template<typename Scalar, typename Derived>
    Eigen::Quaternion<Scalar> T3quat(const Derived &rotAng)
    {
        Eigen::Quaternion<Scalar> q;
        q = Eigen::AngleAxis<Scalar>(-rotAng, Eigen::Matrix<Scalar, 3, 1>(0, 0, 1));
        return q;
    }

public:
    explicit DrydenTurbulenceModel(double dt = 0.01) { setup(dt); }
    DrydenTurbulenceModel(double dt, double wingspan, double windspeed_6m = 5)
    {
        setup(dt, windspeed_6m, wingspan);
    }
    ~DrydenTurbulenceModel() = default;

    void setup(double dt, double windspeed_6m = 5, double wingspan = 2)
    {
        W20 = windspeed_6m * METER_PER_SECOND_TO_KTS;
        sigma_w = 0.1 * W20;
        set_wingspan(wingspan);

        u_ss.dt = v_ss.dt = w_ss.dt = dt;
        p_ss.dt = q_ss.dt = r_ss.dt = dt;

        u_ss.C << 1;
        v_ss.C << 1, 0;
        w_ss.C << 1, 0;
        p_ss.C << 1;
        q_ss.C << 1, 0, 0;
        r_ss.C << 1, 0, 0;
    }
    void set_wingspan(double wingspan_m) { b = wingspan_m * METER_TO_FT; }

    void run(double height, double Va)
    {
        /* height in m, Va in m/s --> convert to ft and ft/s */
        const double h = std::max(height * METER_TO_FT, 10.0);
        const double V = Va * METER_TO_FT;

        const double
                L_w = 0.5 * h,
                L_u = h / pow(0.177 + 0.000823 * h, 1.2),
                L_v = 0.5 * h / pow(0.177 + 0.000823 * h, 1.2),
                sigma_u = sigma_w / pow(0.177 + 0.000823 * h, 0.4),
                sigma_v = sigma_w / pow(0.177 + 0.000823 * h, 0.4);

        /* Random number (white noise) */
        const Eigen::Matrix<double, 1, 1> white_noise{whitenNoiseGen.getValue()};

        /* u ------------------------------------------------------------ */
        const double
                au1 = V / L_u,
                betau = sigma_u * sqrt(2.0 * L_u / (M_PI * V)) * au1,
                bu1 = betau;
        u_ss.A << -au1;
        u_ss.B << bu1;
        u_ss.discretize();
        u_ss.run(white_noise, u);

        /* v ------------------------------------------------------------ */
        const double
                av1 = 2.0 * V / L_v,
                av2 = (V / L_v) * (V / L_v),
                betav = sigma_v * sqrt(L_v / (M_PI * V)) * av2,
                bv1 = sqrt(3) * L_v / V * betav,
                bv2 = betav;
        v_ss.A << -av1, 1,
                -av2, 0;
        v_ss.B << bv1, bv2;
        v_ss.discretize();
        v_ss.run(white_noise, v);

        /* w ------------------------------------------------------------ */
        const double
                aw1 = 2.0 * V / L_w,
                aw2 = (V / L_w) * (V / L_w),
                betaw = sigma_w * sqrt(L_w / (M_PI * V)) * aw2,
                bw1 = sqrt(3) * L_w / V * betaw,
                bw2 = betaw;
        w_ss.A << -aw1, 1,
                -aw2, 0;
        w_ss.B << bw1, bw2;
        w_ss.discretize();
        w_ss.run(white_noise, w);

        /* p ------------------------------------------------------------ */
        const double
                pi_4b = M_PI / (4.0 * b),
                ap1 = V * pi_4b,
                betap = sigma_w * sqrt(0.8 / b) / pow(L_w, 1.0 / 3.0) * pow(pi_4b, 1.0 / 6.0) * ap1,
                bp1 = betap;
        p_ss.A << -ap1;
        p_ss.B << bp1;
        p_ss.discretize();
        p_ss.run(white_noise, p);

        /* q ------------------------------------------------------------ */
        const double
                L_w2 = L_w * L_w,
                V2 = V * V,
                aq1 = (M_PI * L_w * V + b * V * 8.0) / 4.0 / b / L_w,
                aq2 = (2.0 * M_PI * L_w * V2 + 4 * b * V2) / 4 / b / L_w2,
                aq3 = M_PI * V2 * V / 4 / b / L_w2,
                betaq = sigma_w * V * sqrt(M_PI * L_w / V) / 4 / b / L_w2,
                bq1 = sqrt(3) * L_w * betaq,
                bq2 = V * betaq;
        q_ss.A << -aq1, 1, 0,
                -aq2, 0, 1,
                -aq3, 0, 0;
        q_ss.B << 0, bq1, bq2;
        q_ss.discretize();
        q_ss.run(white_noise, q);

        /* r ------------------------------------------------------------ */
        const double
                L_v2 = L_v * L_v,
                _3_bLv2 = 3.0 / (b * L_v2),
                ar1 = (M_PI * L_v * V + b * V * 6.0) / 3.0 / b / L_v,
                ar2 = (2.0 * M_PI * L_v * V2 + 3.0 * b * V2) / _3_bLv2,
                ar3 = M_PI * V2 * V / _3_bLv2,
                betar = sigma_v * V * sqrt(M_PI * L_v / V) / _3_bLv2,
                br1 = sqrt(3) * L_v * betar,
                br2 = V * betar;
        r_ss.A << -ar1, 1, 0,
                -ar2, 0, 1,
                -ar3, 0, 0;
        r_ss.B << 0, br1, br2;
        r_ss.run(white_noise, r);
    }
    void get_turbulence_ned(const std::vector<double> &q_nb_vec, double alpha, double beta,
                            Eigen::Vector3d &n_vTurb,
                            Eigen::Vector3d &b_wTurb)
    {
        const Eigen::Quaternion<double> q_nb(q_nb_vec.data());
        const Eigen::Quaternion<double> q_ba = T2quat<double>(alpha) * T3quat<double>(-beta);
        n_vTurb = q_nb * q_ba * Eigen::Vector3d(u(0), v(0), w(0));
        b_wTurb = q_ba * Eigen::Vector3d(p(0), q(0), r(0));               // Turb. angular rates are in body frame
    }
private:
    double b{6};    // wingspan, default 6ft (2m)
    double W20{15}; // wind speed at 20ft (6m), default 15kts (light turbulence)
    double sigma_w{0.1 * W20};

    random_gen::WhiteNoiseGen whitenNoiseGen{};

    StateSpaceModel<1, 1, 1> u_ss{0.01};
    StateSpaceModel<2, 1, 1> v_ss{0.01};
    StateSpaceModel<2, 1, 1> w_ss{0.01};
    StateSpaceModel<1, 1, 1> p_ss{0.01};
    StateSpaceModel<3, 1, 1> q_ss{0.01};
    StateSpaceModel<3, 1, 1> r_ss{0.01};
    Eigen::Matrix<double, 1, 1> u{0}, v{0}, w{0}, p{0}, q{0}, r{0};
};
}

}

}

#endif //SRC_ATMOSPHERE_HPP
