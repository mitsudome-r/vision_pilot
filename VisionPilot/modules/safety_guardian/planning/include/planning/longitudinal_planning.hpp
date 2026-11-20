#ifndef VISIONPILOT_LONGITUDIANL_PLANNING_HPP
#define VISIONPILOT_LONGITUDIANL_PLANNING_HPP

extern double dt;   // 50Hz control period — still used by planning.cpp

class LongitudinalPlanner {
public:
    struct Config {
        double speed_limit = 33.3;  // m/s
        double a     = 1.5;    // max comfortable acceleration  (m/s²)
        double b     = 3.0;    // comfortable deceleration      (m/s²)
        // Hard bound on the commanded deceleration (m/s², positive). The output of
        // compute_acceleration() is always within [-max_decel, a].
        double max_decel = 5.0;
        double T     = 1.5;    // desired time-headway          (s)
        double s0    = 2.0;    // minimum gap at standstill     (m)
        double delta = 4.0;    // free-road acceleration exponent
        double mu = 0.2;
        double g = 9.81;
    };

    explicit LongitudinalPlanner(const Config& config);

    // Returns the ego acceleration for the current step, bounded to [-max_decel, a].
    // Above the (curvature-limited) speed limit the free-road term follows the IIDM
    // (Treiber & Kesting), i.e. the vehicle slows down with at most the comfortable
    // deceleration b instead of the unbounded -a * (v / v0)^delta of the plain IDM.
    //
    //   ego_v         : ego speed (m/s)
    //   has_cipo      : CIPO in front
    //   cipo_v        : lead-vehicle speed (m/s); set to speed_limit for free road
    //   cipo_distance : bumper-to-bumper gap (m); use 9999.0 for free road
    double compute_acceleration(double kappa, double ego_v, bool has_cipo, double cipo_v, double cipo_distance);

private:
    Config config_;
};

#endif //VISIONPILOT_LONGITUDIANL_PLANNING_HPP
