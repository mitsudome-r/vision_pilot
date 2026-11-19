#ifndef VISIONPILOT_LATERAL_HPP
#define VISIONPILOT_LATERAL_HPP

#include <vector>
#include <Eigen/Core>

// ── Horizon parameters ───────────────────────────────────────────────────────
extern size_t N;

class LateralPlanner {
public:
    struct Config {
        // Physical tyre-angle limit used as the MPC bound (rad).
        // Default (~23 deg) matches the original passenger-car tuning.
        double max_steer_angle     = 0.442;
        // Understeer gradient K_us (rad / (m/s^2)) used in the steering feed-forward.
        // Passenger cars ~0.001-0.003; heavy trucks are typically larger.
        double understeer_gradient = 0.0015;
    };

    LateralPlanner();
    explicit LateralPlanner(const Config& config);
    ~LateralPlanner();

    // Solve the MPC for the steering sequence.
    //
    //   state          = [cte, epsi, kappa_road]   (initial conditions)
    //   v_schedule     = predicted speed at each horizon step      (N elements)
    //   kappa_schedule = predicted road curvature at each horizon  (N elements)
    //                    step.  Built by the caller from the current curvature
    //                    plus a *clamped* linear preview, so it can never
    //                    exceed the steering-achievable curvature.  This is
    //                    passed in (like v_schedule) rather than reconstructed
    //                    inside the MPC, which keeps the optimiser's
    //                    constraints smooth.
    //
    // Returns [delta_0, delta_0, delta_1, ..., delta_{N-2}]
    std::vector<double> compute_steering(double L,
                                        const Eigen::VectorXd& state,
                                        const Eigen::VectorXd& v_schedule,
                                        const Eigen::VectorXd& kappa_schedule);

    const Config& config() const { return config_; }

private:
    Config config_;
};

#endif //VISIONPILOT_LATERAL_HPP
