#pragma once

#include <algorithm>
#include <cmath>

#include "generator.hpp"

namespace em
{
    class TrapezoidalGenerator : public Generator
    {
        private:

            struct TimeProfile
            {
                double t_a = 0.0;
                double t_c = 0.0;
                double t_total = 0.0;
            };

            static TimeProfile calculateTimeProfile(double L, double v_max, double a_max)
            {
                TimeProfile time_profile;
                if (L < 1e-6) return time_profile;

                double L_crit = (v_max * v_max) / a_max;
                if (L >= L_crit)
                {
                    time_profile.t_a = v_max / a_max;
                    time_profile.t_c = (L - L_crit) / v_max;
                } else
                {
                    double v_peak = std::sqrt(L * a_max);
                    time_profile.t_a = v_peak / a_max;
                    time_profile.t_c = 0.0;
                }

                time_profile.t_total = 2 * time_profile.t_a + time_profile.t_c;
                return time_profile;
            }

        public:

            TrapezoidalGenerator(const size_t& frequency, const MachineLimits& machine_limits) :
            Generator(frequency, machine_limits)
            {}

            void generate(const std::vector<MachineState>& control_states) override
            {
                trajectory.clear();
                if (control_states.size() < 2) return;

                for (size_t k = 0; k < control_states.size() - 1; ++k)
                {
                    const MachineState& p0 = control_states[k];
                    const MachineState& p1 = control_states[k + 1];

                    GeometryVector delta = {
                        p1.x - p0.x,
                        p1.y - p0.y,
                        p1.z - p0.z,
                        p1.a - p0.a,
                        p1.c - p0.c
                    };

                    GeometryVector abs_delta = {
                        std::abs(delta.x),
                        std::abs(delta.y),
                        std::abs(delta.z),
                        std::abs(delta.a),
                        std::abs(delta.c)
                    };

                    double lambda_max_vel = 1e9;
                    double lambda_max_acc = 1e9;
                    if (abs_delta.x > 1e-6)
                    {
                        lambda_max_vel = std::min(lambda_max_vel, machine_limits.max_vel.x / abs_delta.x);
                        lambda_max_acc = std::min(lambda_max_acc, machine_limits.max_acc.x / abs_delta.x);
                    }
                    if (abs_delta.y > 1e-6)
                    {
                        lambda_max_vel = std::min(lambda_max_vel, machine_limits.max_vel.y / abs_delta.y);
                        lambda_max_acc = std::min(lambda_max_acc, machine_limits.max_acc.y / abs_delta.y);
                    }
                    if (abs_delta.z > 1e-6)
                    {
                        lambda_max_vel = std::min(lambda_max_vel, machine_limits.max_vel.z / abs_delta.z);
                        lambda_max_acc = std::min(lambda_max_acc, machine_limits.max_acc.z / abs_delta.z);
                    }
                    if (abs_delta.a > 1e-6)
                    {
                        lambda_max_vel = std::min(lambda_max_vel, machine_limits.max_vel.a / abs_delta.a);
                        lambda_max_acc = std::min(lambda_max_acc, machine_limits.max_acc.a / abs_delta.a);
                    }
                    if (abs_delta.c > 1e-6)
                    {
                        lambda_max_vel = std::min(lambda_max_vel, machine_limits.max_vel.c / abs_delta.c);
                        lambda_max_acc = std::min(lambda_max_acc, machine_limits.max_acc.c / abs_delta.c);
                    }

                    double L_xyz = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
                    if ((p1.f > 0.0) && (L_xyz > 1e-6))
                    {
                        lambda_max_vel = std::min(lambda_max_vel, p1.f / L_xyz);
                    }

                    TimeProfile tp_sync = calculateTimeProfile(1.0, lambda_max_vel, lambda_max_acc);
                    if (tp_sync.t_total < 1e-6) continue;

                    double lambda_vel = (tp_sync.t_a + tp_sync.t_c > 1e-9) ? (1.0 / (tp_sync.t_a + tp_sync.t_c)) : 0.0;
                    double lambda_acc = (tp_sync.t_a > 1e-9) ? (lambda_vel / tp_sync.t_a) : 0.0;

                    double phase_t[4] = {
                        0.0,
                        tp_sync.t_a,
                        tp_sync.t_a + tp_sync.t_c,
                        2.0 * tp_sync.t_a + tp_sync.t_c
                    };

                    double phase_acc[3] = {
                        lambda_acc,
                        0.0,
                        -lambda_acc
                    };

                    size_t num_steps = static_cast<size_t>(std::ceil(tp_sync.t_total / dt));
                    for (size_t step = 0; step <= num_steps; ++step)
                    {
                        double t = step * dt;
                        if (t > phase_t[3]) t = phase_t[3];

                        int phase = 0;
                        while (phase < 2 && t > phase_t[phase + 1])
                        {
                            phase++;
                        }

                        double s_0 = 0.0, v_0 = 0.0;
                        for (int p = 0; p < phase; ++p)
                        {
                            double dt_p = phase_t[p + 1] - phase_t[p];
                            s_0 += v_0 * dt_p + 0.5 * phase_acc[p] * dt_p * dt_p;
                            v_0 += phase_acc[p] * dt_p;
                        }

                        double tau = t - phase_t[phase];
                        double lambda_dot_dot = phase_acc[phase];
                        double lambda_dot = v_0 + phase_acc[phase] * tau;
                        double lambda = s_0 + v_0 * tau + 0.5 * phase_acc[phase] * tau * tau;

                        if (step == num_steps || t >= phase_t[3])
                        {
                            lambda = 1.0; lambda_dot = 0.0; lambda_dot_dot = 0.0;
                        }

                        TrajectoryPoint pt;
                        pt.t = trajectory.empty() ? 0.0 : trajectory.back().t + dt;
                        
                        pt.pos.x = p0.x + delta.x * lambda;
                        pt.pos.y = p0.y + delta.y * lambda;
                        pt.pos.z = p0.z + delta.z * lambda;
                        pt.pos.a = p0.a + delta.a * lambda;
                        pt.pos.c = p0.c + delta.c * lambda;

                        pt.vel.x = delta.x * lambda_dot;
                        pt.vel.y = delta.y * lambda_dot;
                        pt.vel.z = delta.z * lambda_dot;
                        pt.vel.a = delta.a * lambda_dot;
                        pt.vel.c = delta.c * lambda_dot;

                        pt.acc.x = delta.x * lambda_dot_dot;
                        pt.acc.y = delta.y * lambda_dot_dot;
                        pt.acc.z = delta.z * lambda_dot_dot;
                        pt.acc.a = delta.a * lambda_dot_dot;
                        pt.acc.c = delta.c * lambda_dot_dot;

                        trajectory.push_back(pt);
                    }
                }
            }
    };
}