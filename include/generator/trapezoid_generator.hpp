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

                    double L_xyz = std::sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
                    double L_a = delta.a;
                    double L_c = delta.c;
                    if (L_xyz < 1e-6 && L_a < 1e-6 && L_c < 1e-6) continue;

                    double v_max_lin = 0.0, a_max_lin = 0.0;
                    if (L_xyz >= 1e-6)
                    {
                        double u[3] = {
                            delta.x / L_xyz,
                            delta.y / L_xyz,
                            delta.z / L_xyz
                        };

                        v_max_lin = machine_limits.max_vel.x / std::abs(u[0]);
                        v_max_lin = std::min(v_max_lin, machine_limits.max_vel.y / std::abs(u[1]));
                        v_max_lin = std::min(v_max_lin, machine_limits.max_vel.z / std::abs(u[2]));

                        a_max_lin = machine_limits.max_acc.x / std::abs(u[0]);
                        a_max_lin = std::min(a_max_lin, machine_limits.max_acc.y / std::abs(u[1]));
                        a_max_lin = std::min(a_max_lin, machine_limits.max_acc.z / std::abs(u[2]));
                    }

                    if (p1.f > 0.0) v_max_lin = std::min(v_max_lin, p1.f);

                    TimeProfile tp_xyz = calculateTimeProfile(L_xyz, v_max_lin, a_max_lin);
                    TimeProfile tp_a = calculateTimeProfile(L_a, machine_limits.max_vel.a, machine_limits.max_acc.a);
                    TimeProfile tp_c = calculateTimeProfile(L_c, machine_limits.max_vel.c, machine_limits.max_acc.c);

                    TimeProfile tp_sync;
                    tp_sync.t_total = std::max({tp_xyz.t_total, tp_a.t_total, tp_c.t_total});
                    tp_sync.t_a = std::max({tp_xyz.t_a, tp_a.t_a, tp_c.t_a});
                    if (tp_sync.t_a * 2.0 > tp_sync.t_total) tp_sync.t_a = tp_sync.t_total / 2.0;
                    tp_sync.t_c = tp_sync.t_total - (2.0 * tp_sync.t_a);

                    double v_peak_lambda = (tp_sync.t_a + tp_sync.t_c > 1e-6) ? (1.0 / (tp_sync.t_a + tp_sync.t_c)) : 0.0;
                    double a_max_lambda  = (tp_sync.t_a > 1e-6) ? (v_peak_lambda / tp_sync.t_a) : 0.0;

                    size_t num_steps = static_cast<size_t>(std::ceil(tp_sync.t_total / dt));
                    for (size_t step = 0; step <= num_steps; ++step)
                    {
                        double t = step * dt;
                        if (t > tp_sync.t_total) t = tp_sync.t_total;

                        double lambda = 0.0;
                        double lambda_dot = 0.0;
                        double lambda_dot_dot = 0.0;

                        if (t < tp_sync.t_a)
                        {
                            lambda_dot_dot = a_max_lambda;
                            lambda_dot = a_max_lambda * t;
                            lambda = 0.5 * a_max_lambda * t * t;
                        }
                        else if (t < tp_sync.t_a + tp_sync.t_c)
                        {
                            double tau = t - tp_sync.t_a;
                            lambda_dot_dot = 0.0;
                            lambda_dot = v_peak_lambda;
                            lambda = (0.5 * a_max_lambda * tp_sync.t_a * tp_sync.t_a) + (v_peak_lambda * tau);
                        }
                        else
                        {
                            double tau = t - (tp_sync.t_a + tp_sync.t_c);
                            lambda_dot_dot = -a_max_lambda;
                            lambda_dot = v_peak_lambda - (a_max_lambda * tau);
                            lambda = (0.5 * a_max_lambda * tp_sync.t_a * tp_sync.t_a) +
                                     (v_peak_lambda * tp_sync.t_c) +
                                     (v_peak_lambda * tau - 0.5 * a_max_lambda * tau * tau);
                        }

                        if (step == num_steps || t >= tp_sync.t_total)
                        {
                            lambda = 1.0;
                            lambda_dot = 0.0;
                            lambda_dot_dot = 0.0;
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