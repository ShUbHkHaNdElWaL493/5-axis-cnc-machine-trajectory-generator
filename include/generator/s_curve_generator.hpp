#pragma once

#include <algorithm>
#include <cmath>

#include "generator.hpp"

namespace em
{
    class SCurveGenerator : public Generator
    {
        private:

            struct TimeProfile
            {
                double t_j = 0.0;
                double t_a = 0.0;
                double t_v = 0.0;

                double peak_vel = 0.0;
                double peak_acc = 0.0;

                double t_total = 0.0;
            };

            static TimeProfile calculateTimeProfile(double L, double v_max, double a_max, double j_max)
            {
                TimeProfile time_profile;
                if (L <= 1e-6) return time_profile;

                double v_a = (a_max * a_max) / j_max; 
                double Tj_calc, Ta_calc;
                if (v_max < v_a) 
                {
                    Tj_calc = std::sqrt(v_max / j_max);
                    Ta_calc = 0.0;
                } 
                else 
                {
                    Tj_calc = a_max / j_max;
                    Ta_calc = (v_max - v_a) / a_max;
                }

                double L_crit = v_max * (Ta_calc + 2.0 * Tj_calc); 
                if (L >= L_crit) 
                {
                    time_profile.t_j = Tj_calc;
                    time_profile.t_a = Ta_calc;
                    time_profile.t_v = (L - L_crit) / v_max;
                    time_profile.peak_vel = v_max;
                    time_profile.peak_acc = (Ta_calc > 0.0) ? a_max : (j_max * Tj_calc);
                } 
                else 
                {
                    time_profile.t_v = 0.0;

                    double L_a = 2.0 * (a_max * a_max * a_max) / (j_max * j_max);
                    if (L >= L_a) 
                    {
                        time_profile.t_j = a_max / j_max;
                        time_profile.peak_acc = a_max;

                        double b = 3.0 * time_profile.t_j;
                        double c = 2.0 * (time_profile.t_j * time_profile.t_j) - (L / a_max);
                        
                        time_profile.t_a = (-b + std::sqrt(b * b - 4.0 * c)) / 2.0;
                        time_profile.peak_vel = a_max * (time_profile.t_j + time_profile.t_a);
                    } 
                    else 
                    {
                        time_profile.t_a = 0.0;
                        time_profile.t_j = std::cbrt(L / (2.0 * j_max));
                        time_profile.peak_acc = j_max * time_profile.t_j;
                        time_profile.peak_vel = j_max * (time_profile.t_j * time_profile.t_j);
                    }
                }

                time_profile.t_total = 4.0 * time_profile.t_j + 2.0 * time_profile.t_a + time_profile.t_v;
                return time_profile;
            }

        public:

            SCurveGenerator(const size_t& frequency, const MachineLimits& machine_limits) :
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
                    double lambda_max_jrk = 1e9;
                    if (abs_delta.x > 1e-6)
                    {
                        lambda_max_vel = std::min(lambda_max_vel, machine_limits.max_vel.x / abs_delta.x);
                        lambda_max_acc = std::min(lambda_max_acc, machine_limits.max_acc.x / abs_delta.x);
                        lambda_max_jrk = std::min(lambda_max_jrk, machine_limits.max_jrk.x / abs_delta.x);
                    }
                    if (abs_delta.y > 1e-6)
                    {
                        lambda_max_vel = std::min(lambda_max_vel, machine_limits.max_vel.y / abs_delta.y);
                        lambda_max_acc = std::min(lambda_max_acc, machine_limits.max_acc.y / abs_delta.y);
                        lambda_max_jrk = std::min(lambda_max_jrk, machine_limits.max_jrk.y / abs_delta.y);
                    }
                    if (abs_delta.z > 1e-6)
                    {
                        lambda_max_vel = std::min(lambda_max_vel, machine_limits.max_vel.z / abs_delta.z);
                        lambda_max_acc = std::min(lambda_max_acc, machine_limits.max_acc.z / abs_delta.z);
                        lambda_max_jrk = std::min(lambda_max_jrk, machine_limits.max_jrk.z / abs_delta.z);
                    }
                    if (abs_delta.a > 1e-6)
                    {
                        lambda_max_vel = std::min(lambda_max_vel, machine_limits.max_vel.a / abs_delta.a);
                        lambda_max_acc = std::min(lambda_max_acc, machine_limits.max_acc.a / abs_delta.a);
                        lambda_max_jrk = std::min(lambda_max_jrk, machine_limits.max_jrk.a / abs_delta.a);
                    }
                    if (abs_delta.c > 1e-6)
                    {
                        lambda_max_vel = std::min(lambda_max_vel, machine_limits.max_vel.c / abs_delta.c);
                        lambda_max_acc = std::min(lambda_max_acc, machine_limits.max_acc.c / abs_delta.c);
                        lambda_max_jrk = std::min(lambda_max_jrk, machine_limits.max_jrk.c / abs_delta.c);
                    }

                    double L_xyz = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
                    if ((p1.f > 0.0) && (L_xyz > 1e-6))
                    {
                        lambda_max_vel = std::min(lambda_max_vel, p1.f / L_xyz);
                    }

                    TimeProfile tp_sync = calculateTimeProfile(1.0, lambda_max_vel, lambda_max_acc, lambda_max_jrk);
                    if (tp_sync.t_total < 1e-6) continue;

                    double lambda_jrk = (tp_sync.t_j > 1e-9) ? (tp_sync.peak_acc / tp_sync.t_j) : 0.0;

                    double phase_t[8] = {
                        0.0,
                        tp_sync.t_j,
                        tp_sync.t_j + tp_sync.t_a,
                        2.0 * tp_sync.t_j + tp_sync.t_a,
                        2.0 * tp_sync.t_j + tp_sync.t_a + tp_sync.t_v,
                        3.0 * tp_sync.t_j + tp_sync.t_a + tp_sync.t_v,
                        3.0 * tp_sync.t_j + 2.0 * tp_sync.t_a + tp_sync.t_v,
                        4.0 * tp_sync.t_j + 2.0 * tp_sync.t_a + tp_sync.t_v
                    };

                    double phase_jrk[7] = {
                        lambda_jrk,
                        0.0,
                        -lambda_jrk,
                        0.0,
                        -lambda_jrk,
                        0.0,
                        lambda_jrk
                    };

                    size_t num_steps = static_cast<size_t>(std::ceil(tp_sync.t_total / dt));
                    for (size_t step = 0; step <= num_steps; ++step)
                    {
                        double t = step * dt;
                        if (t > phase_t[7]) t = phase_t[7];

                        int phase = 0;
                        while (phase < 6 && t > phase_t[phase + 1])
                        {
                            phase++;
                        }

                        double s_0 = 0.0, v_0 = 0.0, a_0 = 0.0;
                        for (int p = 0; p < phase; ++p)
                        {
                            double dt_p = phase_t[p + 1] - phase_t[p];
                            s_0 += v_0 * dt_p + 0.5 * a_0 * dt_p * dt_p + (1.0/6.0) * phase_jrk[p] * dt_p * dt_p * dt_p;
                            v_0 += a_0 * dt_p + 0.5 * phase_jrk[p] * dt_p * dt_p;
                            a_0 += phase_jrk[p] * dt_p;
                        }

                        double tau = t - phase_t[phase];
                        double lambda_dot_dot = a_0 + phase_jrk[phase] * tau;
                        double lambda_dot = v_0 + a_0 * tau + 0.5 * phase_jrk[phase] * tau * tau;
                        double lambda = s_0 + v_0 * tau + 0.5 * a_0 * tau * tau + (1.0/6.0) * phase_jrk[phase] * tau * tau * tau;

                        if (step == num_steps || t >= phase_t[7])
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