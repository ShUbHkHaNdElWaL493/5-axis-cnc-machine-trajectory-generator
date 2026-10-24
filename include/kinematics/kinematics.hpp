#pragma once

#include <cmath>
#include <stdexcept>
#include "types.hpp"

namespace em
{
    class Kinematics
    {
        private:

            static constexpr double DEG_TO_RAD = M_PI / 180.0;

            MachineLimits machine_limits;
            double pivot_offset;
            double setup_offset;

            double getPivotOffsetZ() const { return pivot_offset + setup_offset; }

            void checkMachineLimits(const GeometryVector& m, const double epsilon = 0.001) const
            {
                if (m.x < machine_limits.min_pos.x - epsilon || m.x > machine_limits.max_pos.x + epsilon)
                    throw std::out_of_range("[ERROR] X-axis overtravel (" + std::to_string(m.x) + ")");
                if (m.y < machine_limits.min_pos.y - epsilon || m.y > machine_limits.max_pos.y + epsilon)
                    throw std::out_of_range("[ERROR] Y-axis overtravel (" + std::to_string(m.y) + ")");
                if (m.z < machine_limits.min_pos.z - epsilon || m.z > machine_limits.max_pos.z + epsilon)
                    throw std::out_of_range("[ERROR] Z-axis overtravel (" + std::to_string(m.z) + ")");
                if (m.a < machine_limits.min_pos.a - epsilon || m.a > machine_limits.max_pos.a + epsilon)
                    throw std::out_of_range("[ERROR] A-axis overtravel (" + std::to_string(m.a) + ")");
                if (m.c < machine_limits.min_pos.c - epsilon || m.c > machine_limits.max_pos.c + epsilon)
                    throw std::out_of_range("[ERROR] C-axis overtravel (" + std::to_string(m.c) + ")");
            }

        public:

            Kinematics(MachineLimits machine_limits, double pivot_offset, double setup_offset) :
            machine_limits(machine_limits),
            pivot_offset(pivot_offset),
            setup_offset(setup_offset)
            {}

            GeometryVector calculateFK(const GeometryVector& machine_state) const
            {
                GeometryVector tool_state = machine_state;

                double rad_inv_A = -machine_state.a * DEG_TO_RAD;
                double rad_inv_C = -machine_state.c * DEG_TO_RAD;
                double offset_Z = getPivotOffsetZ();

                double x_m = machine_state.x;
                double y_m = machine_state.y;
                double z_m = machine_state.z + offset_Z;

                double x_inv_a = x_m;
                double y_inv_a = y_m * std::cos(rad_inv_A) - z_m * std::sin(rad_inv_A);
                double z_inv_a = y_m * std::sin(rad_inv_A) + z_m * std::cos(rad_inv_A);

                z_inv_a -= offset_Z;

                tool_state.x = x_inv_a * std::cos(rad_inv_C) - y_inv_a * std::sin(rad_inv_C);
                tool_state.y = x_inv_a * std::sin(rad_inv_C) + y_inv_a * std::cos(rad_inv_C);
                tool_state.z = z_inv_a;

                return tool_state;
            }

            GeometryVector calculateIK(const GeometryVector& tool_state) const
            {
                GeometryVector machine_state = tool_state;

                double rad_A = tool_state.a * DEG_TO_RAD;
                double rad_C = tool_state.c * DEG_TO_RAD;
                double offset_Z = getPivotOffsetZ();

                double x_c = tool_state.x * std::cos(rad_C) - tool_state.y * std::sin(rad_C);
                double y_c = tool_state.x * std::sin(rad_C) + tool_state.y * std::cos(rad_C);
                double z_c = tool_state.z;

                z_c += offset_Z;

                double x_a = x_c;
                double y_a = y_c * std::cos(rad_A) - z_c * std::sin(rad_A);
                double z_a = y_c * std::sin(rad_A) + z_c * std::cos(rad_A);

                machine_state.x = x_a;
                machine_state.y = y_a;
                machine_state.z = z_a - offset_Z;

                // checkMachineLimits(machine_state);
                return machine_state;
            }
    };
}