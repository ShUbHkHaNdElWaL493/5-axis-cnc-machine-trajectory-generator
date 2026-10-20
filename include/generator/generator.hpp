#pragma once

#include <fstream>
#include <vector>

#include "types.hpp"

namespace em
{
    class Generator
    {
        protected:
            double dt;
            MachineLimits machine_limits;
            std::vector<TrajectoryPoint> trajectory;

        public:

            Generator(const size_t& frequency, const MachineLimits& machine_limits) :
            dt(1.0 / frequency),
            machine_limits(machine_limits)
            {}

            virtual ~Generator() = default;

            virtual void generate(const std::vector<MachineState>& control_states) = 0;

            void exportTrajectoryToCSV(const std::string& file_name)
            {
                std::ofstream file(file_name);
                file << "t,pos.x,pos.y,pos.z,pos.a,pos.c,vel.x,vel.y,vel.z,vel.a,vel.c,acc.x,acc.y,acc.z,acc.a,acc.c" << std::endl;
                for (TrajectoryPoint trajectory_point : trajectory)
                {
                    file << trajectory_point.t << ","
                         << trajectory_point.pos.x << ","
                         << trajectory_point.pos.y << ","
                         << trajectory_point.pos.z << ","
                         << trajectory_point.pos.a << ","
                         << trajectory_point.pos.c << ","
                         << trajectory_point.vel.x << ","
                         << trajectory_point.vel.y << ","
                         << trajectory_point.vel.z << ","
                         << trajectory_point.vel.a << ","
                         << trajectory_point.vel.c << ","
                         << trajectory_point.acc.x << ","
                         << trajectory_point.acc.y << ","
                         << trajectory_point.acc.z << ","
                         << trajectory_point.acc.a << ","
                         << trajectory_point.acc.c << std::endl;
                }
            }
    };
}