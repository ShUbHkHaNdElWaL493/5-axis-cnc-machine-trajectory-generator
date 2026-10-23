#pragma once

namespace em
{
    struct GeometryVector
    {
        double x, y, z, a, c;
    };

    struct MachineState
    {
        double x, y, z, a, c, f;
    };

    struct MachineLimits 
    {
        GeometryVector max_vel;
        GeometryVector max_acc;
        GeometryVector max_jrk;
    };

    struct TrajectoryPoint
    {
        double t;
        GeometryVector pos;
        GeometryVector vel;
        GeometryVector acc;
    };
}