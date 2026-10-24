#include <memory>

#include "parser/parser.hpp"
#include "generator/s_curve_generator.hpp"
#include "generator/trapezoid_generator.hpp"
#include "kinematics/kinematics.hpp"

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "[ERROR] Usage: " << argv[0] << " <s_curve|trapezoidal> <path_to_gcode.tap>\n";
        return 1;
    }

    constexpr size_t frequency = 1000;

    constexpr em::GeometryVector max_pos = {150, 180, 0, 110, 1e99};
    constexpr em::GeometryVector min_pos = {-150, -180, -180, -110, -1e99};
    constexpr em::GeometryVector max_vel = {250, 500, 250, 461, 720};
    constexpr em::GeometryVector max_acc = {1510, 1390, 2340, 9220, 7200};
    constexpr em::GeometryVector max_jrk = {15000, 14000, 25000, 90000, 75000};
    constexpr em::MachineLimits machine_limits = {
        max_pos,
        min_pos,
        max_vel,
        max_acc,
        max_jrk
    };

    constexpr double pivot_offset = -85.0;
    constexpr double setup_offset = 65.0;

    std::ifstream file(argv[2]);
    if (!file.is_open())
    {
        std::cerr << "[ERROR] Failed to open file " << argv[2] << std::endl;
        return 1;
    }

    em::GCodeParser parser(file);
    std::cout << "[INFO] Parsed control states from '" << argv[2] << "'" << std::endl;

    file.close();

    const std::vector<em::MachineState> control_states = parser.getControlStates();

    const std::string generator_type = argv[1];
    std::shared_ptr<em::Generator> generator;
    if (generator_type == "s_curve")
    {
        generator = std::make_shared<em::SCurveGenerator>(frequency, machine_limits);
    } else if (generator_type == "trapezoidal")
    {
        generator = std::make_shared<em::TrapezoidalGenerator>(frequency, machine_limits);
    } else
    {
        std::cerr << "[ERROR] Allowed generator types: s_curve | trapezoidal" << std::endl;
        return 1;
    }

    generator->generate(control_states);
    std::cout << "[INFO] Generated trajectory for the control states" << std::endl;

    em::Kinematics kinematics(machine_limits, pivot_offset, setup_offset);
    const std::vector<em::TrajectoryPoint> trajectory = generator->getTrajectory();
    for (em::TrajectoryPoint trajectory_point : trajectory)
    {
        kinematics.calculateIK(trajectory_point.pos);
    }
    std::cout << "[INFO] Verified position limits for the machine using IK" << std::endl;

    generator->exportTrajectoryToCSV("trajectory.csv");
    std::cout << "[INFO] Exported trajectory to 'trajectory.csv'" << std::endl;

    return 0;
}