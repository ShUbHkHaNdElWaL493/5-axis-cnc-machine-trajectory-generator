#include "parser/parser.hpp"
#include "generator/s_curve_generator.hpp"
#include "generator/trapezoid_generator.hpp"

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "[ERROR] Usage: " << argv[0] << " <path_to_gcode.tap>\n";
        return 1;
    }

    const size_t frequency = 1000;

    const em::GeometryVector limits_vel = {250, 500, 250, 461, 720};
    const em::GeometryVector limits_acc = {1510, 1390, 2340, 9220, 7200};
    const em::GeometryVector limits_jrk = {15000, 14000, 25000, 90000, 75000};

    std::ifstream file(argv[1]);
    if (!file.is_open())
    {
        std::cerr << "[ERROR] Failed to open file " << argv[1] << "\n";
    }

    em::GCodeParser parser(file);

    file.close();

    std::vector<em::MachineState> control_states = parser.getControlStates();

    em::SCurveGenerator s_curve_generator(
        frequency,
        {
            limits_vel,
            limits_acc,
            limits_jrk
        }
    );
    s_curve_generator.generate(control_states);
    s_curve_generator.exportTrajectoryToCSV("s_curve.csv");

    em::TrapezoidalGenerator trapezoidal_generator(
        frequency,
        {
            limits_vel,
            limits_acc,
            limits_jrk
        }
    );
    trapezoidal_generator.generate(control_states);
    trapezoidal_generator.exportTrajectoryToCSV("trapezoidal.csv");

    return 0;
}