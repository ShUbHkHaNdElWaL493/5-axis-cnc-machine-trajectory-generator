#include "parser.hpp"

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "[ERROR] Usage: " << argv[0] << " <path_to_gcode.tap>\n";
        return 1;
    }

    std::ifstream file(argv[1]);
    if (!file.is_open())
    {
        std::cerr << "[ERROR] Failed to open file " << argv[1] << "\n";
    }

    em::GCodeParser parser(file);

    file.close();

    parser.printPoints();

    return 0;
}