#pragma once

#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>

#include "types.hpp"

namespace em
{
    class GCodeParser
    {
        private:
            std::vector<MachineState> control_states;

        public:

            GCodeParser(std::ifstream& file)
            {
                std::string line;
                MachineState current_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 

                while (std::getline(file, line))
                {
                    size_t comment_start = line.find('(');
                    if (comment_start != std::string::npos)
                    {
                        size_t comment_end = line.find(')', comment_start);
                        if (comment_end != std::string::npos)
                        {
                            line.erase(comment_start, comment_end - comment_start + 1);
                        } else
                        {
                            line.erase(comment_start);
                        }
                    }

                    std::istringstream iss(line);
                    std::string token;
                    bool position_changed = false;

                    while (iss >> token)
                    {
                        if (token.empty()) continue;
                        char prefix = std::toupper(token[0]);
                        try
                        {
                            if (prefix == 'X') { current_state.x = std::stod(token.substr(1)); position_changed = true; }
                            else if (prefix == 'Y') { current_state.y = std::stod(token.substr(1)); position_changed = true; }
                            else if (prefix == 'Z') { current_state.z = std::stod(token.substr(1)); position_changed = true; }
                            else if (prefix == 'A') { current_state.a = std::stod(token.substr(1)); position_changed = true; }
                            else if (prefix == 'C') { current_state.c = std::stod(token.substr(1)); position_changed = true; }
                            else if (prefix == 'F') { current_state.f = std::stod(token.substr(1)) / 60.0; }
                        } catch (const std::invalid_argument& e) { continue; }
                    }

                    if (position_changed)
                    {
                        control_states.push_back(current_state);
                    }
                }

                std::cout << "[INFO] Successfully parsed " << control_states.size() << " control states." << std::endl;
            }

            const std::vector<MachineState>& getControlStates() const { return control_states; }
    };
}