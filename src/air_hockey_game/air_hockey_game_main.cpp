#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "air_hockey_game/air_hockey_game.hpp"
#include "geometry/point.hpp"

int main() {
    try {
        constexpr double length{2.};
        constexpr double width{2.};
        simple_slice::air_hockey_game::AirHockey game(length, width);

        // Get the first 10 wall contacts and print them
        const auto initial_position{simple_slice::geometry::Point{1., 1.}};
        constexpr auto angle{22.5};
        const auto wall_contacts = game.puck_hit_locations(initial_position, angle);
        std::cout << std::fixed << std::setprecision(2) << "First " << wall_contacts.size()
                  << " wall contacts in (x, y, z) format:\n";
        for (std::size_t i = 0; i < wall_contacts.size(); ++i) {
            std::cout << (i + 1) << ": " << wall_contacts[i] << "\n";
        }

        // Print the table with hit locations as an ASCII schematic
        game.PrintTable(wall_contacts, /*cols=*/80, /*rows=*/30);

        return 0;

    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid argument: " << e.what() << "\n";
    } catch (const std::out_of_range& e) {
        std::cerr << "Out of range: " << e.what() << "\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
    }

    return 1;
}
