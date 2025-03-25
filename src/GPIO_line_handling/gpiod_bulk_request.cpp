#include <iostream>
#include <gpiod.hpp>
#include <vector>
#include <unistd.h>

// Remove this when I've found a ms_sleep function
#include <chrono>
#include <thread>

// Refrences:
// https://www.lane-fu.com/linuxmirror/libgpiod/doc/html/classgpiod_1_1line__bulk.html
// https://github.com/brgl/libgpiod/blob/master/examples/toggle_multiple_line_values.c
// https://stackoverflow.com/questions/4184468/sleep-for-milliseconds

const int sleep_time = 1000; // In milliseconds

const std::string test_string = "10";

int main() {
    gpiod::chip chip("gpiochip0");

    auto bulk_lines = chip.get_lines({23, 24}); // Channel 1, Channel 2

    bulk_lines.request({"output", gpiod::line_request::DIRECTION_OUTPUT, 0});

    bulk_lines.set_values({1, 0});

    // std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));

    bulk_lines.set_values({0, 1});

    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));

    bulk_lines.set_values({0, 0});

    std::cout << "End of file" << std::endl;
    return 0;
}