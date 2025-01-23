#include <iostream>
#include <gpiod.hpp>


#include <thread>
#include <chrono>


int main() {
    gpiod::chip chip("gpiochip0");

    gpiod::line button_value = chip.get_line(14);

    button_value.request({"input", gpiod::line_request::DIRECTION_INPUT, 0});

    while(true) {
        int value = button_value.get_value();

        std::cout << "Button value: " << value << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}