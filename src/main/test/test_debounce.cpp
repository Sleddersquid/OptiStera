#include <iostream>
#include <chrono>

#include <thread>

#define TEMP_UPDATE 1 * 1000 // in ms

int main()
{

    auto last_read = std::chrono::high_resolution_clock::now();
    int32_t debounce_interval = TEMP_UPDATE; // in ms

    while (true)
    {

        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read).count() > debounce_interval)
        {
            last_read = now;
            std::cout << "Debounce interval passed" << std::endl;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read).count() << std::endl;
            continue;
        }

        std::cout << "FILW RITING JUST HAPPENED" << std::endl;
    }
}
