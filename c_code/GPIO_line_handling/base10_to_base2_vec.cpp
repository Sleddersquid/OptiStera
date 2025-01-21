#include <iostream>

#include <bitset>

// Refrences:
// https://stackoverflow.com/questions/22746429/c-decimal-to-binary-converting
// https://stackoverflow.com/questions/12657962/how-do-i-generate-a-random-number-between-two-variables-that-i-have-stored
// https://cplusplus.com/reference/cstdlib/rand/

const unsigned int min_value = 0;
const unsigned int max_value = 63;
const int bit_length = 6;

unsigned int randNum = 0;

int main() {

    for (size_t i = 0; i < 100; i++) {
        randNum = rand() % (max_value - min_value + 1) + min_value;
        std::string binary = std::bitset<bit_length>(randNum).to_string(); //to binary
        std::cout << binary << "\n";
    }

    std::cout << "End of file" << std::endl;
    return 0;
}