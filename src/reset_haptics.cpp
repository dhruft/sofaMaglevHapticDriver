#include "HapticDriver.h"
#include <iostream>

int main() {
    std::cout << "Attempting to release haptic device..." << std::endl;
    if (StartHaptics()) {
        StopHaptics(); // This triggers the takeoff/land sequence to center the device
        std::cout << "Success. Device should now be at (0,0,0) and free." << std::endl;
    } else {
        std::cerr << "Could not connect to device." << std::endl;
    }
    return 0;
}