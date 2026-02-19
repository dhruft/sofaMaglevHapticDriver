#include <sofa/core/ObjectFactory.h>
#include "SOFAHapticDevice.h"

using sofa::component::controller::SOFAHapticDevice;

extern "C" {
    // This is the function SOFA looks for
    void initExternalModule()
    {
        // You can print a message to confirm it loaded
        printf("HapticDriver Plugin Loaded!\n");
        
        // MANUALLY REGISTER THE CLASS HERE
        // This cannot be optimized away by the compiler
        int err = sofa::core::RegisterObject("Custom Haptic Device Driver")
            .add< sofa::component::controller::SOFAHapticDevice >() // Ensure namespace matches exactly
            .addAlias("MyHapticDriver")
            .addAlias("HapticDriver");
            
        // Just to be sure, print if it worked (err is usually the class ID)
        printf("Registered SOFAHapticDevice with ID: %d\n", err);
    }

    const char* getModuleName()
    {
        return "HapticDriver";
    }

    const char* getModuleVersion()
    {
        return "0.1";
    }

    const char* getModuleLicense()
    {
        return "LGPL";
    }

    const char* getModuleDescription()
    {
        return "A custom driver for magnetic levitation haptics";
    }

    const char* getModuleComponentList()
    {
        // This is just for information/GUI help
        return "SOFAHapticDevice";
    }
}
