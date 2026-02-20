#pragma once

#ifdef _WIN32
    #define EXPORT_API __declspec(dllexport)
#else
    #define EXPORT_API
#endif

extern "C" {
    /**
     * Lifecycle
     * Connects to 192.168.0.2, calibrates gravity, and registers the high-speed callback.
     * Returns 1 on success, 0 on failure.
     */
    EXPORT_API int StartHaptics();

    /**
     * Cleanup
     * Unregisters callbacks, lands the flotor, and disconnects.
     */
    EXPORT_API int StopHaptics();
    EXPORT_API int land();

    /**
     * Getters
     * Reads the latest position/velocity from the shared atomic memory.
     * Apply scaling factor before returning.
     */
    EXPORT_API void GetPosition(double* x, double* y, double* z);
    EXPORT_API void GetVelocity(double* Vx, double* Vy, double* Vz);
    EXPORT_API void GetOrientation(double* rx, double* ry, double* rz); // Rotation (radians)
    EXPORT_API void GetButtonState(int* button1, int* button2);

    /**
     * Setters
     * Determines how the haptic loop responds.
     */
    // Motion Scaling: 1.0 = 1:1 ratio. 10.0 = 1cm real movement is 10cm virtual.
    EXPORT_API void SetScalingFactor(double scale);
}
