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
     
    // Mode 0: Force Control (Simulated via Position Offset)
    // Mode 1: Spring Control (Built-in Impedance)
    // Mode 2: Plane Constraint (The "True" LGA)
    EXPORT_API void SetControlMode(int mode);
    
    // Forces the device to stay on the positive side of a plane
    EXPORT_API void SetPlane(double nx, double ny, double nz, double distance, double stiffness, double damping);

    // Send a raw force vector (N) - Use this for SOFA
    EXPORT_API void SetForce(double fx, double fy, double fz);

    // Set a virtual spring target - Use this for simple Unity walls
    EXPORT_API void SetSpring(double targetX, double targetY, double targetZ, double stiffness, double damping);

    // Motion Scaling: 1.0 = 1:1 ratio. 10.0 = 1cm real movement is 10cm virtual.
    EXPORT_API void SetScalingFactor(double scale);
}
