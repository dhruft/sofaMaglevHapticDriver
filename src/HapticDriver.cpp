#include "HapticDriver.h"
#include "ml_api.h" // Your custom API header
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "SOFAHapticDevice.h"

extern "C" {

// --- CONSTANTS ---
const char* HOST_NAME = "192.168.0.2";
const double DEVICE_RADIUS = 0.012; // 12mm workspace
const double MAX_FORCE = 8.0;       // Safety clamp (Newtons)
const double STIFFNESS_MAX = 4000.0; // Max stiffness for force rendering

// --- SHARED MEMORY (Thread Safe) ---
std::atomic<double> g_posX(0), g_posY(0), g_posZ(0);
std::atomic<double> g_rotX(0), g_rotY(0), g_rotZ(0);
std::atomic<int>    g_btn1(0), g_btn2(0);
// --- ADDED VELOCITY ATOMICS ---
std::atomic<double> g_velX(0), g_velY(0), g_velZ(0); 

// Commands
std::atomic<double> g_scale(1.0);

// Device State
ml_device_handle_t device_hdl;
ml_forces_t gravity = { 0, 0, 0, 0, 0, 0 };
bool g_running = false;

// --- HELPER: Clamp Forces ---
double Clamp(double val, double max) {
    if (val > max) return max;
    if (val < -max) return -max;
    return val;
}

// --- BUTTON CALLBACK ---
int button_callback(ml_device_handle_t dev, ml_button_t butt) {
    g_btn1.store(butt.left);
    g_btn2.store(butt.right);
    return 0;
}

// Inside tick_callback
int tick_callback(ml_device_handle_t dev, ml_position_t* pos_ptr) {
    if (!g_running) return 0;

    // 1. Get current physical position
    double raw_x = pos_ptr->values[0];
    double raw_y = pos_ptr->values[1];
    double raw_z = pos_ptr->values[2];

    // Update global atomics for SOFA's handleEvent to see
    g_posX.store(raw_x * g_scale.load());
    g_posY.store(raw_y * g_scale.load());
    g_posZ.store(raw_z * g_scale.load());

    g_rotX.store(pos_ptr->values[3]);
    g_rotY.store(pos_ptr->values[4]);
    g_rotZ.store(pos_ptr->values[5]);

    // 2. REACH INTO SOFA AND COMPUTE LCP FORCE
    double fx = 0, fy = 0, fz = 0;
    if (sofa::component::controller::SOFAHapticDevice::s_instance) {
        // Call the mini-solver!
        sofa::component::controller::SOFAHapticDevice::s_instance->computeHapticForce(
            raw_x, raw_y, raw_z, 
            0, 0, 0, 0, 
            fx, fy, fz
        );
    }

    ml_position_t despos = {0};
    ml_gain_vec_t gains = {0};

    // Apply gravity compensation feed-forward
    for (int i=0; i<6; i++) {
        gains.values[i].ff = gravity.values[i];
    }
    
    fx = Clamp(fx, MAX_FORCE);
    fy = Clamp(fy, MAX_FORCE);
    fz = Clamp(fz, MAX_FORCE);

    double k = STIFFNESS_MAX; // Use high stiffness for crisp force transmission
    
    gains.values[0].p = k; gains.values[0].d = 30.0;
    gains.values[1].p = k; gains.values[1].d = 30.0;
    gains.values[2].p = k; gains.values[2].d = 30.0; // Z often needs higher gains?

    // Offset position trick
    despos.values[0] = raw_x + (fx / k);
    despos.values[1] = raw_y + (fy / k);
    despos.values[2] = raw_z + (fz / k);

    // Rotational Gains (Keep stiff to prevent spinning)
    for (int i=3; i<6; i++) {
        gains.values[i].p = 10.0;
        gains.values[i].d = 0.40;
    }

    // 3. Apply the Damping locally at 1000Hz (much more stable!)
    // [Calculate Velocity here as we did before]
    // fx -= (vx * damping); ...

    // 4. Send to Device
    ml_SetGainVecAxes(dev, ML_GAINSET_TYPE_NORMAL, gains);
    ml_SetDesiredPosition(dev, despos);

    return 0;
}

// --- EXPORTED FUNCTIONS ---

int StartHaptics() {
    if (g_running) return 1;

    // 1. Connect
    if (ml_Connect(&device_hdl, (char*)HOST_NAME) != ML_STATUS_OK) {
        printf("Failed to connect to %s\n", HOST_NAME);
        return 0;
    }

    // 2. Takeoff
    ml_RegisterCallbackButtonPressed(device_hdl, button_callback);
    if (ml_Takeoff(device_hdl) != ML_STATUS_OK) {
        printf("Takeoff failed\n");
        ml_Disconnect(device_hdl);
        return 0;
    }

    // 3. Radius Constraint
    ml_SetBoundaryRadius(device_hdl, DEVICE_RADIUS);

    // 4. Gravity Calibration
    // In bunny.cpp, they subtract 1 from Z to add mass sensation
    ml_FindGravity(device_hdl, &gravity);
    gravity.values[2] -= 1.0; 
    ml_SetGravity(device_hdl, gravity);
    ml_DefyGravity(device_hdl);

    // 5. Start Loop
    g_running = true;
    ml_RegisterCallbackTick(device_hdl, tick_callback);

    // 6. Set Speed Limits (Safety)
    ml_velocities_t limits;
    for(int i=0; i<3; i++) limits.values[i] = 10.0f; // Trans
    for(int i=3; i<6; i++) limits.values[i] = 50.0f; // Rot
    ml_SetSpeedLimits(device_hdl, limits);

    return 1;
}

int StopHaptics() {
    if (!g_running) return 1;
    g_running = false; // Stop the custom 1000Hz loop logic

    // 1. Stop the User Input
    ml_UnregisterCallbackButtonPressed(device_hdl);

    // 2. SAFETY: Set "Holding Gains"
    // These values (from your working snippet) are tuned to hold the device 
    // steady in free space without vibration.
    ml_gain_vec_t safe_gains = {0};
    
    // X and Y: Moderate stiffness (2000), High damping (30) to stop oscillation
    safe_gains.values[0].p = 2000; safe_gains.values[0].d = 30;
    safe_gains.values[1].p = 2000; safe_gains.values[1].d = 30;
    
    // Z: High stiffness (8000) to fight gravity
    safe_gains.values[2].p = 8000; safe_gains.values[2].d = 30;
    
    // Orientation: Loose but damped
    for(int i=3; i<6; i++) {
        safe_gains.values[i].p = 30.0;
        safe_gains.values[i].d = 0.3;
    }
    
    // Feed-forward gravity (keep this or it drops)
    safe_gains.values[2].ff = gravity.values[2];

    // Apply these safe gains NOW
    ml_SetGainVecAxes(device_hdl, ML_GAINSET_TYPE_NORMAL, safe_gains);

    // 3. Unlock Constraints
    // Remove any "Virtual Walls" (Plane Mode) so the device is free to move to center
    for (int i=0; i<6; i++) {
        ml_UnlockAxis(device_hdl, (ml_axis_index_t)i);
    }

    // 4. Reset to Center (Hover)
    // Now that gains are safe and axes are unlocked, this will smoothly move to (0,0,0)
    ml_Takeoff(device_hdl);

    // 5. Cleanup
    ml_UnregisterCallbackTick(device_hdl);
    ml_Disconnect(device_hdl);
    
    return 1;
}

 int land() {

    if (!g_running) return 1;

    g_running = false;


    // Based on stopDemo() in bunny.cpp
    ml_UnregisterCallbackButtonPressed(device_hdl);
    

    // Soften gains before landing
    ml_gain_vec_t soft_gains = {0};
    for(int i=0; i<3; i++) { soft_gains.values[i].p = 100; soft_gains.values[i].d = 10; }
    ml_SetGainVecAxes(device_hdl, ML_GAINSET_TYPE_NORMAL, soft_gains);

    // Unlock any constraints
    for(int i=0; i<6; i++) ml_UnlockAxis(device_hdl, (ml_axis_index_t)i);

    ml_UnregisterCallbackTick(device_hdl);
    ml_Disconnect(device_hdl);

    return 1;

} 

void GetPosition(double* x, double* y, double* z) {
    *x = g_posX.load();
    *y = g_posY.load();
    *z = g_posZ.load();
}

void GetVelocity(double* vx, double* vy, double* vz) {
    *vx = g_velX.load();
    *vy = g_velY.load();
    *vz = g_velZ.load();
}

void GetOrientation(double* rx, double* ry, double* rz) {
    *rx = g_rotX.load();
    *ry = g_rotY.load();
    *rz = g_rotZ.load();
}

void GetButtonState(int* b1, int* b2) {
    *b1 = g_btn1.load();
    *b2 = g_btn2.load();
}


void SetScalingFactor(double scale) {
    g_scale.store(scale);
}

} // End extern C
