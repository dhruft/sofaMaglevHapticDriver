#include "SOFAHapticDevice.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

namespace sofa::component::controller
{

SOFAHapticDevice::SOFAHapticDevice()
    : d_position(initData(&d_position, "position", "Position of the haptic device"))
    , d_velocity(initData(&d_velocity, "velocity", "Velocity of the device"))
    , d_button1(initData(&d_button1, "button1", "Button state"))
    , d_targetForce(initData(&d_targetForce, "targetForce", "Force to apply back to device"))
    , d_scale(initData(&d_scale, 1.0, "scale", "Motion scaling factor"))
    , l_forceFeedback(initLink("forceFeedBack", "link to the forceFeedBack component, if not set will search through graph and take first one encountered."))
{
    this->f_listening.setValue(true); // Enable onAnimateBegin
}

SOFAHapticDevice::~SOFAHapticDevice()
{
    StopHaptics();
}

void SOFAHapticDevice::init()
{
    // Start your C++ Driver (The 1000Hz Loop)
    printf("mmmmmmmmmmmmmmmmmmmmmmmmmmmmm");
    StartHaptics();
    printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    SetScalingFactor(d_scale.getValue());
    printf("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
    SetControlMode(0); // Default to Spring/Impedance mode
    printf("cccccccccccccccccccccccccccccc");
    
    sofa::type::vector<sofa::type::Vec3d> initialPos;
    initialPos.push_back(sofa::type::Vec3d(0,0,0)); // Start at origin
    d_position.setValue(initialPos);
}

void SOFAHapticDevice::reinit()
{
    // Called if you change "scale" in the GUI at runtime
    SetScalingFactor(d_scale.getValue());
}

//void SOFAHapticDevice::AnimateBeginEvent(const double dt)
void SOFAHapticDevice::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event))
    {

	    // 1. READ from Device
	    double x, y, z;
	    GetPosition(&x, &y, &z);
	    sofa::type::vector<sofa::type::Vec3d> posVec;
	    posVec.push_back(sofa::type::Vec3d(x, y, z)); 

	    int b1, b2;
	    GetButtonState(&b1, &b2);

	    // 2. WRITE to SOFA
	    // This updates the mechanical state of the "Tool" in the simulation
	    d_position.setValue(posVec);
	    d_button1.setValue(b1);

	    // 3. READ Force from SOFA (Collision Response)
	    // (For now, we just assume something writes to d_targetForce. 
	    //  In the next step, we will connect this to the collision engine).
	    // Inside handleEvent (in the AnimateEndEvent block):

	    // 1. Get the pointer to the vector
	    const auto& forceVec = d_targetForce.getValue();
/*
	    // 2. Check if it's empty
	    if (not forceVec.empty()) {
	    	printf("YUMMERS 1");
	    };

	    // 3. Take the first value (index 0)
	    type::Vec3d f = forceVec[0];

	    // 4. Debug & Send
	    if (f.norm() > 0.0001) {
		printf(">>> FORCE CAUGHT: %f %f %f\n", f[0], f[1], f[2]);
	    } else {
	    	printf("nope");
	    }

	    // Scale it up if needed (physics forces can be weak) 
	    // e.g., standard penalty stiffness=1000 might result in 5N, 
	    // but your device might need 5000 (mN) or more.
	    double forceScale = 1.0; 
	    SetForce(f[0] * forceScale, f[1] * forceScale, f[2] * forceScale);
	   */
    }
    
    else if (dynamic_cast<sofa::simulation::AnimateEndEvent*>(event))
    {
        // Access the vector of forces
        const auto& forceVec = d_targetForce.getValue();

        // Safety check: Is the vector empty?
        if (forceVec.empty()) return;
        
        printf("YUMMERS 2");

        // Get the first force
        type::Vec3d f = forceVec[0];

        // DEBUG: Print if force is detected
        if (f.norm() > 0.0001) {
            printf(">>> FORCE CAUGHT: %f %f %f\n", f[0], f[1], f[2]);
        } else {
        printf("fish");
        }

        // Send to hardware (multiply by 1000 if your device needs mN)
        SetForce(f[0], f[1], f[2]);
    }

}

} // namespace
