#include "SOFAHapticDevice.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/Node.h> // Needed for SearchRoot
#include <chrono>

namespace sofa::component::controller
{

    // Initialize static member
SOFAHapticDevice* SOFAHapticDevice::s_instance = nullptr;

SOFAHapticDevice::SOFAHapticDevice()
    : d_position(initData(&d_position, "position", "Position of the haptic device"))
    , d_velocity(initData(&d_velocity, "velocity", "Velocity of the device"))
    , d_button1(initData(&d_button1, "button1", "Button state"))
    , d_targetForce(initData(&d_targetForce, "targetForce", "Force to apply back to device"))
    , d_scale(initData(&d_scale, 1.0, "scale", "Motion scaling factor"))
    , d_dampingForce(initData(&d_dampingForce, 0.0001, "damping", "Default damping applied to the force feedback"))
    , l_forceFeedback(initLink("forceFeedBack", "Link to the LCPForceFeedback component"))
{
    s_instance = this; 
    this->f_listening.setValue(true);
}

SOFAHapticDevice::~SOFAHapticDevice()
{

    // 1. Immediately null the singleton so the tick_callback skips logic
    s_instance = nullptr; 

    // 2. Stop the hardware threads
    StopHaptics();

    // std::ofstream outFile("haptic_log.csv");
    // outFile << "Time,Force,Penetration\n";
    // for (const auto& e : m_logBuffer) {
    //     outFile << e.timestamp << "," << e.forceNorm << "," << e.penetration << "\n";
    // }
    // outFile.close();
}

void SOFAHapticDevice::init()
{
    StartHaptics();
    SetScalingFactor(d_scale.getValue());
    
    if (l_forceFeedback.empty())
    {
        // Correct way to grab the pointer from the context
        this->getContext()->get(m_forceFeedback, sofa::core::objectmodel::BaseContext::SearchRoot);
    }
    else
    {
        m_forceFeedback = l_forceFeedback.get();
    }

    if (!m_forceFeedback)
        msg_warning() << "ForceFeedback component not found in scene graph.";
    
    // m_logBuffer.reserve(20000);
}

void SOFAHapticDevice::reinit()
{
    // Called if you change "scale" in the GUI at runtime
    SetScalingFactor(d_scale.getValue());
}

void SOFAHapticDevice::computeHapticForce(
    double x, double y, double z, 
    double u, double v, double w, double q,
    double& fx, double& fy, double& fz) {
    if (!m_forceFeedback) return;

    // Use a local VecDeriv to interface with the LCP solver
    sofa::type::Vec3d f(0,0,0);
    
    // THIS IS THE KEY: The LCP mini-solve happens here at 1000Hz!
    m_forceFeedback->computeForce(x, y, z, u, v, w, q, f[0], f[1], f[2]);
    
    fx = f[0]; fy = f[1]; fz = f[2];
}

void SOFAHapticDevice::handleEvent(sofa::core::objectmodel::Event* event) {
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event)) {
        // Now handleEvent ONLY updates the GHOST position in SOFA
        double x, y, z;
        GetPosition(&x, &y, &z);
        
        double u, v, w;
        GetOrientation(&u, &v, &w);

        Coord& posDevice = sofa::helper::getWriteOnlyAccessor(d_position);
        double scale = d_scale.getValue();
        posDevice.getCenter() = sofa::type::Vec3d(x*scale, y*scale, z*scale);
    }
}

} // namespace
