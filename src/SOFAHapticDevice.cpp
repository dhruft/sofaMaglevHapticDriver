#include "SOFAHapticDevice.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/Node.h> // Needed for SearchRoot
#include <chrono>

namespace sofa::component::controller
{

SOFAHapticDevice::SOFAHapticDevice()
    : d_position(initData(&d_position, "position", "Position of the haptic device"))
    , d_velocity(initData(&d_velocity, "velocity", "Velocity of the device"))
    , d_button1(initData(&d_button1, "button1", "Button state"))
    , d_targetForce(initData(&d_targetForce, "targetForce", "Force to apply back to device"))
    , d_scale(initData(&d_scale, 1.0, "scale", "Motion scaling factor"))
    , d_dampingForce(initData(&d_dampingForce, 0.0001, "damping", "Default damping applied to the force feedback"))
    , l_forceFeedback(initLink("forceFeedBack", "Link to the LCPForceFeedback component"))
{
    this->f_listening.setValue(true);
}

SOFAHapticDevice::~SOFAHapticDevice()
{
    StopHaptics();
    std::ofstream outFile("haptic_log.csv");
    outFile << "Time,Force,Penetration\n";
    for (const auto& e : m_logBuffer) {
        outFile << e.timestamp << "," << e.forceNorm << "," << e.penetration << "\n";
    }
    outFile.close();
}

void SOFAHapticDevice::init()
{
    StartHaptics();
    SetScalingFactor(d_scale.getValue());
    SetControlMode(0); 

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
    
    m_logBuffer.reserve(20000);
}

void SOFAHapticDevice::reinit()
{
    // Called if you change "scale" in the GUI at runtime
    SetScalingFactor(d_scale.getValue());
}

void SOFAHapticDevice::handleEvent(sofa::core::objectmodel::Event* event)
{
        
    const float damping = float(d_dampingForce.getValue());
    static auto lastTime = std::chrono::high_resolution_clock::now();
    static int frameCount = 0;
    
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event))
    {
    
    frameCount++;

    if (frameCount >= 100) {
        auto now = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration<double>(now - lastTime).count();
        msg_error() << "SOFA Physics Frequency: " << (100.0 / duration) << " Hz";
        lastTime = now;
        frameCount = 0;
    }

    double x, y, z;
   	GetPosition(&x, &y, &z);
   	
   	double Vx, Vy, Vz;
   	GetVelocity(&Vx, &Vy, &Vz);

    // Get a reference to the data inside SOFA
    Coord& posDevice = sofa::helper::getWriteOnlyAccessor(d_position);

    // Set the translation (Center)
    posDevice.getCenter() = sofa::type::Vec3d(x, y, z) * d_scale.getValue();

    // Set the rotation (Identity if you don't have orientation data)
    posDevice.getOrientation() = sofa::type::Quat(0, 0, 0, 1); 

    sofa::type::Vec3d force(0, 0, 0);

        // 2. FORCE FEEDBACK (SOFA -> Device)
	if (m_forceFeedback)
	{
	    
	    m_forceFeedback->computeForce(x, y, z, 0, 0, 0, 0, force[0], force[1], force[2]);

	    // Check if we are in contact (norm > 0)
	    if (force.norm() > 1e-6) // Use a small epsilon for float safety
	    //if (true)
	    {
            // Calculate velocity vector
            sofa::type::Vec3d velocity(Vx, Vy, Vz);
            
            printf("%f \n", force.norm());
            // Apply damping: F_total = F_sofa - (velocity * damping)
            // Note: damping needs to be tuned. 0.0001 might be too weak to feel.
            force -= (velocity * damping);
	    }
	    
	    SetForce(force[0], force[1], force[2]);
	    d_targetForce.setValue({force});
	}

    HapticLogEntry entry;
    entry.timestamp = this->getContext()->getTime();
    entry.forceNorm = force.norm();
    //entry.penetration = (proxyPos - handPos).norm();
    
    m_logBuffer.push_back(entry);
	
    }
}

} // namespace
