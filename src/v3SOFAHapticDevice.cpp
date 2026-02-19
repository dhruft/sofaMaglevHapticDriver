#include "SOFAHapticDevice.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/Node.h> // Needed for SearchRoot

namespace sofa::component::controller
{

SOFAHapticDevice::SOFAHapticDevice()
    : d_position(initData(&d_position, "position", "Position of the haptic device"))
    , d_velocity(initData(&d_velocity, "velocity", "Velocity of the device"))
    , d_button1(initData(&d_button1, "button1", "Button state"))
    , d_targetForce(initData(&d_targetForce, "targetForce", "Force to apply back to device"))
    , d_scale(initData(&d_scale, 1.0, "scale", "Motion scaling factor"))
    , d_k(initData(&d_k, "k", "stiffness constant"))
    , d_d(initData(&d_d, "d", "derivative constant"))
    , l_instrumentMO(initLink("instrumentMO", "Link to the virtual instrument's MechanicalObject"))
{
    this->f_listening.setValue(true);
}

SOFAHapticDevice::~SOFAHapticDevice()
{
    StopHaptics();
}

void SOFAHapticDevice::init()
{
    StartHaptics();
    SetScalingFactor(d_scale.getValue());
    SetControlMode(1); 
   
    if (!l_instrumentMO.empty()) {
        m_instrumentMO = l_instrumentMO.get();
        msg_error() << "LINK PATH: " << l_instrumentMO.getPath();
    } else {
        msg_error() << "instrumentMO link is empty! The proxy position cannot be tracked.";
    }
    
    /*
    // 2. Manual Component Lookup
    // We look for any MechanicalObject in the entire scene graph
    const auto* context = dynamic_cast<const sofa::simulation::Node*>(this->getContext());
    
    // We search for a Rigid3d MechanicalObject starting from the root
    m_instrumentMO = context->get<sofa::component::statecontainer::MechanicalObject<sofa::defaulttype::Rigid3dTypes>>(
        this->getTags(), 
        sofa::core::objectmodel::BaseContext::SearchUp
    );*/

    if (m_instrumentMO) {
        msg_error() << "SUCCESS: Manually found instrument: " << m_instrumentMO->getName();
    } else {
        msg_error() << "FAILURE: Could not find a Rigid3d MechanicalObject anywhere in the scene.";
        

    }
}

void SOFAHapticDevice::reinit()
{
    // Called if you change "scale" in the GUI at runtime
    SetScalingFactor(d_scale.getValue());
}

void SOFAHapticDevice::handleEvent(sofa::core::objectmodel::Event* event)
{

    if (!m_instrumentMO) return;

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event))
    {
        // 1. Get Physical Device Position
        double x, y, z;
        GetPosition(&x, &y, &z);
        
        // Update the "Ghost" (HaplyMO) in SOFA
        Coord& posDevice = sofa::helper::getWriteOnlyAccessor(d_position);
        posDevice.getCenter() = sofa::type::Vec3d(x, y, z);

        // 2. Get Virtual Proxy Position (The Instrument)
        // We look for the MechanicalObject of the instrument that is colliding
        
        const auto& positions = m_instrumentMO->readPositions(); // Or just m_instrumentMO->readPositions()
    
	// Safety check to ensure the MO actually has points
        if (!positions.empty()) 
        {
		const auto& proxyPos = positions[0]; // This is your Rigid3d (Center + Orientation)
			
		SetControlMode(1); // Spring Mode
			
		// Send: (TargetX, TargetY, TargetZ, Stiffness, Damping)
               // Use a high stiffness (2000-4000) and healthy damping (20-40)
               SetSpring(proxyPos[0], proxyPos[1], proxyPos[2], d_k.getValue(), d_d.getValue());
	}
        
        
    }
}
/*
void SOFAHapticDevice::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event))
    {
        double x, y, z;
   	GetPosition(&x, &y, &z);

    	// Get a reference to the data inside SOFA
    	Coord& posDevice = sofa::helper::getWriteOnlyAccessor(d_position);
    
    	// Set the translation (Center)
    	posDevice.getCenter() = sofa::type::Vec3d(x, y, z) * d_scale.getValue();
    
    	// Set the rotation (Identity if you don't have orientation data)
    	posDevice.getOrientation() = sofa::type::Quat(0, 0, 0, 1); 

        // 2. FORCE FEEDBACK (SOFA -> Device)
        if (m_forceFeedback)
        {
            sofa::type::Vec3d force(0,0,0);
            
            // Ask the collision engine: "What force is at this position?"
            // Arguments: (px, py, pz, vx, vy, vz, tool_id, fx, fy, fz)
            m_forceFeedback->computeForce(x, y, z,  
                                         0, 0, 0, 0, 
                                         force[0], force[1], force[2]);

            // Optional: apply the same force scale as your Haply driver (forceCoef)
            // For now, sending raw physics force
            SetForce(force[0], force[1], force[2]);
            
            // Store for debug/GUI visualization
            d_targetForce.setValue({force});
        }
    }
}*/

} // namespace
