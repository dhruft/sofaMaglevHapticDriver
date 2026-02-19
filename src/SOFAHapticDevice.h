#pragma once

// CHANGE 1: Include the high-level Controller header
#include <sofa/core/objectmodel/Context.h>
#include <sofa/component/haptics/ForceFeedback.h> // Essential include
#include <sofa/component/controller/Controller.h> // Was BaseController.h
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/type/Vec.h>
#include "HapticDriver.h"

#include <vector>

struct HapticLogEntry {
    double timestamp;
    double forceNorm;
    double penetration;
};

namespace sofa::component::controller
{

// CHANGE 2: Inherit from "Controller", not "BaseController"
class SOFAHapticDevice : public sofa::component::controller::Controller
{
public:
    // CHANGE 3: Update the macro
    SOFA_CLASS(SOFAHapticDevice, sofa::component::controller::Controller);

    //Data<sofa::type::vector<sofa::type::Vec3d>> d_position; 
    // In your .h file
    typedef sofa::defaulttype::Rigid3Types::Coord Coord; // Position + Orientation
    sofa::core::objectmodel::Data<Coord> d_position;     // Not a vector, just one Coord
    Data<sofa::type::vector<sofa::type::Vec3d>> d_velocity;
    Data<int> d_button1;
    Data<sofa::type::vector<sofa::type::Vec3d>> d_targetForce;
    Data<double> d_scale; 
    Data<double> d_k;
    Data<double> d_d;
    Data<double> d_dampingForce;
    sofa::core::objectmodel::SingleLink<SOFAHapticDevice, sofa::component::haptics::ForceFeedback,sofa::core::objectmodel::BaseContext::SearchRoot> l_forceFeedback;
    
    sofa::component::haptics::ForceFeedback* m_forceFeedback = nullptr;
    // This allows the XML to point to the instrument's MechanicalObject
    // sofa::core::objectmodel::SingleLink<SOFAHapticDevice, sofa::component::statecontainer::MechanicalObject<sofa::defaulttype::Rigid3dTypes>, sofa::core::objectmodel::BaseContext::SearchRoot> l_instrumentMO;

    // Pointer for fast access during the simulation loop
    // sofa::component::statecontainer::MechanicalObject<sofa::defaulttype::Rigid3dTypes>* m_instrumentMO = nullptr;

    SOFAHapticDevice();
    virtual ~SOFAHapticDevice();

    void init() override;
    void reinit() override;

    // CHANGE 4: The correct virtual function name in modern SOFA
    // It takes "const double" for the timestep (dt)
    //void onBeginAnimationStep(const double dt) override;
    void handleEvent(sofa::core::objectmodel::Event* event) override; 

private:
    std::vector<HapticLogEntry> m_logBuffer;
};

} // namespace
