#pragma once

// CHANGE 1: Include the high-level Controller header
#include <sofa/component/controller/Controller.h> // Was BaseController.h
#include <sofa/type/Vec.h>
#include "HapticDriver.h"

namespace sofa::component::controller
{

// CHANGE 2: Inherit from "Controller", not "BaseController"
class SOFAHapticDevice : public sofa::component::controller::Controller
{
public:
    // CHANGE 3: Update the macro
    SOFA_CLASS(SOFAHapticDevice, sofa::component::controller::Controller);

    Data<sofa::type::vector<sofa::type::Vec3d>> d_position; 
    Data<sofa::type::vector<sofa::type::Vec3d>> d_velocity;
    Data<int> d_button1;
    Data<sofa::type::vector<sofa::type::Vec3d>> d_targetForce;
    Data<double> d_scale; 

    SOFAHapticDevice();
    virtual ~SOFAHapticDevice();

    void init() override;
    void reinit() override;

    // CHANGE 4: The correct virtual function name in modern SOFA
    // It takes "const double" for the timestep (dt)
    //void onBeginAnimationStep(const double dt) override;
    void handleEvent(sofa::core::objectmodel::Event* event) override; 
};

} // namespace
