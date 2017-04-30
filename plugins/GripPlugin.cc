#include "GripPlugin.hh"

#include <gazebo/physics/physics.hh>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GripPlugin);

void GripPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    const auto parentModel = _parent;
    const auto world = parentModel->GetWorld();
    const auto physics = world->GetPhysicsEngine();

    const auto parentLink = parentModel->GetLink("link");
    const auto childModel = boost::dynamic_pointer_cast<physics::Model>(world->GetEntity("knife"));
    const auto childLink = childModel->GetLink("link");

    const auto joint = physics->CreateJoint("revolute", parentModel);
    joint->Load(parentLink, childLink, math::Pose());
    joint->Init();

    // set the axis of revolution
    joint->SetAxis(0, math::Vector3(0,0,1));
    // Set limits to 0 - make the joint fixes
    joint->SetHighStop(0, math::Angle::Zero);
    joint->SetLowStop(0, math::Angle::Zero);
    // Enforce joint corrections
    joint->SetParam("erp", 0, 1.0);
    // Do not allow the joint to be elastic
    joint->SetParam("cfm", 0, 0.0);
    joint->SetDamping(0, 0.5);
    joint->SetName("joint_" + parentLink->GetScopedName() + "_" + childLink->GetScopedName());
}