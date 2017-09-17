#include "StickPlugin.hh"

#include <gazebo/physics/physics.hh>
#include <string>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(StickPlugin);

void StickPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    const auto parentModel = _parent;
    const auto world = parentModel->GetWorld();
    const auto physics = world->GetPhysicsEngine();
    
    const std::string link = _sdf->GetElement("link")->GetValue()->GetAsString();

    const auto parentLink = parentModel->GetLink("link");
    const auto childLink = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(link));

    this->joint = physics->CreateJoint("fixed", parentModel);
    this->joint->Load(parentLink, childLink, parentLink->GetWorldPose() - childLink->GetWorldPose());
    this->joint->Init();
    this->joint->SetProvideFeedback(true);
    this->joint->SetName("stick_joint_" + parentLink->GetScopedName() + "_" + childLink->GetScopedName());

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&StickPlugin::OnUpdate, this, _1));
}

void StickPlugin::OnUpdate(const common::UpdateInfo &_info) {
    auto wrench = this->joint->GetForceTorque(0u);
    auto measuredForce = wrench.body1Force;

//    gzdbg << "Force: " << wrench.body1Force.GetLength() << " "
//          << wrench.body1Torque.GetLength() << " "
//          << wrench.body2Force.GetLength() << " "
//          << wrench.body2Torque.GetLength() << " " << "\n";

    auto force = 10000.0;

    auto measuredForceLength = measuredForce.GetLength();

    if (measuredForceLength > force) {
        gzdbg << "Removed joint: " << " (" << joint->GetName() << "), force: " << measuredForceLength << "\n";
        this->joint->Detach();
        this->joint = nullptr;

        event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
        this->updateConnection = nullptr;
    }
}
