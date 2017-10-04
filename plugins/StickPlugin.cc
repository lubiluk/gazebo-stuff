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
    _sdf->GetElement("force")->GetValue()->Get(this->forceThreshold);

    this->parentLink = parentModel->GetLink("link");
    this->childLink = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(link));

    this->joint = physics->CreateJoint("fixed", parentModel);
    // Bullet physics needs accurate joint position
    // ODE does't care
    this->joint->Load(this->parentLink, this->childLink, this->parentLink->GetWorldPose() - this->childLink->GetWorldPose());
    this->joint->Init();
    this->joint->SetProvideFeedback(true);
    this->joint->SetName("stick_joint_" + this->parentLink->GetScopedName() + "_" + this->childLink->GetScopedName());
    
    // Disable gravity on the butter link
    this->parentLink->SetGravityMode(false);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&StickPlugin::OnUpdate, this, _1));
}

void StickPlugin::OnUpdate(const common::UpdateInfo &_info) {
    auto wrench = this->joint->GetForceTorque(0u);
    auto measuredForce = wrench.body1Force;

    auto force = this->forceThreshold;

    auto measuredForceLength = measuredForce.GetLength();

    if (measuredForceLength > force) {
        gzdbg << "Removed joint: " << " (" << joint->GetName() << "), force: " << measuredForceLength << "\n";
        this->joint->Detach();
        this->joint = nullptr;
        
        // Enable gravity on the childLink
        this->parentLink->SetGravityMode(true);

        event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
        this->updateConnection = nullptr;
    }
}
