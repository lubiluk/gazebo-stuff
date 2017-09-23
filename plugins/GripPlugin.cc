#include "GripPlugin.hh"

#include <gazebo/physics/physics.hh>
#include <string>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GripPlugin);

void GripPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    const auto parentModel = _parent;
    const auto world = parentModel->GetWorld();
    const auto physics = world->GetPhysicsEngine();

    const std::string link = _sdf->GetElement("link")->GetValue()->GetAsString();

    const auto parentLink = parentModel->GetLink("link");
    const auto childLink = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(link));

    const auto joint = physics->CreateJoint("fixed", parentModel);
    // Bullet physics needs accurate joint position
    // ODE does't care
    joint->Load(parentLink, childLink, parentLink->GetWorldPose() - childLink->GetWorldPose());
    joint->Init();
    joint->SetName("grip_joint_" + parentLink->GetScopedName() + "_" + childLink->GetScopedName());
}
