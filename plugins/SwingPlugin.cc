#include "SwingPlugin.hh"
#include <gazebo/physics/physics.hh>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SwingPlugin)

void SwingPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/)
{
    // Store the pointer to the model that will be manipulated
    this->model = parent;

    // create the animation
    gazebo::common::PoseAnimationPtr anim(
            new gazebo::common::PoseAnimation("swing", 8.0, false));

    gazebo::common::PoseKeyFrame *key;

    auto pos = parent->GetWorldPose().pos.Ign();
    auto rot = parent->GetWorldPose().rot.Ign();

    // set starting location of the box
    key = anim->CreateKeyFrame(0);
    key->Translation(pos);
    key->Rotation(rot);

    // set waypoint location after 1 seconds
    key = anim->CreateKeyFrame(2.0);
    key->Translation(pos + ignition::math::Vector3d(0.0, 0.07, 0.0));
    key->Rotation(rot + ignition::math::Quaterniond(0.5, 0.0, 0.0));


    key = anim->CreateKeyFrame(4.0);
    key->Translation(pos + ignition::math::Vector3d(0.0, 0.07, 0.1));
    key->Rotation(rot + ignition::math::Quaterniond(0.5, 0.0, 0.0));

    key = anim->CreateKeyFrame(6.0);
    key->Translation(pos + ignition::math::Vector3d(0.0, -0.35, 0.035));
    key->Rotation(rot + ignition::math::Quaterniond(-3.3, 0.0, 0.0));

    key = anim->CreateKeyFrame(8.0);
    key->Translation(pos + ignition::math::Vector3d(-0.30, -0.35, 0.035));
    key->Rotation(rot + ignition::math::Quaterniond(-3.3, 0.0, 0.0));

    // set the animation
    parent->SetAnimation(anim);
}
