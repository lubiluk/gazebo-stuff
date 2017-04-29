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
            new gazebo::common::PoseAnimation("swing", 6.0, false));

    gazebo::common::PoseKeyFrame *key;

    math::Vector3 pos = parent->GetWorldPose().pos;
    math::Quaternion rot = parent->GetWorldPose().rot;

    // set starting location of the box
    key = anim->CreateKeyFrame(0);
    key->SetTranslation(pos);
    key->SetRotation(rot);

    // set waypoint location after 1 seconds
    key = anim->CreateKeyFrame(2.0);
    key->SetTranslation(pos + math::Vector3(0, 0.10, 0.0));
    key->SetRotation(rot);


    key = anim->CreateKeyFrame(4.0);
    key->SetTranslation(pos + math::Vector3(0, 0.15, 0.1));
    key->SetRotation(rot);

    key = anim->CreateKeyFrame(6.0);
    key->SetTranslation(pos + math::Vector3(0, -0.20, 0.15));
    key->SetRotation(rot);
//
//        key = anim->CreateKeyFrame(8.0);
//        key->SetTranslation(math::Vector3(-0.854600, 1.156014, 0.824962));
//        key->SetRotation(math::Quaternion(3.136410, -0.519508, 0.002588));

    // set the animation
    parent->SetAnimation(anim);
}
