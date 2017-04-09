#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

using namespace std;

namespace gazebo
{
class SwingPlugin : public ModelPlugin
{
  public:
    void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/)
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

        // set waypoint location after 2 seconds
        key = anim->CreateKeyFrame(1.0);
        key->SetTranslation(pos + math::Vector3(0, 0.10, 0));
        key->SetRotation(rot);


        key = anim->CreateKeyFrame(3.0);
        key->SetTranslation(pos + math::Vector3(0, 0.15, 0.15));
        key->SetRotation(rot);

        // set the animation
        parent->SetAnimation(anim);
    }

  private:
    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SwingPlugin)
}
