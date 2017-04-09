#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>

using namespace std;

namespace gazebo
{
class Swing : public ModelPlugin
{
  public:
    void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/)
    {
        // Store the pointer to the model that will be manipulated
        this->model = parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&Swing::OnUpdate, this, _1));

        this->initialPos = this->model->GetWorldPose().pos;
    }

    void OnUpdate(const common::UpdateInfo & /* info */)
    {
        auto pos = this->model->GetWorldPose().pos;

        if ( abs(pos.y - this->initialPos.y) < 0.15) {
            // Apply a small linear velocity to the model.
            this->model->SetLinearVel(math::Vector3(0, 0.1, 0));
        } else if ( abs(pos.z - this->initialPos.z) < 0.15) {
            // Apply a small linear velocity to the model.
            this->model->SetLinearVel(math::Vector3(0, 0.1, 0.1));
        }
    }

  private:
    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    math::Vector3 initialPos;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Swing)
}
