#ifndef PLUGINS_STICKPLUGIN_H
#define PLUGINS_STICKPLUGIN_H


#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>

namespace gazebo {
    class StickPlugin : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
        void OnUpdate(const common::UpdateInfo & _info);

    private:
        physics::JointPtr joint;
        event::ConnectionPtr updateConnection;
    };
}


#endif //PLUGINS_STICKPLUGIN_H
