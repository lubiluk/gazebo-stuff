#ifndef PLUGINS_PR2GRIPPERGRIPPLUGIN_HH
#define PLUGINS_PR2GRIPPERGRIPPLUGIN_HH

#include <gazebo/gazebo.hh>

namespace gazebo {
    class PR2GripperGripPlugin : public ModelPlugin {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
    };
}

#endif //PLUGINS_PR2GRIPPERGRIPPLUGIN_HH
