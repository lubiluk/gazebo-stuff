//
// Created by lubiluk on 4/29/17.
//

#ifndef PLUGINS_GRIPPLUGIN_HH
#define PLUGINS_GRIPPLUGIN_HH

#include <gazebo/gazebo.hh>

namespace gazebo {
    class GripPlugin : public ModelPlugin {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
    };
}

#endif //PLUGINS_GRIPPLUGIN_HH
