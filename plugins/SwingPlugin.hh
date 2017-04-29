#ifndef PLUGINS_SWINGPLUGIN_HH
#define PLUGINS_SWINGPLUGIN_HH

#include <gazebo/gazebo.hh>

namespace gazebo {
    class SwingPlugin : public ModelPlugin {
        public: void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/) override;
        // Pointer to the model
        private: physics::ModelPtr model;
        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };
}

#endif //PLUGINS_SWINGPLUGIN_HH
