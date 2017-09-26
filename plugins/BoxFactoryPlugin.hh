#ifndef PLUGINS_BOXFACTORYPLUGIN_H
#define PLUGINS_BOXFACTORYPLUGIN_H


#include <gazebo/gazebo.hh>

namespace gazebo {
    class BoxFactoryPlugin : public WorldPlugin {
        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) override;
    };
}


#endif //PLUGINS_BOXFACTORYPLUGIN_H
