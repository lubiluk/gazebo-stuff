#ifndef PLUGINS_SPHERECONTAINERFACTORYPLUGIN_HH
#define PLUGINS_SPHERECONTAINERFACTORYPLUGIN_HH

#include <gazebo/gazebo.hh>

namespace gazebo {
    class ShpereContainerFactoryPlugin : public WorldPlugin {
    public:
        void Load(physics::WorldPtr parent, sdf::ElementPtr sdf) override;
    };
}


#endif //PLUGINS_SPHERECONTAINERFACTORYPLUGIN_HH
