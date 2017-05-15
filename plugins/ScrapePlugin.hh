#ifndef PLUGINS_SCRAPEPLUGIN_HH
#define PLUGINS_SCRAPEPLUGIN_HH

#include <gazebo/gazebo.hh>

namespace gazebo {
    /**
    * A movement of scraping butter form a knife against edge of a frying pan. It moves an object downwards, until
    * a contact occurs, and then moves it backward.
    */
    class ScrapePlugin : public ModelPlugin {
    public:
        void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/) override;

        void OnContacts(ConstContactsPtr &_msg);

        void OnUpdate(const common::UpdateInfo &_info);

    private:
        physics::ModelPtr model;
        transport::NodePtr node;
        transport::SubscriberPtr contactSubscriber;
        event::ConnectionPtr updateConnection;
        math::Vector3 velocity;
        std::mutex mutexVelocity;
    };
}

#endif //PLUGINS_SCRAPEPLUGIN_HH
