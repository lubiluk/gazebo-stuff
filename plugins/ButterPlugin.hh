#ifndef PLUGINS_BUTTERPLUGIN_HH
#define PLUGINS_BUTTERPLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <mutex>

namespace gazebo {
    class ButterPlugin : public ModelPlugin {
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
        // Called by the world update start event
        public: void OnUpdate(const common::UpdateInfo & _info);

        // Pointer to the model
        private: physics::ModelPtr model;
        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
        //
        private: transport::NodePtr node;
        private: transport::SubscriberPtr contactSubscriber;

        public: void OnContacts(ConstContactsPtr &_msg);

        /// \brief The current contacts.
        private: std::vector<msgs::Contact> contacts;
        /* \brief The current joints. Gazebo doesn't track added or removed joints,
         * so this plugin tracks them on its own. */
        private: std::vector<physics::JointPtr> joints;
        /// \brief Pointer to the world.
        private: physics::WorldPtr world;
        /// \brief The physics engine.
        private: physics::PhysicsEnginePtr physics;
        /// \brief Mutex used to protect reading/writing the contact message.
        public: std::mutex mutexContacts;
        private: void PrintJoints();
    };
}

#endif //PLUGINS_BUTTERPLUGIN_HH
