#ifndef PLUGINS_STICKYPLUGIN_HH
#define PLUGINS_STICKYPLUGIN_HH

#include <gazebo/gazebo.hh>

namespace gazebo
{
    /// \brief A plugin for a contact sensor. Inherit from this class to make
    /// your own contact plugin.
    class StickyPlugin : public SensorPlugin
    {
        /// \brief Constructor.
    public: StickyPlugin();

        /// \brief Destructor.
    public: virtual ~StickyPlugin();

        /// \brief Load the sensor plugin.
        /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
        /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

        /// \brief Callback that recieves the contact sensor's update signal.
        /// Override this this function to get callbacks when the contact sensor
        /// is updated with new data.
    private: virtual void OnUpdate();

        /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

        /// \brief Connection that maintains a link between the contact sensor's
        /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;


//    private: std::string GetBreakableJointString(std::string linkName1, std::string linkName2, std::string force);

    private: void Stick(physics::LinkPtr parentLink, physics::LinkPtr childLink);
    };
}


#endif //PLUGINS_STICKYPLUGIN_HH
