#include "StickyPlugin.hh"

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>


using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(StickyPlugin)

/////////////////////////////////////////////////
StickyPlugin::StickyPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
StickyPlugin::~StickyPlugin()
{
}

/////////////////////////////////////////////////
void StickyPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
    // Get the parent sensor.
    this->parentSensor =
            boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "StickyPlugin requires a ContactSensor.\n";
        return;
    }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(
            boost::bind(&StickyPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void StickyPlugin::OnUpdate()
{
    unsigned int collisionCount = this->parentSensor->GetCollisionCount();

    // Scan through all current collisions
    for (unsigned int i = 0; i < collisionCount; ++i) {
        auto collisionName = this->parentSensor->GetCollisionName(i);
        auto contacts = this->parentSensor->GetContacts(collisionName);

        // Go through all contacts and check if links involved are already connected
        for (const auto &contactRow : contacts) {
            auto contact = contactRow.second;
            // Get links involved in the contact
            // parentLink is a butter particle
            auto parentLink = contact.collision1->GetLink();
            // childLink is an object that collided with the butter
            auto childLink = contact.collision2->GetLink();

            // Get joints from parent to child
            auto childJoints = parentLink->GetChildJoints();

            // Check if any of the joints already connects both links
            auto anyConnected = std::any_of(childJoints.begin(), childJoints.end(),
                                            [&parentLink, &childLink](physics::JointPtr joint) {
                                                return joint->AreConnected(parentLink, childLink);
                                            });

            // If not connected create a new joint from parent to child link
            if (!anyConnected) {
                // Temporarily stick only to the knife
                if (childLink->GetModel()->GetName() != "knife") {
                    continue;
                }

                Stick(parentLink, childLink);
            }
        }
    }
}

void StickyPlugin::Stick(physics::LinkPtr parentLink, physics::LinkPtr childLink) {
    gzdbg << "New joint: " << parentLink->GetScopedName() << " -> " << childLink->GetScopedName() << "\n";

    // Get sticky model
    auto parentModel = parentLink->GetModel();

    // Create a joint for sticky particle (parent)
    auto joint = parentModel->GetWorld()->GetPhysicsEngine()->CreateJoint("revolute", parentModel);

    // attach joint to links
    joint->Attach(parentLink, childLink);

    // load the joint, and set up its anchor point
    joint->Load(parentLink, childLink, math::Pose());

    // set the axis of revolution
    joint->SetAxis(0, math::Vector3(0,0,1));

    // Set limits to 0 - make the joint fixes
    joint->SetHighStop(0, math::Angle::Zero);
    joint->SetLowStop(0, math::Angle::Zero);

    // Enforce joint corrections
    joint->SetAttribute("erp", 0, 1.0);
    // Allow the joint to be elastic
    joint->SetAttribute("cfm", 0, 1.0);

    // Child link is scoped in order to ensure uniqueness
    std::string nameIndex = parentLink->GetName() + "_" + childLink->GetScopedName();
    joint->SetName("joint_" + nameIndex);

    joint->Init();
}

//std::string StickyPlugin::GetBreakableJointString(std::string linkName1, std::string linkName2, std::string force) {
//    std::stringstream ss;
//    ss << "<joint name=\"joint_" << linkName1 << "_" << linkName2 << "\" type=\"revolute\">\
//                        <parent>" << linkName1 << "</parent>\
//                        <child>" << linkName2 << "</child>\
//                        <axis>\
//                            <xyz>0 0 1</xyz>\
//                            <limit>\
//                                <lower>0.0</lower>\
//                                <upper>0.0</upper>\
//                            </limit>\
//                        </axis>\
//                        <physics>\
//                            <ode>\
//                                <erp>1</erp>\
//                                <cfm>1</cfm>\
//                            </ode>\
//                        </physics>\
//                        <sensor name=\"force_torque\" type=\"force_torque\">\
//                              <always_on>true</always_on>\
//                              <update_rate>1000</update_rate>\
//                              <plugin name=\"breakable_0_0\" filename=\"libBreakableJointPlugin.so\">\
//                                    <breaking_force_N>" << force << "</breaking_force_N>\
//                              </plugin>\
//                        </sensor>\
//                   </joint>";
//
//    return ss.str();
//}