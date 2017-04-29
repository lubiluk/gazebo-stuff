#include "ButterPlugin.hh"
#include <vector>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ButterPlugin)

void ButterPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
    this->model = _parent;
    this->world = this->model->GetWorld();
    this->physics = this->world->GetPhysicsEngine();
    this->node = transport::NodePtr(new transport::Node());

    ////

    this->node->Init();


    ////

    auto contactManager = this->physics->GetContactManager();
    auto filterName = this->model->GetScopedName() + "_contacts";

    std::vector<std::string> collisionNames;

    for (auto &link : this->model->GetLinks()) {
        for (auto &collision : link->GetCollisions()) {
            collisionNames.push_back(collision->GetScopedName());
        }
    }

    auto topic = contactManager->CreateFilter(filterName, collisionNames);
    this->contactSubscriber = this->node->Subscribe(topic, &ButterPlugin::OnContacts, this);

    ////

    for (auto &joint : this->model->GetJoints()) {
        joint->SetProvideFeedback(true);
        this->joints.push_back(joint);
    }

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ButterPlugin::OnUpdate, this, _1));
}

void ButterPlugin::OnUpdate(const common::UpdateInfo &_info) {
    // Detach
    auto it = this->joints.begin();
    while (it != this->joints.end()) {
        const auto &joint = (*it);

        auto wrench = joint->GetForceTorque(0u);
        auto measuredForce = wrench.body1Force;

        double force = 0.5;
        auto modelName = joint->GetChild()->GetModel()->GetName();

        if (modelName == "knife") force = 5;

        if (measuredForce.GetLength() > force) {
            gzdbg << "Removed joint: " << joint->GetParent()->GetScopedName() << " -> " << joint->GetChild()->GetScopedName() << " (" << joint->GetName() << "), force:" << force << modelName << "\n";
            joint->Detach();
            it = this->joints.erase(it);
        } else ++it;
    }

    // Attach

    std::lock_guard<std::mutex> lock(this->mutexContacts);
    for (const auto &contact : this->contacts) {
        auto name1 = contact.collision1();
        auto name2 = contact.collision2();

        auto collision1 = boost::dynamic_pointer_cast<physics::Collision>(this->world->GetEntity(name1));
        auto collision2 = boost::dynamic_pointer_cast<physics::Collision>(this->world->GetEntity(name2));

        // Get links involved in the contact
        // parentLink is a butter particle
        const auto &parentLink = collision1->GetLink();
        // childLink is an object that collided with the butter
        const auto &childLink = collision2->GetLink();

        // Check if any of the joints already connects both links
        const bool anyConnected = std::any_of(this->joints.begin(), this->joints.end(),
                                              [&parentLink, &childLink](physics::JointPtr &joint) {
                                                  return joint->AreConnected(parentLink, childLink);
                                              });

        // If not connected create a new joint from parent to child link
        if (anyConnected) continue;

        // Create a joint
        // Create a joint for sticky particle (parent)
        auto joint = this->physics->CreateJoint("revolute", this->model);

        // load the joint, and set up its anchor point
        joint->Load(parentLink, childLink, math::Pose());
        joint->Init();

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
        // Needed for the force-torque sensing to work
        joint->SetProvideFeedback(true);

        this->joints.push_back(joint);

        gzdbg << "Created joint: " << parentLink->GetScopedName() << " -> " << childLink->GetScopedName() << " (" << joint->GetName() << ")\n";
    }

    this->contacts.clear();
}

void ButterPlugin::OnContacts(ConstContactsPtr &_msg)
{
    std::lock_guard<std::mutex> lock(this->mutexContacts);

    for (int i = 0; i < _msg->contact_size(); ++i) {
        auto contact = _msg->contact(i);

        physics::CollisionPtr collision1 = boost::dynamic_pointer_cast<physics::Collision>(
                this->world->GetEntity(contact.collision1()));
        physics::CollisionPtr collision2 = boost::dynamic_pointer_cast<physics::Collision>(
                this->world->GetEntity(contact.collision2()));

        // Temporarily stick only to the knife
        if (collision1->GetModel()->GetName() != "knife" && collision2->GetModel()->GetName() != "knife") {
            continue;
        }

        this->contacts.push_back(contact);
    }
}