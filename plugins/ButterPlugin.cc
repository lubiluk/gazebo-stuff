#include "ButterPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ButterPlugin)

void ButterPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
    this->model = _parent;
    this->world = this->model->GetWorld();
    this->physics = this->world->GetPhysicsEngine();
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();

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
    // Attach new joints
    std::lock_guard<std::mutex> lock(this->mutexContacts);
    for (const auto &contact : this->contacts) {
        const auto name1 = contact.collision1();
        const auto name2 = contact.collision2();

        const auto collision1 = boost::dynamic_pointer_cast<physics::Collision>(this->world->GetEntity(name1));
        const auto collision2 = boost::dynamic_pointer_cast<physics::Collision>(this->world->GetEntity(name2));

        // Get links involved in the contact
        // parentLink is a butter particle
        const auto &parentLink = collision1->GetLink();
        // childLink is an object that collided with the butter
        const auto &childLink = collision2->GetLink();

        // Check if any of the joints already connects both links
        // This seems not to work correctly
        bool anyConnected = std::any_of(this->joints.begin(), this->joints.end(),
                                              [&parentLink, &childLink](physics::JointPtr &joint) {
                                                  return joint->AreConnected(parentLink, childLink);
                                              });

        // If not connected create a new joint from parent to child link
        if (anyConnected) continue;

        // Create a joint
        // Create a joint for sticky particle (parent)
        auto joint = this->physics->CreateJoint("fixed", this->model);

        // load the joint, and set up its anchor point
        joint->Load(parentLink, childLink, math::Pose());
        joint->Init();

        // Enforce joint corrections
//        joint->SetParam("erp", 0, 1.0);
//        // Allow the joint to be elastic
//        joint->SetParam("cfm", 0, 1.0);

        // Child link is scoped in order to ensure uniqueness
        std::string nameIndex = parentLink->GetName() + "_" + childLink->GetScopedName();
        joint->SetName("joint_" + nameIndex);
        // Needed for the force-torque sensing to work
        joint->SetProvideFeedback(true);

        this->joints.push_back(joint);

        gzdbg << "Created joint: " << parentLink->GetScopedName() << " -> " << childLink->GetScopedName() << " (" << joint->GetName() << ")\n";
    }

    this->contacts.clear();

    // Detach strained joints

    auto it = this->joints.begin();
    while (it != this->joints.end()) {
        const auto &joint = (*it);

        auto wrench = joint->GetForceTorque(0u);
        auto measuredForce = wrench.body1Force;

        auto force = 0.4;
        auto modelName = joint->GetChild()->GetModel()->GetName();

        // Temporary hack for testing
        // Stickiness constant should be specified in link SDF
        if (modelName == "knife") force = 5000000.0;

        auto measuredForceLength = measuredForce.GetLength();

        if (measuredForceLength > force) {
            gzdbg << "Removed joint: " << joint->GetParent()->GetScopedName()
                  << " -> " << joint->GetChild()->GetScopedName()
                  << " (" << joint->GetName() << "), force: " << measuredForceLength << "\n";
            joint->Detach();
            it = this->joints.erase(it);
        } else ++it;
    }
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