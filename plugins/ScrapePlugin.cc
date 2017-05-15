#include "ScrapePlugin.hh"

#include <gazebo/physics/physics.hh>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ScrapePlugin);

void ScrapePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    this->model = _parent;
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();

    const auto &world = this->model->GetWorld();
    const auto &physics = world->GetPhysicsEngine();
    const auto &contactManager = physics->GetContactManager();
    auto filterName = this->model->GetScopedName() + "_contacts";

    std::vector<std::string> collisionNames;

    for (const auto &link : this->model->GetLinks()) {
        for (const auto &collision : link->GetCollisions()) {
            collisionNames.push_back(collision->GetScopedName());
        }
    }

    // Contact filter
    auto topic = contactManager->CreateFilter(filterName, "knife::link::collision");
    this->contactSubscriber = this->node->Subscribe(topic, &ScrapePlugin::OnContacts, this);

    // World update connection
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ScrapePlugin::OnUpdate, this, _1));

    // Set the initial movement direction
    this->velocity = math::Vector3(0.0, 0.0, -0.1);
}

void ScrapePlugin::OnContacts(ConstContactsPtr &_msg) {
    if (_msg->contact_size() == 0) return;
    if (_msg->contact(0).collision2() == "butter::link::collision") return;

    // Scraping movement direction
    std::lock_guard<std::mutex> lock(this->mutexVelocity);
    this->velocity = math::Vector3(-0.1, 0.0, 0.0);
}

void ScrapePlugin::OnUpdate(const common::UpdateInfo &_info) {
    std::lock_guard<std::mutex> lock(this->mutexVelocity);
    this->model->SetLinearVel(this->velocity);
}
