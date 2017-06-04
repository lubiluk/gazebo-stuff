#include "ShpereContainerFactoryPlugin.hh"

#include <gazebo/physics/physics.hh>
#include <cmath>

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(ShpereContainerFactoryPlugin)

void ShpereContainerFactoryPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
    std::string poseArg = _sdf->GetElement("pose")->GetValue()->GetAsString();
    std::istringstream pss(poseArg);

    double x, y, z, pitch, yaw, roll;
    pss >> x >> y >> z >> roll >> pitch >> yaw;

    math::Pose pose(x, y, z, roll, pitch, yaw);

    unsigned containerRadius = 30;
    unsigned containerHeight = 6;
    double sphereRadius = 0.005;

    std::stringstream xml;

    xml << "<sdf version ='1.6'>"
            "<model name ='sphere_container'>"
            "<static>false</static>"
            "<pose>" << poseArg << "</pose>"
                "<link name='link'>";


    double sphereZ = 0.0;

    for (unsigned i = 1; i <= containerRadius; ++i) {
        double ringRadius = sphereRadius * i;
        unsigned sphereNumber = static_cast<unsigned>(ceil(M_PI * ringRadius / sphereRadius));
        double angleStep = 2 * M_PI / sphereNumber;

        for (unsigned j = 0; j < sphereNumber; ++j) {
            double angle = angleStep * j;
            double sphereX = ringRadius * sin(angle);
            double sphereY = ringRadius * cos(angle);
            std::string index = std::to_string(i) + "_" + std::to_string(j);

            xml << "<collision name ='collision_" << index << "'>"
                        "<pose>" << sphereX << " " << sphereY << " " << sphereZ << " 0 0 0</pose>"
                        "<geometry>"
                            "<sphere>"
                                "<radius>" << sphereRadius << "</radius>"
                            "</sphere>"
                        "</geometry>"
                    "</collision>"
                    "<visual name ='visual_" << index << "'>"
                        "<pose>" << sphereX << " " << sphereY << " " << sphereZ << " 0 0 0</pose>"
                        "<geometry>"
                            "<sphere>"
                            "<radius>" << sphereRadius << "</radius>"
                            "</sphere>"
                        "</geometry>"
                        "<material>"
                            "<script>"
                                "<uri>file://media/materials/scripts/gazebo.material</uri>"
                                "<name>Gazebo/DarkGrey</name>"
                            "</script>"
                        "</material>"
                    "</visual>";
        }
    }

    for (unsigned k = 0; k < containerHeight; ++k) {
        double ringRadius = sphereRadius * containerRadius;
        unsigned sphereNumber = static_cast<unsigned>(ceil(M_PI * ringRadius / sphereRadius));
        double angleStep = 2 * M_PI / sphereNumber;

        sphereZ = k * sphereRadius;

        for (unsigned j = 0; j < sphereNumber; ++j) {
            double angle = angleStep * j;
            double sphereX = ringRadius * sin(angle);
            double sphereY = ringRadius * cos(angle);
            std::string index = "rant_" + std::to_string(k) + "_" + std::to_string(j);

            xml << "<collision name ='collision_" << index << "'>"
                        "<pose>" << sphereX << " " << sphereY << " " << sphereZ << " 0 0 0</pose>"
                        "<geometry>"
                            "<sphere>"
                                "<radius>" << sphereRadius << "</radius>"
                            "</sphere>"
                        "</geometry>"
                    "</collision>"
                    "<visual name ='visual_" << index << "'>"
                        "<pose>" << sphereX << " " << sphereY << " " << sphereZ << " 0 0 0</pose>"
                        "<geometry>"
                            "<sphere>"
                                "<radius>" << sphereRadius << "</radius>"
                            "</sphere>"
                        "</geometry>"
                        "<material>"
                            "<script>"
                                "<uri>file://media/materials/scripts/gazebo.material</uri>"
                                "<name>Gazebo/DarkGrey</name>"
                            "</script>"
                        "</material>"
                    "</visual>";
        }
    }


    xml << "</link>"
           "</model>"
           "</sdf>";

    sdf::SDF butterSDF;
    butterSDF.SetFromString(xml.str());

    _parent->InsertModelSDF(butterSDF);
}
