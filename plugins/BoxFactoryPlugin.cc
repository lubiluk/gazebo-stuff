#include "BoxFactoryPlugin.hh"
#include <gazebo/physics/physics.hh>
#include <sstream>

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(BoxFactoryPlugin)

void BoxFactoryPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
    std::string poseArg = _sdf->GetElement("pose")->GetValue()->GetAsString();
    std::istringstream pss(poseArg);

    double x, y, z, pitch, yaw, roll;
    pss >> x >> y >> z >> roll >> pitch >> yaw;

    math::Pose pose(x, y, z, roll, pitch, yaw);
    
    double width = 0.01;
    double depth = 0.01;
    double height = 0.1;
    double friction = 0.4;
    double friction2 = 0.4;
    double mass = 0.5;
    double thickness = 0.005;
    bool isStatic = false;
    
    _sdf->GetElement("width")->GetValue()->Get(width);
    _sdf->GetElement("depth")->GetValue()->Get(depth);
    _sdf->GetElement("height")->GetValue()->Get(height);
    _sdf->GetElement("friction")->GetValue()->Get(friction);
    _sdf->GetElement("friction2")->GetValue()->Get(friction2);
    _sdf->GetElement("mass")->GetValue()->Get(mass);
    _sdf->GetElement("thickness")->GetValue()->Get(thickness);
    _sdf->GetElement("static")->GetValue()->Get(isStatic);
    
    std::stringstream xml;
    
    xml << "<?xml version='1.0'?>"
           "<sdf version='1.6'>"
             "<model name='box'>"
               "<static>" << isStatic << "</static>"
               "<pose>" << pose << "</pose>"
               "<link name='link'>"
                 "<pose>0 0 0 0 0 0</pose>"
                 "<inertial>"
                   "<mass>" << mass << "</mass>"
                   "<pose>0 0 " << (height / 2.0 - 0.1 * height) << " 0 0 0</pose>"
                   "<inertia>"
                     "<ixx>" << (mass / 12.0 * (height*height + depth*depth)) << "</ixx>"
                     "<iyy>" << (mass / 12.0 * (width*width + depth*depth)) << "</iyy>"
                     "<izz>" << (mass / 12.0 * (width*width + height*height)) << "</izz>"
                     "<ixy>0</ixy>"
                     "<ixz>0</ixz>"
                     "<iyz>0</iyz>"
                    "</inertia>"
                  "</inertial>"
            
                  "<!-- Floor -->"
                  "<collision name='collision_floor'>"
                    "<pose>0 0 " << (thickness / 2.0) << " 0 0 0</pose>"
                    "<geometry>"
                      "<box>"
                        "<size>" << width << " " << (depth - thickness * 2) << " " << thickness << "</size>"
                      "</box>"
                    "</geometry>"
                    "<surface>"
                      "<friction>"
                        "<ode>"
                          "<mu>" << friction << "</mu>"
                          "<mu2>" << friction2 << "</mu2>"
                        "</ode>"
                        "<bullet>"
                            "<friction>" << friction << "</friction>"
                            "<friction2>" << friction2 << "</friction2>"
                        "</bullet>"
                      "</friction>"
                    "</surface>"
                  "</collision>"
                  "<visual name='visual_floor'>"
                    "<pose>0 0 " << (thickness / 2.0) << " 0 0 0</pose>"
                    "<geometry>"
                      "<box>"
                        "<size>" << width << " " << (depth - thickness * 2.0) << " " << thickness << "</size>"
                      "</box>"
                    "</geometry>"
                    "<material>"
                      "<script>"
                        "<name>Gazebo/White</name>"
                        "<uri>file://media/materials/scripts/gazebo.material</uri>"
                      "</script>"
                    "</material>"
                  "</visual>"
                  "<!-- /Floor -->"
                  
                  "<!-- Wall 1 -->"
                  "<collision name='collision_wall_1'>"
                    "<pose>" << (-width / 2.0 - thickness / 2.0) << " 0 " << (height / 2.0) << " 0 0 0</pose>"
                    "<geometry>"
                      "<box>"
                        "<size>" << thickness << " " << depth << " " << height << "</size>"
                      "</box>"
                    "</geometry>"
                    "<surface>"
                      "<friction>"
                        "<ode>"
                          "<mu>" << friction << "</mu>"
                          "<mu2>" << friction2 << "</mu2>"
                        "</ode>"
                        "<bullet>"
                            "<friction>" << friction << "</friction>"
                            "<friction2>" << friction2 << "</friction2>"
                        "</bullet>"
                      "</friction>"
                    "</surface>"
                  "</collision>"
                  "<visual name='visual_wall_1'>"
                    "<pose>" << (-width / 2.0 - thickness / 2.0) << " 0 " << (height / 2.0) << " 0 0 0</pose>"
                    "<geometry>"
                      "<box>"
                        "<size>" << thickness << " " << depth << " " << height << "</size>"
                      "</box>"
                    "</geometry>"
                    "<material>"
                      "<script>"
                        "<name>Gazebo/White</name>"
                        "<uri>file://media/materials/scripts/gazebo.material</uri>"
                      "</script>"
                    "</material>"
                  "</visual>"
                  "<!-- /Wall 1 -->"
                  
                  "<!-- Wall 2 -->"
                  "<collision name='collision_wall_2'>"
                    "<pose>" << (width / 2.0 + thickness / 2.0) << " 0 " << (height / 2.0) << " 0 0 0</pose>"
                    "<geometry>"
                      "<box>"
                        "<size>" << thickness << " " << depth << " " << height << "</size>"
                      "</box>"
                    "</geometry>"
                    "<surface>"
                      "<friction>"
                        "<ode>"
                          "<mu>" << friction << "</mu>"
                          "<mu2>" << friction2 << "</mu2>"
                        "</ode>"
                        "<bullet>"
                            "<friction>" << friction << "</friction>"
                            "<friction2>" << friction2 << "</friction2>"
                        "</bullet>"
                      "</friction>"
                    "</surface>"
                  "</collision>"
                  "<visual name='visual_wall_2'>"
                    "<pose>" << (width / 2.0 + thickness / 2.0) << " 0 " << (height / 2.0) << " 0 0 0</pose>"
                    "<geometry>"
                      "<box>"
                        "<size>" << thickness << " " << depth << " " << height << "</size>"
                      "</box>"
                    "</geometry>"
                    "<material>"
                      "<script>"
                        "<name>Gazebo/White</name>"
                        "<uri>file://media/materials/scripts/gazebo.material</uri>"
                      "</script>"
                    "</material>"
                  "</visual>"
                  "<!-- /Wall 2 -->"
                  
                  "<!-- Wall 3 -->"
                  "<collision name='collision_wall_3'>"
                    "<pose>0 " << (depth / -2.0 + thickness / 2.0) << " " << (height / 2.0) << " 0 0 0</pose>"
                    "<geometry>"
                      "<box>"
                        "<size>" << width << " " << thickness << " " << height << "</size>"
                      "</box>"
                    "</geometry>"
                    "<surface>"
                      "<friction>"
                        "<ode>"
                          "<mu>" << friction << "</mu>"
                          "<mu2>" << friction2 << "</mu2>"
                        "</ode>"
                        "<bullet>"
                            "<friction>" << friction << "</friction>"
                            "<friction2>" << friction2 << "</friction2>"
                        "</bullet>"
                      "</friction>"
                    "</surface>"
                  "</collision>"
                  "<visual name='visual_wall_3'>"
                    "<pose>0 " << (depth / -2.0 + thickness / 2.0) << " " << (height / 2.0) << " 0 0 0</pose>"
                    "<geometry>"
                      "<box>"
                        "<size>" << width << " " << thickness << " " << height << "</size>"
                      "</box>"
                    "</geometry>"
                    "<material>"
                      "<script>"
                        "<name>Gazebo/White</name>"
                        "<uri>file://media/materials/scripts/gazebo.material</uri>"
                      "</script>"
                    "</material>"
                  "</visual>"
                  "<!-- /Wall 3 -->"
                  
                  "<!-- Wall 4 -->"
                  "<collision name='collision_wall_4'>"
                    "<pose>0 " << (depth / 2.0 - thickness / 2.0) << " " << (height / 2.0) << " 0 0 0</pose>"
                    "<geometry>"
                      "<box>"
                        "<size>" << width << " " << thickness << " " << height << "</size>"
                      "</box>"
                    "</geometry>"
                    "<surface>"
                      "<friction>"
                        "<ode>"
                          "<mu>" << friction << "</mu>"
                          "<mu2>" << friction2 << "</mu2>"
                        "</ode>"
                        "<bullet>"
                            "<friction>" << friction << "</friction>"
                            "<friction2>" << friction2 << "</friction2>"
                        "</bullet>"
                      "</friction>"
                    "</surface>"
                  "</collision>"
                  "<visual name='visual_wall_4'>"
                    "<pose>0 " << (depth / 2.0 - thickness / 2.0) << " " << (height / 2.0) << " 0 0 0</pose>"
                    "<geometry>"
                      "<box>"
                        "<size>" << width << " " << thickness << " " << height << "</size>"
                      "</box>"
                    "</geometry>"
                    "<material>"
                      "<script>"
                        "<name>Gazebo/White</name>"
                        "<uri>file://media/materials/scripts/gazebo.material</uri>"
                      "</script>"
                    "</material>"
                  "</visual>"
                  "<!-- /Wall 4 -->"
              "</link>"
            "</model>"
          "</sdf>";
    
    // Create SDF from the XML string
    sdf::SDF modelSDF;
    modelSDF.SetFromString(xml.str());
    
    // Insert the SDF into the world in runtime
    _parent->InsertModelSDF(modelSDF);
}


