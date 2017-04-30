#include "ButterFactoryPlugin.hh"

#include <gazebo/physics/physics.hh>

using namespace gazebo;
using namespace std;

GZ_REGISTER_WORLD_PLUGIN(ButterFactoryPlugin)

void ButterFactoryPlugin::Load(physics::WorldPtr parent, sdf::ElementPtr sdf) {
    string poseArg = sdf->GetElement("pose")->GetValue()->GetAsString();
    istringstream pss(poseArg);

    double x, y, z, pitch, yaw, roll;
    pss >> x >> y >> z >> roll >> pitch >> yaw;

    math::Pose pose(x, y, z, roll, pitch, yaw);

    int xlen = 5;
    int ylen = 5;
    int zlen = 5;
    double radius = 0.01;

    string sizeArg = sdf->GetElement("size")->GetValue()->GetAsString();
    istringstream sss(sizeArg);

    sss >> xlen >> ylen >> zlen >> radius;

    stringstream xml;

    xml << "<sdf version ='1.6'>"
            "<model name ='butter'>"
            "<pose>" << poseArg << "</pose>"
                "<self_collide>true</self_collide>";

    double xShift = -xlen * radius;
    double yShift = -ylen * radius;
    double zShift = -zlen * radius;

    // Force at which the butter particles should break apart
    double mass = 0.005;
    double inertiaDiagonal = 2.0 / 5.0 * mass * radius * radius;

    // Create links
    for (int i = 0; i < xlen; ++i) {
        for (int j = 0; j < ylen; ++j) {
            for (int k = 0; k < zlen; ++k) {
                string index = to_string(i) + "_" + to_string(j) + "_" + to_string(k);

                xml << "<link name='link_" << index << "'>"
                        "<pose>"
                    << radius * 2 * i + xShift << " "
                    << radius * 2 * j + yShift << " "
                    << radius * 2 * k + zShift << " 0 0 0"
                            "</pose>"
                            "<inertial>"
                            "<mass>" << mass << "</mass>"
                            "<inertia>"
                            "<ixx>" << inertiaDiagonal << "</ixx>"
                            "<ixy>0</ixy>"
                            "<ixz>0</ixz>"
                            "<iyy>" << inertiaDiagonal << "</iyy>"
                            "<iyz>0</iyz>"
                            "<izz>" << inertiaDiagonal << "</izz>"
                            "</inertia>"
                            "</inertial>"
                            "<collision name ='collision_" << i << "_" << j << "_" << k << "'>"
                            "<geometry>"
                            "<sphere>"
                            "<radius>" << radius << "</radius>"
                            "</sphere>"
                            "</geometry>"
                            "<surface>"
//                            "<friction>"
//                            "<ode>"
//                            "<mu>20</mu>"
//                            "<mu2>20</mu2>"
//                            "</ode>"
//                            "</friction>"
                            "<contact>"
                            "<ode>"
                            "<soft_cfm>0</soft_cfm>"
                            "<soft_erp>0.8</soft_erp>"
                            "</ode>"
                            "</contact>"
                            "</surface>"
                            "</collision>"
                            "<visual name ='visual_" << index << "'>"
                            "<geometry>"
                            "<sphere>"
                            "<radius>" << radius << "</radius>"
                            "</sphere>"
                            "</geometry>"
                            "<material>"
                            "<script>"
                            "<uri>file://media/materials/scripts/gazebo.material</uri>"
                            "<name>Gazebo/Yellow</name>"
                            "</script>"
                            "</material>"
                            "</visual>"
                            "</link>";
            }
        }
    }

    // Create joints
    for (int i = 0; i < xlen; ++i) {
        for (int j = 0; j < ylen; ++j) {
            for (int k = 0; k < zlen; ++k) {
                string current = to_string(i) + "_" + to_string(j) + "_" + to_string(k);

                if (i > 0) {
                    string previous = to_string(i - 1) + "_" + to_string(j) + "_" + to_string(k);

                    xml << GetJointString(current, previous);
                }

                if (j > 0) {
                    string previous = to_string(i) + "_" + to_string(j - 1) + "_" + to_string(k);

                    xml << GetJointString(current, previous);
                }

                if (k > 0) {
                    string previous = to_string(i) + "_" + to_string(j) + "_" + to_string(k - 1);

                    xml << GetJointString(current, previous);
                }
            }
        }
    }

    xml << "<plugin name='butter' filename='libButterPlugin.so'></plugin>"
            "</model>"
            "</sdf>";

    sdf::SDF butterSDF;
    butterSDF.SetFromString(xml.str());

    parent->InsertModelSDF(butterSDF);
}

string ButterFactoryPlugin::GetJointString(string linkName1, string linkName2) {
    stringstream ss;
    ss << "<joint name='joint_" << linkName1 << "_" << linkName2 << "' type='fixed'>"
            "<parent>link_" << linkName1 << "</parent>"
               "<child>link_" << linkName2 << "</child>"
               "<physics>"
               "<ode>"
               "<erp>0.8</erp>"
               "<cfm>0</cfm>"
               "</ode>"
               "</physics>"
               "</joint>";

    return ss.str();
}