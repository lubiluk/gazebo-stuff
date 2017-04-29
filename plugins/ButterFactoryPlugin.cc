#include "ButterFactoryPlugin.hh"

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

    xml << "<sdf version ='1.4'>"
            "<model name ='butter'>"
            "<pose>" << poseArg << "</pose>";

    double xShift = -xlen * radius;
    double yShift = -ylen * radius;
    double zShift = -zlen * radius;

    // Force at which the butter particles should break apart
    string force = "0.05";

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
                            "<mass>0.003</mass>"
                            "</inertial>"
                            "<collision name ='collision_" << i << "_" << j << "_" << k << "'>"
                            "<geometry>"
                            "<sphere>"
                            "<radius>" << radius << "</radius>"
                            "</sphere>"
                            "</geometry>"
                            "<surface>"
                            "<friction>"
                            "<ode>"
                            "<mu>20</mu>"
                            "<mu2>20</mu2>"
                            "</ode>"
                            "</friction>"
                            "<contact>"
                            "<ode>"
                            "<soft_cfm>2</soft_cfm>"
                            "<soft_erp>0</soft_erp>"
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
                            "<self_collide>true</self_collide>"
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

                    xml << GetBreakableJointString(current, previous, force);
                }

                if (j > 0) {
                    string previous = to_string(i) + "_" + to_string(j - 1) + "_" + to_string(k);

                    xml << GetBreakableJointString(current, previous, force);
                }

                if (k > 0) {
                    string previous = to_string(i) + "_" + to_string(j) + "_" + to_string(k - 1);

                    xml << GetBreakableJointString(current, previous, force);
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

string ButterFactoryPlugin::GetBreakableJointString(string linkName1, string linkName2, string force) {
    stringstream ss;
    ss << "<joint name='joint_" << linkName1 << "_" << linkName2 << "' type='revolute'>"
                        "<parent>link_" << linkName1 << "</parent>"
                        "<child>link_" << linkName2 << "</child>"
                        "<axis>"
                            "<xyz>0 0 1</xyz>"
                            "<limit>"
                                "<lower>0.0</lower>"
                                "<upper>0.0</upper>"
                            "</limit>"
                        "</axis>"
                        "<physics>"
                            "<ode>"
                                "<erp>1</erp>"
                                "<cfm>1</cfm>"
                            "</ode>"
                        "</physics>"
            "</joint>";

    return ss.str();
}