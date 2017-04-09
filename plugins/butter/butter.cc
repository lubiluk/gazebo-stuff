#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

using namespace std;

namespace gazebo {
    class ButterFactoryPlugin : public WorldPlugin {
    public:
        void Load(physics::WorldPtr parent, sdf::ElementPtr sdf) {
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

            xml << "<sdf version ='1.4'>";
            xml << "<model name ='butter'>";
            xml << "<pose>" << poseArg << "</pose>";

            double xShift = -xlen * radius;
            double yShift = -ylen * radius;
            double zShift = -zlen * radius;

            for (int i = 0; i < xlen; ++i) {
                for (int j = 0; j < ylen; ++j) {
                    for (int k = 0; k < zlen; ++k) {
                        xml << "<link name='link_" << i << "_" << j << "_" << k << "'>"
                                    "<pose>" << radius * 2 * i + xShift << " " << radius * 2 * j + yShift << " " << radius * 2 * k + zShift << " 0 0 0</pose>"
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
                                    "<visual name ='visual_" << i << "_" << j << "_" << k << "'>"
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

            for (int i = 0; i < xlen; ++i) {
                for (int j = 0; j < ylen; ++j) {
                    for (int k = 0; k < zlen; ++k) {
                        string current = to_string(i) + "_" + to_string(j) + "_" + to_string(k);
                        string force = "0.5";

                        if (i > 0) {
                            string previous = to_string(i-1) + "_" + to_string(j) + "_" + to_string(k);

                            xml << "<joint name=\"joint_" << current << "_" << previous << "\" type=\"revolute\">\
                                    <parent>link_" << current << "</parent>\
                                    <child>link_" << previous << "</child>\
                                    <axis>\
                                        <xyz>0 0 1</xyz>\
                                        <limit>\
                                        <lower>0.0</lower>\
                                        <upper>0.0</upper>\
                                        </limit>\
                                    </axis>\
                                    <physics>\
                                        <ode>\
                                            <erp>1</erp>\
                                            <cfm>1</cfm>\
                                        </ode>\
                                    </physics>\
                                    <sensor name=\"force_torque\" type=\"force_torque\">\
                                          <always_on>true</always_on>\
                                          <update_rate>1000</update_rate>\
                                          <plugin name=\"breakable_0_0\" filename=\"libBreakableJointPlugin.so\">\
                                                <breaking_force_N>" << force << "</breaking_force_N>\
                                          </plugin>\
                                    </sensor>\
                               </joint>";
                        }

                        if (j > 0) {
                            string previous = to_string(i) + "_" + to_string(j-1) + "_" + to_string(k);

                            xml << "<joint name=\"joint_" << current << "_" << previous << "\" type=\"revolute\">\
                                    <parent>link_" << current << "</parent>\
                                    <child>link_" << previous << "</child>\
                                    <axis>\
                                        <xyz>0 0 1</xyz>\
                                        <limit>\
                                        <lower>0.0</lower>\
                                        <upper>0.0</upper>\
                                        </limit>\
                                    </axis>\
                                    <physics>\
                                        <ode>\
                                            <erp>1</erp>\
                                            <cfm>1</cfm>\
                                        </ode>\
                                    </physics>\
                                    <sensor name=\"force_torque\" type=\"force_torque\">\
                                          <always_on>true</always_on>\
                                          <update_rate>1000</update_rate>\
                                          <plugin name=\"breakable_0_0\" filename=\"libBreakableJointPlugin.so\">\
                                                <breaking_force_N>" << force << "</breaking_force_N>\
                                          </plugin>\
                                    </sensor>\
                               </joint>";
                        }

                        if (k > 0) {
                            string previous = to_string(i) + "_" + to_string(j) + "_" + to_string(k-1);

                            xml << "<joint name=\"joint_" << current << "_" << previous << "\" type=\"revolute\">\
                                    <parent>link_" << current << "</parent>\
                                    <child>link_" << previous << "</child>\
                                    <axis>\
                                        <xyz>0 0 1</xyz>\
                                        <limit>\
                                        <lower>0.0</lower>\
                                        <upper>0.0</upper>\
                                        </limit>\
                                    </axis>\
                                    <physics>\
                                        <ode>\
                                            <erp>1</erp>\
                                            <cfm>1</cfm>\
                                        </ode>\
                                    </physics>\
                                    <sensor name=\"force_torque\" type=\"force_torque\">\
                                          <always_on>true</always_on>\
                                          <update_rate>1000</update_rate>\
                                          <plugin name=\"breakable_0_0\" filename=\"libBreakableJointPlugin.so\">\
                                                <breaking_force_N>" << force << "</breaking_force_N>\
                                          </plugin>\
                                    </sensor>\
                               </joint>";
                        }
                    }
                }
            }

            xml << "</model>";
            xml << "</sdf>";

            sdf::SDF butterSDF;
            butterSDF.SetFromString(xml.str());

            parent->InsertModelSDF(butterSDF);
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(ButterFactoryPlugin)
}
