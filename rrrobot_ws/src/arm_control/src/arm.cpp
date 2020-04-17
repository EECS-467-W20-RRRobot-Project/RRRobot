#include <arm.h>

#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include <LinearMath/btTransform.h>
#include <kdl/frames_io.hpp>

#include <iostream>
#include <sstream>
#include <unordered_map>

using std::cout;
using std::endl;
using std::string;
using std::stringstream;
using std::unordered_map;
using std::vector;

struct FrameInformation
{
    string link_name;       // link name
    KDL::Joint joint;       // joint information (name, type)
    KDL::Frame link_frame;  // link location (world coordinates)
    KDL::Frame joint_frame; // joint location relative to parent(link_name)
    float mass;
    KDL::Vector com_location; // relative to frame origin
    KDL::RotationalInertia rotational_inertia;
};

/*
 * sdf_file: this is the sdf file that defines the model. This
 *      class will use that description of the robot to model 
 *      its dynamics.
 */
Arm::Arm(const std::string &sdf_file)
{
    // read sdf file
    sdf::SDFPtr sdf(new sdf::SDF());
    sdf::init(sdf);
    sdf::readFile(sdf_file, sdf);

    // get model
    const sdf::ElementPtr root = sdf->Root();
    const sdf::ElementPtr model = root->GetElement("model");
    const string model_name = model->Get<string>("name");
    cout << "Found " << model_name << " model" << endl;

    unordered_map<string, FrameInformation> links;
    unordered_map<string, string> link_ordering;
    string first_link;

    // start reading links and joints
    sdf::ElementPtr link = model->GetElement("link");
    while (link)
    {
        const string name(link->Get<string>("name"));
        links[name].link_name = name;

        // Transformation from world to link coordinates
        stringstream frame_location(link->GetElement("pose")->GetValue()->GetAsString());
        float x, y, z;
        float roll, pitch, yaw;
        frame_location >> x >> y >> z >> roll >> pitch >> yaw;
        KDL::Rotation rotation = KDL::Rotation::RPY(roll, pitch, yaw);
        KDL::Vector link_position(x, y, z);
        links[name].link_frame = KDL::Frame(rotation, link_position);

        const sdf::ElementPtr inertial_data = link->GetElement("inertial");
        float mass = inertial_data->Get<float>("mass");
        links[name].mass = mass;

        // Get the center of mass for this link
        stringstream inertial_data_ss(inertial_data->GetElement("pose")->GetValue()->GetAsString());
        inertial_data_ss >> links[name].com_location[0] >> links[name].com_location[1] >> links[name].com_location[2];
        links[name].com_location += links[name].link_frame.p;

        const sdf::ElementPtr inertia = inertial_data->GetElement("inertia");
        links[name].rotational_inertia = KDL::RotationalInertia(
            inertia->Get<float>("ixx"),
            inertia->Get<float>("iyy"),
            inertia->Get<float>("izz"),
            inertia->Get<float>("ixy"),
            inertia->Get<float>("ixz"),
            inertia->Get<float>("iyz"));

        link = link->GetNextElement("link");
    }

    sdf::ElementPtr joint = model->GetElement("joint");
    while (joint) // TODO(dwitcpa): add support for translational joints
    {
        const string name(joint->Get<string>("name"));
        const string parent(joint->GetElement("parent")->GetValue()->GetAsString());
        const string child(joint->GetElement("child")->GetValue()->GetAsString());
        link_ordering[parent] = child;
        if (parent == string("world"))
        {
            joint = joint->GetNextElement("joint");
            continue;
        }

        stringstream axis(joint->GetElement("axis")->GetElement("xyz")->GetValue()->GetAsString());

        int axis_num = 0;
        int cur;
        while (axis >> cur)
        {
            if (cur != 0)
                break;
            axis_num++;
        }
        KDL::Joint::JointType joint_type = KDL::Joint::JointType::RotAxis;
        if (axis_num == 0)
            joint_type = KDL::Joint::JointType::RotX;
        else if (axis_num == 1)
            joint_type = KDL::Joint::JointType::RotY;
        else if (axis_num == 2)
            joint_type = KDL::Joint::JointType::RotZ;

        links[child].joint = KDL::Joint(name, joint_type);

        stringstream pose_string(joint->GetElement("pose")->GetValue()->GetAsString());
        float x, y, z;
        float roll, pitch, yaw;
        pose_string >> x >> y >> z >> roll >> pitch >> yaw;

        KDL::Rotation frame_rotation = KDL::Rotation::RPY(roll, pitch, yaw);
        KDL::Vector frame_location(x, y, z);
        links[child].joint_frame = KDL::Frame(frame_rotation, frame_location);

        joint = joint->GetNextElement("joint");
    }

    string cur_link_name = "world";
    KDL::Frame prev_frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(0, 0, 0));
    KDL::Joint::JointType to_set(KDL::Joint::JointType::None);

    while (link_ordering.find(cur_link_name) != link_ordering.end())
    {
        cur_link_name = link_ordering[cur_link_name];

        // get the world coordinates of the joint
        KDL::Frame joint_in_world = links[cur_link_name].link_frame * links[cur_link_name].joint_frame;
        KDL::Frame prev_frame_inverse = prev_frame.Inverse();
        KDL::Frame joint_relative_to_prev = prev_frame.Inverse() * joint_in_world;

        KDL::Vector com_in_prev = joint_relative_to_prev * links[cur_link_name].com_location;

        KDL::Joint::JointType next(links[cur_link_name].joint.getType());
        KDL::RigidBodyInertia inertia(links[cur_link_name].mass, com_in_prev /*links[cur_link_name].com_location*/, links[cur_link_name].rotational_inertia);
        KDL::Segment to_add(links[cur_link_name].link_name, KDL::Joint(links[cur_link_name].joint.getName(), to_set), joint_relative_to_prev, inertia);
        to_set = next;

        arm.addSegment(to_add);

        prev_frame = joint_in_world;
    }

    arm.addSegment(KDL::Segment(string("pseudo_link"), KDL::Joint("pseudo_joint", to_set), KDL::Frame::Identity(), KDL::RigidBodyInertia()));

    cout << "# of arm segments: " << arm.getNrOfSegments() << "\t# of arm joints: " << arm.getNrOfJoints() << endl;
    fksolver.reset(new KDL::ChainFkSolverPos_recursive(arm));
    iksolver.reset(new KDL::ChainIkSolverPos_LMA(arm));
    idsolver.reset(new KDL::ChainIdSolver_RNE(arm, KDL::Vector(0, 0, -9.81)));
}

/*
 * Using the provided joint positions, this calculates the pose of the end of the
 * arm. Returns whether it was successful
 */
bool Arm::calculateForwardKinematics(KDL::JntArray joint_positions, KDL::Frame &end_effector_frame)
{
    return fksolver->JntToCart(joint_positions, end_effector_frame);
}

bool Arm::calculateInverseKinematics(const KDL::JntArray &cur_configuration, const KDL::Frame &desired_end_effector_frame, KDL::JntArray &final_configuration)
{
    return iksolver->CartToJnt(cur_configuration, desired_end_effector_frame, final_configuration);
}

int Arm::calculateInverseDynamics(const KDL::JntArray &pos, const KDL::JntArray &vel, const KDL::JntArray &accel, const KDL::Wrenches &f_external, KDL::JntArray &required_torque)
{
    return idsolver->CartToJnt(pos, vel, accel, f_external, required_torque);
}

const KDL::Chain &Arm::getArm() const
{
    return arm;
}

void Arm::print() const
{
    for (const KDL::Segment &seg : arm.segments)
    {
        cout << seg.getName() << endl;
        cout << seg.getFrameToTip() << endl;
        cout << seg.getJoint().getTypeName() << endl;
        cout << "Inertial frame:" << endl;
        cout << "\tmass: " << seg.getInertia().getMass() << endl;
        cout << '\t' << seg.getInertia().getCOG() << endl;
        cout << endl;
    }
}