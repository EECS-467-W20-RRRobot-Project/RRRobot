#include <arm.h>

#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include <LinearMath/btTransform.h>

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
    string link_name;
    KDF::Joint joint;
    KDF::Frame link_frame;
    KDF::Frame joint_frame;
    RigidBodyInertia inertia;
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

    unordered_map<string, KDL::Segment> links;
    unordered_map<string, string> link_ordering;
    string first_link;

    // start reading links and joints
    sdf::ElementPtr link = model->GetElement("link");
    while (link)
    {
        cout << "Link: " << link->Get<string>("name") << endl;

        const string name(link->Get<string>("name"));
        KDL::Joint cur_joint(KDL::Joint::None);

        const sdf::ElementPtr inertial_data = link->GetElement("inertial");
        float mass = inertial_data->Get<float>("mass");
        cout << "Mass: " << mass << endl;

        string inertial_data_s = inertial_data->GetElement("pose")->GetValue()->GetAsString();
        stringstream inertial_data_ss(inertial_data_s);
        KDL::Vector inertial_frame_info;

        inertial_data_ss >> inertial_frame_info.data[0];
        inertial_data_ss >> inertial_frame_info.data[1];
        inertial_data_ss >> inertial_frame_info.data[2];

        const sdf::ElementPtr inertia = inertial_data->GetElement("inertia");
        KDL::RotationalInertia rotational_inertia(
            inertia->Get<float>("ixx"),
            inertia->Get<float>("iyy"),
            inertia->Get<float>("izz"),
            inertia->Get<float>("ixy"),
            inertia->Get<float>("ixz"),
            inertia->Get<float>("iyz"));

        KDL::RigidBodyInertia link_inertia(mass, inertial_frame_info, rotational_inertia);

        // Transformation from world to link coordinates
        // stringstream frame_location(link->GetElement("pose")->GetValue()->GetAsString());
        // float x, y, z;
        // float roll, pitch, yaw;
        // frame_location >> x >> y >> z >> roll >> pitch >> yaw;
        // KDL::Rotation rotation = KDL::Rotation::RPY(roll, pitch, yaw);
        // KDL::Vector link_position(x, y, z);
        // KDL::Frame frame(rotation, link_position);

        KDL::Segment cur_link(name, cur_joint, KDL::Frame::Identity(), link_inertia);
        //arm.addSegment(cur_link);
        links[name] = cur_link;

        // cout << inertia->Get<float>("ixx") << endl;
        // cout << inertia->Get<float>("iyy") << endl;
        // cout << inertia->Get<float>("izz") << endl;
        // cout << inertia->Get<float>("ixy") << endl;
        // cout << inertia->Get<float>("ixz") << endl;
        // cout << inertia->Get<float>("iyz") << endl;

        // inertial_data->GetElement("pose")->PrintValues(""); //PrintValues("");
        // cout << inertial_data->GetElement("pose")->GetAttributeCount() << endl;
        // const sdf::ParamPtr pose = inertial_data->GetElement("pose")->GetValue();
        // const sdf::ElementPtr pose_elt = inertial_data->GetElement("pose");
        // cout << pose->GetAsString() << endl;
        // // cout << pose->IsType<ignition::math::Pose3>() << endl;
        // cout << pose << endl;
        // cout << *pose_elt->GetValue() << endl;
        // sdf::ElementPtr cur = pose_elt->GetFirstElement();
        // while (cur)
        // {
        //     cout << cur->Get<float>() << endl;

        //     cur = cur->GetNextElement();
        // }

        //ignition::math::Pose3 inertial_pose = inertial_data->Get<ignition::math::Pose3>("pose");
        //cout << "Inertial pose: " << inertial_pose << endl; //inertial_pose.pos.x << '\t' << inertial_pose.pos.y << '\t' <<  << endl;
        // float data;
        // while (data << inertial_pose)
        // {
        //     cout << data << '\t';
        // }
        // cout << endl;
        // KDL::RigidBodyInertia(
        //     inertial_data->Get<float>("mass"),

        // );
        // KDL::Segment
        // current_link()
        //     arm.addSegment()

        link = link->GetNextElement("link");
    }

    cout << "# of arm segments: " << arm.getNrOfSegments() << "\t# of arm joints: " << arm.getNrOfJoints() << endl;

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
            continue; //first_link = child;
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

        KDL::Joint cur_joint(name, joint_type);

        stringstream pose_string(joint->GetElement("pose")->GetValue()->GetAsString());
        float x, y, z;
        float roll, pitch, yaw;
        pose_string >> x >> y >> z >> roll >> pitch >> yaw;

        KDL::Rotation frame_rotation = KDL::Rotation::RPY(roll, pitch, yaw);
        KDL::Vector frame_location(x, y, z);

        links[child] = KDL::Segment(links[child].getName(), cur_joint, KDL::Frame(frame_rotation, frame_location), links[child].getInertia());
        //KDL::Segment cur_segment = arm.getSegment

        joint = joint->GetNextElement("joint");
    }

    string cur_link_name = "world";
    KDL::Frame prev_frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(0, 0, 0));
    // btTransform prev_frame_world_transform(btMatrix3x3::getIdentity());
    while (link_ordering.find(cur_link_name) != link_ordering.end())
    {
        cur_link_name = link_ordering[cur_link_name];

        const KDL::Segment &cur = links[cur_link_name];
        const KDL::Frame &cur_frame = cur.getFrameToTip();
        double cur_roll, cur_pitch, cur_yaw;
        cur_frame.M.GetRPY(cur_roll, cur_pitch, cur_yaw);
        double prev_roll, prev_pitch, prev_yaw;
        prev_frame.M.GetRPY(prev_roll, prev_pitch, prev_yaw);
        KDL::Rotation new_rotation = KDL::Rotation::RPY(cur_roll - prev_roll, cur_pitch - prev_pitch, cur_yaw - prev_yaw);
        double roll, pitch, yaw;
        new_rotation.GetRPY(roll, pitch, yaw);

        KDL::Vector cur_pos = cur_frame.p;
        KDL::Vector prev_pos = prev_frame.p;
        KDL::Vector new_position(cur_pos.x() /* - prev_pos.x()*/, cur_pos.y() /*  - prev_pos.y()*/, cur_pos.z() /* - prev_pos.z()*/);

        // const KDL::Rotation &cur_rotation = cur_frame.M;
        // const KDL::Vector &cur_translation = cur_frame.p;
        // const KDL::Vector &rot_x = cur_rotation.UnitX();
        // const KDL::Vector &rot_y = cur_rotation.UnitY();
        // const KDL::Vector &rot_z = cur_rotation.UnitZ();

        // btTransform cur_frame_world_transform(
        //     btMatrix3x3(rot_x[0], rot_y[0], rot_z[0],
        //                 rot_x[1], rot_y[1], rot_z[1],
        //                 rot_x[2], rot_y[2], rot_z[2]),
        //     btVector3(btScalar(cur_translation.x()),
        //               btScalar(cur_translation.y()),
        //               btScalar(cur_translation.z())));
        // btTransform transform;
        // //transform.mult(prev_frame_world_transform, cur_frame_world_transform /*.inverse()*/);
        // transform = cur_frame float roll, pitch, yaw;
        // transform.getRotation().getEulerZYX(yaw, pitch, roll);
        // KDL::Rotation rot(KDL::Rotation::RPY(roll, pitch, yaw));
        // btVector3 translation = transform.getOrigin();
        // KDL::Vector trans((float)translation.getX(), (float)translation.getY(), (float)translation.getZ());

        cout << cur_link_name << endl;
        // cout << "\tprev_frame world transform (";

        cout << "\tOrigin position: " << new_position.x() << ", " << new_position.y() << ", " << new_position.z() << endl;
        cout << "\tFrame rotation (roll, pitch, yaw): (" << roll << ", " << pitch << ", " << yaw << ")" << endl;

        KDL::Segment to_add(cur.getName(), cur.getJoint(), KDL::Frame(new_rotation, new_position), cur.getInertia());
        arm.addSegment(to_add);

        prev_frame = cur_frame;
        // prev_frame_world_transform = cur_frame_world_transform;
    }

    cout << arm.getNrOfJoints() << '\t' << arm.getNrOfSegments() << endl;
}

/*
 * Gets the center of mass of the remaining arm, starting from the
 * link in index (ie the center of mass of links l[i]-l[n])
 */
KDL::Vector Arm::getCOM(size_t index)
{
}

/*
 * Gets the center of mass of the remaining arm, starting from the
 * link named 'link_name'
 */
KDL::Vector Arm::getCOM(const std::string &link_name)
{
}

/*
 * Gets the inertia of the remaining arm about the joint in index. This
 * treats the preceding segments as if they are stationary and calculates
 * the inertia of the remaining arm.
 */
float Arm::getInertia(size_t index)
{
}

/*
 * Gets the inertia of the remaining arm about the joint named 'joint_name'. This
 * treats the preceding segments as if they are stationary and calculates
 * the inertia of the remaining arm.
 */
float Arm::getInertia(const std::string &joint_name)
{
}

/*
 * This returns the mass of the remaining links, starting from (and including)
 * the link at index
 */
float Arm::getSupportedMass(size_t index)
{
}

/*
 * This returns the mass of the remaining links, starting from (and including)
 * the link named 'link_name'
 */
float Arm::getSupportedMass(const std::string &link_name)
{
}

/*
 * This returns the torques required to get the arm moving with the desired 
 * angular acceleration.
 */
std::vector<float> Arm::getRequiredTorques(/*std::vector<float> theta_des, std::vector<float> theta_dot_des, */ std::vector<float> theta_double_dot_des)
{
}

const KDL::Chain &Arm::getArm() const
{
    return arm;
}

/*
 * i_com: the inertia about the center of mass of this link
 * mass: the mass of this link
 * distance: the distance from the center of mass to the new axis
 */
float Arm::parallelAxisTheorem(float i_com, float mass, float distance)
{
}