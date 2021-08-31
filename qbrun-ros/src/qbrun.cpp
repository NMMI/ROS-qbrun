// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include <math.h>
#include <string>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

// qbRobotics custom services
#include <qb_device_srvs/Trigger.h>
#include <qb_device_srvs/SetCommands.h>
#include <qb_device_srvs/InitializeDevice.h>
#include <qb_device_srvs/GetMeasurements.h>

#define RAD2TICK 65536/(4*M_PI)
#define TICK2RAD (4*M_PI)/65536

using namespace std;

// Variables
qb_device_srvs::InitializeDevice reg;
qb_device_srvs::Trigger dea;
qb_device_srvs::Trigger act;
qb_device_srvs::SetCommands scmd;
qb_device_srvs::GetMeasurements gmeas;
ros::Subscriber sub_cmd1;
ros::Subscriber sub_cmd2;
ros::Publisher pub_state1, pub_state2, pub_state_link;
std_msgs::Float64MultiArray cmd1, cmd2;
ros::Time curr_time, prev_time;

// Compute commands in tick from stiffness and equilibrium position
short int compute_cube_motor_position(double eq, double stiff, short int& th1, short int& th2)
{
    th1 = (short int)((eq + stiff) * 32768.0 / (2* M_PI));
    th2 = (short int)((eq - stiff) * 32768.0 / (2* M_PI));
}

// Define a zeros matrix of dimension dim
short int zeros(int dim, short int Mat[][2])
{
    // Set zero all the initial commands
    for (int i=0; i<dim; i++) {
        Mat[i][0] = 0;
        Mat[i][1] = 0;
    }
}

// Subscriber callbacks
void getCmd1_callback(const std_msgs::Float64MultiArray& pos)
{
    cmd1 = pos;
}
void getCmd2_callback(const std_msgs::Float64MultiArray& pos)
{
    cmd2 = pos;
}

// Compute motor velocity
double compute_vel(double time, double th, double th_old){
    return (th - th_old)/time;
}

// Main fuction
int main(int argc, char **argv) {
    ros::init(argc, argv, "qbRun");
    ros::NodeHandle n;
    int n_cubes;
    double T_sample;
    bool flag_rad;
    std::vector<int> IDs;
    std::string ns_name;
    sensor_msgs::JointState state_1, state_2, state_link;
    sensor_msgs::JointState state_1_old, state_2_old, state_link_old;
    std::string ref1_topic_name, ref2_topic_name;
    std::string state1_topic_name, state2_topic_name, robot_state_topic_name;

    // Get number of cubes and stiffness from external ROS .confg file and resize variables
    n.getParam("IDs", IDs);
    n.getParam("T_sample", T_sample);
    n.getParam("flag_rad", flag_rad);
    n.getParam("namespace", ns_name);
    ros::Rate fs(1 / T_sample);

    // Initialize or resize variables
    n_cubes = IDs.size();
    short int cmd[n_cubes][2];
    zeros(n_cubes, cmd);
    cmd1.data.resize(n_cubes);
    cmd2.data.resize(n_cubes);
    // first and secondo motor
    state_1.name.resize(n_cubes);
    state_2.name.resize(n_cubes);
    state_1.position.resize(n_cubes);
    state_2.position.resize(n_cubes);
    state_1_old.position.resize(n_cubes);
    state_2_old.position.resize(n_cubes);
    state_1.velocity.resize(n_cubes);
    state_2.velocity.resize(n_cubes);
    state_1.effort.resize(n_cubes);
    state_2.effort.resize(n_cubes);
    // link
    state_link.name.resize(n_cubes);
    state_link.position.resize(n_cubes);
    state_link.velocity.resize(n_cubes);
    state_link.effort.resize(n_cubes);
    state_link_old.position.resize(n_cubes);

    // Initialize command values to zero
    for (int i = 0; i < n_cubes; ++i)
    {
        cmd1.data[i] = 0;
        cmd2.data[i] = 0;
    }

    // Subscribe to the commands
    ref1_topic_name = "/" +  ns_name + "/reference_1";
    ref2_topic_name = "/" +  ns_name + "/reference_2";
    sub_cmd1 = n.subscribe(ref1_topic_name,10, &getCmd1_callback);
    sub_cmd2 = n.subscribe(ref2_topic_name,10, &getCmd2_callback);

    // Publish the motors' state
    state1_topic_name = "/" +  ns_name + "/motor_1_state";
    state2_topic_name = "/" +  ns_name + "/motor_2_state";
    robot_state_topic_name = "/" +  ns_name + "/robot_state";
    pub_state1 = n.advertise<sensor_msgs::JointState>(state1_topic_name, 10);
    pub_state2 = n.advertise<sensor_msgs::JointState>(state2_topic_name, 10);
    pub_state_link = n.advertise<sensor_msgs::JointState>(robot_state_topic_name, 10);

    // Call the service to register the devices and activation
    for (int i = 0; i < n_cubes; i++) {
        ros::ServiceClient client_reg = n.serviceClient<qb_device_srvs::InitializeDevice>("/communication_handler/initialize_device");
        reg.request.id = IDs[i];
        reg.request.activate = true;
        client_reg.call(reg);
    }

    // Call the service for set commands
    ros::ServiceClient client_cmd = n.serviceClient<qb_device_srvs::SetCommands>("/communication_handler/set_commands");

    // Call the service for get measurements
    ros::ServiceClient client_meas = n.serviceClient<qb_device_srvs::GetMeasurements>("/communication_handler/get_measurements");

    while (ros::ok())
    {
        curr_time = ros::Time::now();
        // compute cycle time
        double delta_time = ( curr_time.toSec() - prev_time.toSec() );

        // Call the service to set command to each actuator
        std::cout << "\r[COMMANDS] [" + std::to_string(curr_time.toSec()) + "]\t";
        for (int i = 0; i < n_cubes; i++) {
     
            scmd.request.id = IDs[i];
            scmd.request.set_commands = true;
            scmd.request.set_commands_async = true;

            // Position passed directly as prime mover references in rad or tick
            if (flag_rad)
            {
                cmd[i][0] = (short int)(cmd1.data[i]*RAD2TICK);
                cmd[i][1] = (short int)(cmd2.data[i]*RAD2TICK);
            }
            else
            {
                cmd[i][0] = (short int)cmd1.data[i];
                cmd[i][1] = (short int)cmd2.data[i];                
            }

            scmd.request.commands.resize(2);
            for (int j=0; j<2; j++) {
                scmd.request.commands.at(j) = cmd[i][j];
            }
            std::cout << "ID-" << IDs[i] << " = ["<< cmd[i][0] << "," << cmd[i][1] << "]\t";
            client_cmd.call(scmd);
        }

        // Call the service to get the motor measurements
        for (int i = 0; i < n_cubes; i++) {

            gmeas.request.id = IDs[i];
            gmeas.request.get_positions = true;
            gmeas.request.get_currents = true;
            gmeas.request.get_distinct_packages = true;

            client_meas.call(gmeas);

            // assign stamp time 
            ros::Time act_time = ros::Time::now();
            state_1.header.stamp = act_time;
            state_2.header.stamp = act_time;
            state_link.header.stamp = act_time;

            // assign names
            state_1.name[i] = "ID-" + std::to_string(IDs[i]);
            state_2.name[i] = "ID-" + std::to_string(IDs[i]);
            state_link.name[i] = "ID-" + std::to_string(IDs[i]);

            // assign positions
            std::vector<short int> pos_temp;
            pos_temp.resize(3);
            pos_temp = gmeas.response.positions;
            state_1.position[i] = pos_temp[0]*TICK2RAD;
            state_2.position[i] = pos_temp[1]*TICK2RAD;
            state_link.position[i] = pos_temp[2]*TICK2RAD;
            
            // compute and assign velocities
            state_1.velocity[i] = compute_vel(delta_time, state_1.position[i], state_1_old.position[i]);
            state_2.velocity[i] = compute_vel(delta_time, state_2.position[i], state_2_old.position[i]);
            state_link.velocity[i] = compute_vel(delta_time, state_link.position[i], state_link_old.position[i]);

            // assign currents
            std::vector<short int> curr_temp;
            curr_temp.resize(2);
            curr_temp = gmeas.response.currents;
            state_1.effort[i] = curr_temp[0];
            state_2.effort[i] = curr_temp[1];

            // Return the elastic torque on the link estimated from the elastic models
            double a = 8.9992;
            double k = 0.0019;
            double tau_el1 = k*sinh(a*(state_link.position[i] - state_1.position[i]));
            double tau_el2 = k*sinh(a*(state_link.position[i] - state_2.position[i]));
            
            state_link.effort[i] = tau_el1 + tau_el2;

            std::cout << "POS-" << std::to_string(IDs[i]) << " = [" << state_link.position[i] << ", "<< state_1.position[i] << ", " << state_2.position[i] << "]\t";

        }
        std::cout << std::endl;

        // Publish motor states
        pub_state1.publish(state_1);
        pub_state2.publish(state_2);
        pub_state_link.publish(state_link);

        // Update old values
        prev_time = curr_time;
        state_1_old = state_1;
        state_2_old = state_2;
        state_link_old = state_link;

        fs.sleep();
        ros::spinOnce();
    }

    // Deactivation of all the cubes
    for (int i = 0; i < n_cubes; i++) {
        do{
            ros::ServiceClient client_dea = n.serviceClient<qb_device_srvs::Trigger>("/communication_handler/deactivate_motors");
            dea.request.id = IDs[i];
            client_dea.call(dea);
            std::cout << dea.response.success;
        }
        while(dea.response.success);
    }

    ros::waitForShutdown();

    return 0;
}
