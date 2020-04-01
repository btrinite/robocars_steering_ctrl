/**
 * @file offb_raw_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 source $src_path/Tools/setup_gazebo.bash ${src_path} ${build_path}

 gzserver --verbose ${src_path}/Tools/sitl_gazebo/worlds/${model}.world &
 */
#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>
#include <algorithm> 
#include <cmath>

#include <robocars_msgs/robocars_actuator_output.h>
#include <robocars_msgs/robocars_actuator_ctrl_mode.h>
#include <robocars_msgs/robocars_radio_channels.h>
#include <robocars_msgs/robocars_brain_state.h>

#include <robocars_steering_ctrl.hpp>

RosInterface * ri;

static int command_input_min;
static int command_input_max;
static int command_output_min;
static int command_output_max;
static int loop_hz;

class onRunningMode;
class onIdle;
class onManualDriving;
class onAutonomousDriving;
class onQualibtrateMode;

class onRunningMode
: public RobocarsStateMachine
{
    public:
        onRunningMode() : RobocarsStateMachine("onRunningMode"),__tick_count(0) {};
        onRunningMode(const char * subStateName) : RobocarsStateMachine(subStateName),__tick_count(0) {};


    protected:

        uint32_t __tick_count;
        
        void entry(void) override {
            RobocarsStateMachine::entry();
        };

        void react(ManualDrivingEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react( AutonomousDrivingEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react( IdleStatusEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react(EnterQualibrateModeEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react(TickEvent const & e) override {
            __tick_count++;
            if ((__tick_count%loop_hz)==0) {
                //Update param each second
                ri->updateParam(); 
            }
        };

};

class onQualibtrateMode
: public RobocarsStateMachine
{
    public:
        onQualibtrateMode() : RobocarsStateMachine("onQualibtrateMode") {};

    private:

        void entry(void) override {
            RobocarsStateMachine::entry();
            ri->initQualibration();
        };

        void react (LeaveQualibrateModeEvent const & e) override {
            RobocarsStateMachine::react(e);
            transit<onIdle>();
        }

        void react (RadioChannelEvent const & e) override {
            ri->qualibrate(e.radio_channel_value); 
        };

        void react (TickEvent const & e) override {
        };

};

class onIdle
: public onRunningMode
{
    public:
        onIdle() : onRunningMode("onArm") {};

    private:

        void entry(void) override {
            onRunningMode::entry();
        };
  
        void react(ManualDrivingEvent const & e) override { 
            onRunningMode::react(e);
            transit<onManualDriving>();
        };

        void react(EnterQualibrateModeEvent const & e) override { 
            onRunningMode::react(e);
            transit<onQualibtrateMode>();
        };

        void react(TickEvent const & e) override {
            onRunningMode::react(e);
            ri->maintainIdleActuator();
        };

};

class onManualDriving
: public onRunningMode
{
    public:
        onManualDriving() : onRunningMode("onManualDriving") {};

    private:

        void entry(void) override {
            onRunningMode::entry();
        };

        void react (AutonomousDrivingEvent const & e) override {
            onRunningMode::react(e);
            transit<onAutonomousDriving>();
        }

        void react(IdleStatusEvent const & e) override { 
            onRunningMode::react(e);
            transit<onIdle>();
        };

        void react (RadioChannelEvent const & e) override {
            ri->controlActuator(e.radio_channel_value); 
        }

        void react (TickEvent const & e) override {
            onRunningMode::react(e);
        };

};

class onAutonomousDriving
: public onRunningMode
{
    public:
        onAutonomousDriving() : onRunningMode("onAutonomousDriving") {};

    private:

        virtual void entry(void) { 
            onRunningMode::entry();
        };  

        virtual void react(IdleStatusEvent                 const & e) override { 
            onRunningMode::react(e);
            transit<onIdle>();
        };

        virtual void react(ManualDrivingEvent                     const & e) override { 
            onRunningMode::react(e);
            transit<onManualDriving>();
        };

        virtual void react(TickEvent                      const & e) override { 
            onRunningMode::react(e);
        };
};

FSM_INITIAL_STATE(RobocarsStateMachine, onIdle)


uint32_t mapRange(uint32_t in1,uint32_t in2,uint32_t out1,uint32_t out2,uint32_t value)
{
  if (value<in1) {value=in1;}
  if (value>in2) {value=in2;}
  return out1 + ((value-in1)*(out2-out1))/(in2-in1);
}

uint32_t mapRange(_Float32 in1,_Float32 in2,_Float32 out1,_Float32 out2,_Float32 value)
{
  if (value<in1) {value=in1;}
  if (value>in2) {value=in2;}
  return out1 + ((value-in1)*(out2-out1))/(in2-in1);
}

void RosInterface::initParam() {
    if (!nh.hasParam("command_input_min")) {
        nh.setParam ("command_input_min", 363);       
    }
    if (!nh.hasParam("command_input_max")) {
        nh.setParam ("command_input_max", 1641);       
    }
    if (!nh.hasParam("command_output_min")) {
        nh.setParam ("command_output_min", 1000);       
    }
    if (!nh.hasParam("command_output_max")) {
        nh.setParam ("command_output_max", 2000);       
    }
    if (!nh.hasParam("loop_hz")) {
        nh.setParam ("loop_hz", 60);       
    }
}
void RosInterface::updateParam() {
    nh.getParam("command_input_min", command_input_min);
    nh.getParam("command_input_max", command_input_max);
    nh.getParam("command_output_min", command_output_min);
    nh.getParam("command_output_max", command_output_max);
    nh.getParam("loop_hz", loop_hz);
}

void RosInterface::initPub () {
    act_steering_pub = nh.advertise<robocars_msgs::robocars_actuator_output>("output", 10);
}

void RosInterface::initSub () {
    channels_sub = nh.subscribe<robocars_msgs::robocars_radio_channels>("/radio_channels", 1, &RosInterface::channels_msg_cb, this);
    state_sub = nh.subscribe<robocars_msgs::robocars_brain_state>("/robocars_brain_state", 1, &RosInterface::state_msg_cb, this);
    mode_sub = nh.subscribe<robocars_msgs::robocars_actuator_ctrl_mode>("/robocars_actuator_ctrl_mode", 1, &RosInterface::mode_msg_cb, this);
}

void RosInterface::channels_msg_cb(const robocars_msgs::robocars_radio_channels::ConstPtr& msg){
    
    send_event(RadioChannelEvent(msg->ch1));
}

void RosInterface::state_msg_cb(const robocars_msgs::robocars_brain_state::ConstPtr& msg) {
    static u_int32_t last_state = -1;
    if (msg->state != last_state) {
        switch (msg->state) {
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_IDLE:
                send_event(IdleStatusEvent());        
            break;
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_MANUAL_DRIVING:
                send_event(ManualDrivingEvent());        
            break;
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_AUTONOMOUS_DRIVING:
                send_event(AutonomousDrivingEvent());        
            break;
        }
        last_state=msg->state;
    }
    
}

void RosInterface::mode_msg_cb(const robocars_msgs::robocars_actuator_ctrl_mode::ConstPtr& msg) {
    if (msg->mode == robocars_msgs::robocars_actuator_ctrl_mode::ACTUATOR_MODE_QUALIBRATE) {
        send_event(EnterQualibrateModeEvent());        
    } else {
        send_event(LeaveQualibrateModeEvent());        
    }
}

void RosInterface::controlActuator (uint32_t steering_value) {

    robocars_msgs::robocars_actuator_output steeringMsg;

    steeringMsg.header.stamp = ros::Time::now();
    steeringMsg.header.seq=1;
    steeringMsg.header.frame_id = "mainSteering";
    steeringMsg.pwm = mapRange(command_input_min,command_input_max,command_output_min,command_output_max,steering_value);
    steeringMsg.norm = mapRange((_Float32)command_input_min,(_Float32)command_input_max,-1.0,1.0,(_Float32)steering_value);

    act_steering_pub.publish(steeringMsg);
}

void RosInterface::maintainIdleActuator () {

    robocars_msgs::robocars_actuator_output steeringMsg;

    steeringMsg.header.stamp = ros::Time::now();
    steeringMsg.header.seq=1;
    steeringMsg.header.frame_id = "idleSteering";
    steeringMsg.pwm = 1500;
    steeringMsg.norm = 0.0;

    act_steering_pub.publish(steeringMsg);
}

void RosInterface::initQualibration() {
    command_input_min = 1500;
    command_input_max = 1500;
}

void RosInterface::qualibrate (uint32_t steering_value) {
    if (steering_value < command_input_min) {
        nh.setParam ("command_input_min", (int)steering_value);
        command_input_min = steering_value;         
    }
    if (steering_value > command_input_max) {
        nh.setParam ("command_input_max", (int)steering_value);         
        command_input_max = steering_value;         
    }
}

int main(int argc, char **argv)
{
    int loopCnt=0;
    ros::init(argc, argv, "robocars_steering_ctrl_fsm");

    ri = new RosInterface;

    ri->initPub();
    fsm_list::start();
    ri->initSub();
    ROS_INFO("Steering Ctrl: Starting");

    // wait for FCU connection
    ros::Rate rate(loop_hz);
    while(ros::ok()){
        ros::spinOnce();
        send_event (TickEvent());
        rate.sleep();
    }
}

