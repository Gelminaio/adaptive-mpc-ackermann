#include "microros_node.h"

#include "config.h"
#if USE_MICROROS

#include "globals.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/bool.h>

#include <math.h>

namespace comms
{
    // micro-ROS entities (file-scope so the C callback can reach them)
    static rclc_support_t support;
    static rcl_allocator_t allocator;
    static rcl_node_t node;
    static rclc_executor_t executor;

    static rcl_publisher_t pub_joint;
    static rcl_publisher_t pub_imu;
    static rcl_subscription_t sub_cmdvel;

    static rcl_subscription_t sub_arm;
    static std_msgs__msg__Bool msg_arm;

    static sensor_msgs__msg__JointState msg_joint;
    static sensor_msgs__msg__Imu msg_imu;
    static geometry_msgs__msg__Twist msg_cmdvel;

    // Buffers for JointState arrays (2 wheels)
    static double joint_positions[2];
    static double joint_velocities[2];

    // helper: macro-like error check
    static bool rcl_ok(rcl_ret_t rc) { return rc == RCL_RET_OK; }

    // cmd_vel callback
    // converts Twist (linear.x, angular.z) into wheel setpoints + steering angle using inverse Ackermann (bicycle model)(explained in my documentation).
    static void cmdvel_callback(const void *msgin)
    {
        const geometry_msgs__msg__Twist *m = static_cast<const geometry_msgs__msg__Twist *>(msgin);

        const float v = static_cast<float>(m->linear.x);      // m/s
        const float omega = static_cast<float>(m->angular.z); // rad/s

        // steering angle from bicycle model: delta = atan(omega * L / v)
        float steering_deg = 0.0f;
        if (fabsf(v) > 0.05f)
        {
            const float delta_rad = atanf(omega * WHEELBASE_M / v);
            steering_deg = delta_rad * 180.0f / static_cast<float>(M_PI);
        }
        // clamp handled inside servo driver; set steering.
        g_servo.setAngle(steering_deg);

        // Rear wheels both get v as setpoint (no electronic differential yet (I will implement it one day)).
        g_vehicle_state.wheel_left.velocity_setpoint_mps = v;
        g_vehicle_state.wheel_right.velocity_setpoint_mps = v;

        // Notify safety: a fresh command arrived (resets timeout).
        g_safety.notifyCommand(millis());
    }

    // arm callback: true -> arm, false -> disarm
    static void arm_callback(const void *msgin)
    {
        const std_msgs__msg__Bool *m = static_cast<const std_msgs__msg__Bool *>(msgin);
        if (m->data)
        {
            g_safety.clearEmergency();
            g_safety.arm();
        }
        else
        {
            g_safety.disarm();
        }
    }

    bool MicroRosNode::begin()
    {
        // set up serial transport. Serial must already be started (it is, in main).
        set_microros_serial_transports(Serial);
        delay(2000); // give a moment to the agent

        agent_connected_ = createEntities();
        return agent_connected_;
    }

    bool MicroRosNode::createEntities()
    {
        allocator = rcl_get_default_allocator();

        // Support init
        if (!rcl_ok(rclc_support_init(&support, 0, NULL, &allocator)))
            return false;

        // Node
        if (!rcl_ok(rclc_node_init_default(&node, "ackermann_firmware", "", &support)))
            return false;

        // Publisher: joint_states
        if (!rcl_ok(rclc_publisher_init_default(
                &pub_joint, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                "joint_states")))
            return false;

        // Publisher: imu/data_raw
        if (!rcl_ok(rclc_publisher_init_default(
                &pub_imu, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                "imu/data_raw")))
            return false;

        // Subscriber: cmd_vel
        if (!rcl_ok(rclc_subscription_init_default(
                &sub_cmdvel, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                "cmd_vel")))
            return false;

        // Subscriber: arm
        if (!rcl_ok(rclc_subscription_init_default(
                &sub_arm, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                "arm")))
            return false;

        // Executor with 2 handles (the subscriptions)
        if (!rcl_ok(rclc_executor_init(&executor, &support.context, 2, &allocator)))
            return false;
        if (!rcl_ok(rclc_executor_add_subscription(
                &executor, &sub_cmdvel, &msg_cmdvel,
                &cmdvel_callback, ON_NEW_DATA)))
            return false;
        if (!rcl_ok(rclc_executor_add_subscription(
                &executor, &sub_arm, &msg_arm,
                &arm_callback, ON_NEW_DATA)))
            return false;

        // Initialize JointState message arrays
        msg_joint.position.data = joint_positions;
        msg_joint.position.size = 2;
        msg_joint.position.capacity = 2;
        msg_joint.velocity.data = joint_velocities;
        msg_joint.velocity.size = 2;
        msg_joint.velocity.capacity = 2;

        return true;
    }

    void MicroRosNode::destroyEntities()
    {
        rcl_publisher_fini(&pub_joint, &node);
        rcl_publisher_fini(&pub_imu, &node);
        rcl_subscription_fini(&sub_cmdvel, &node);
        rcl_subscription_fini(&sub_arm, &node);
        rclc_executor_fini(&executor);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        agent_connected_ = false;
    }

    void MicroRosNode::publishJointStates()
    {
        // Wheel angular position (rad) and velocity (rad/s) from encoders.
        // position: total revolutions * 2pi; velocity: m/s converted to rad/s.
        const float r = WHEEL_RADIUS_M;
        joint_positions[0] =
            (double)g_encoder_left.getCount() / ENCODER_TICKS_PER_REV * 2.0 * M_PI;
        joint_positions[1] =
            (double)g_encoder_right.getCount() / ENCODER_TICKS_PER_REV * 2.0 * M_PI;
        joint_velocities[0] = g_vehicle_state.wheel_left.velocity_mps / r;
        joint_velocities[1] = g_vehicle_state.wheel_right.velocity_mps / r;

        rcl_publish(&pub_joint, &msg_joint, NULL);
    }

    void MicroRosNode::publishImu()
    {
        const ImuData &imu = g_vehicle_state.imu;

        msg_imu.orientation.w = imu.qw;
        msg_imu.orientation.x = imu.qx;
        msg_imu.orientation.y = imu.qy;
        msg_imu.orientation.z = imu.qz;

        msg_imu.angular_velocity.x = imu.gyro_x;
        msg_imu.angular_velocity.y = imu.gyro_y;
        msg_imu.angular_velocity.z = imu.gyro_z;

        msg_imu.linear_acceleration.x = imu.lin_acc_x;
        msg_imu.linear_acceleration.y = imu.lin_acc_y;
        msg_imu.linear_acceleration.z = imu.lin_acc_z;

        rcl_publish(&pub_imu, &msg_imu, NULL);
    }

    void MicroRosNode::spinOnce()
    {
        if (!agent_connected_)
        {
            // try to (re)connect periodically.
            agent_connected_ = createEntities();
            if (!agent_connected_)
            {
                destroyEntities();
                return;
            }
        }

        // spin the executor briefly (handles incoming cmd_vel).
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

        // publish at ~50 Hz.
        const uint32_t now = millis();
        if (now - last_publish_ms_ >= 20)
        {
            last_publish_ms_ = now;
            publishJointStates();
            publishImu();
        }
    }
}
#endif