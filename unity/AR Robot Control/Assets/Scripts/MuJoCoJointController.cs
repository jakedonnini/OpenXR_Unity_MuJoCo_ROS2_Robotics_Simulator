using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.PandaKinematics; // Your custom message namespace
using Mujoco;
using System.Linq;

public class MuJoCoJointController : MonoBehaviour
{
    [Header("ROS Settings")]
    [SerializeField] private string topicName = "arm_joint_commands";
    [SerializeField] public enum ControlMode { Position, Velocity, Jog_Velocity }

    [Header("MuJoCo Joints")]
    [SerializeField] public ControlMode controlMode = ControlMode.Velocity;

    
    [Header("MuJoCo Joints")]
    [SerializeField] private MjActuator[] jointActuators = new MjActuator[8]; // 8th is gripper
    
    // Alternative: if using MjHingeJoint directly
    [SerializeField] private MjHingeJoint[] joints = new MjHingeJoint[7];
    [SerializeField] public float Kp = 1.0f;
    [SerializeField] public float tolerance = 0.1f;
    
    private ROSConnection ros;
    
    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointCommandMsg>(topicName);
        if (controlMode == ControlMode.Position)
        {
            ros.Subscribe<JointCommandMsg>(topicName, OnJointCommandReceived);
            Debug.Log($"Subscribed to {topicName}");

            // switch actuators to position mode if needed
            for (int i = 0; i < jointActuators.Length; i++)
            {
                if (jointActuators[i] != null)
                {
                    // jointActuators[i].Type = MjActuator.ActuatorType.Position;
                    // jointActuators[i].CustomParams.Kp = 500.0f;
                    // jointActuators[i].CustomParams.Kv = 20.0f;
                }
            }
        }
        else if (controlMode == ControlMode.Velocity)
        {
            ros.RegisterPublisher<JointCommandMsg>(topicName);
            ros.Subscribe<JointCommandMsg>(topicName, OnJointVelocityCommandReceived);

            Debug.Log($"Subscribed to {topicName}");

            // switch actuators to velocity mode if needed
            for (int i = 0; i < jointActuators.Length; i++)
            {
                if (jointActuators[i] != null && i != 7) // assuming 8th actuator is gripper
                {
                    jointActuators[i].Type = MjActuator.ActuatorType.Velocity;
                }
            }

            // gripper is awlays in general mode
            jointActuators[7].Type = MjActuator.ActuatorType.Position; // assuming 8th actuator is gripper
        }
        else if (controlMode == ControlMode.Jog_Velocity)
        {
            ros.RegisterPublisher<JointCommandMsg>(topicName);
            ros.Subscribe<JointCommandMsg>(topicName, jogVelocity);
            Debug.Log($"Subscribed to {topicName}");
        }
    }
    
    void OnJointCommandReceived(JointCommandMsg msg)
    {
        if (msg.command_type != "position_joint")
        {
            Debug.LogWarning($"Received unexpected command type: {msg.command_type}");
            return;
        }
        
        if (msg.joint_angles.Length != 7)
        {
            Debug.LogWarning($"Expected 7 joint angles, got {msg.joint_angles.Length}");
            return;
        }
        
        // Apply joint angles to MuJoCo actuators (radians)
        for (int i = 0; i < 7 && i < jointActuators.Length; i++)
        {
            if (jointActuators[i] != null)
            {
                // Set the control signal for position actuators (radians) and convert to degrees if necessary
                jointActuators[i].Control = (float)msg.joint_angles[i];
            }
        }

        jointActuators[7].Control = (float)msg.gripper_pos; // Gripper control
        
        // Debug.Log($"Applied joint angles: [{string.Join(", ", msg.joint_angles)}]");
    }

    void jogVelocity(JointCommandMsg msg)
    {
        // return true when it has reached the goal
        if (msg.joint_angles.Length != 7)
        {
            Debug.LogWarning($"Expected 7 joint angles, got {msg.joint_angles.Length}");
            return;
        }

        bool reached_goal = false;
        bool[] joint_reached_goal = new bool[7];

        // iterate velocities until reaches goal
        while(!reached_goal)
        {
            
            for (int i = 0; i < 7 && i < jointActuators.Length; i++)
            {
                if (jointActuators[i] != null)
                {
                    float diff = joints[i].Configuration - (float)msg.joint_angles[i];

                    if (Mathf.Abs(diff) > tolerance)
                    {
                        jointActuators[i].Control = -Kp * diff; // simple P controller
                        joint_reached_goal[i] = false;
                    } else {
                        jointActuators[i].Control = 0;
                        joint_reached_goal[i] = true;
                    }
                }
            }

            Debug.Log($"velocities: [{string.Join(", ", jointActuators.Take(7).Select(a => a.Control))}]");

            // check if all joints have reached the goal
            reached_goal = joint_reached_goal.Contains(false) == false;
        }
        // Needs way to stop move until this completes, maybe a feedback topic or service to report status back to ROS
    }
    
    // Alternative method if using velocity control
    void OnJointVelocityCommandReceived(JointCommandMsg msg)
    {
        Debug.Log($"Received joint velocity command message. [{string.Join(", ", msg.joint_angles)}]");
        if (msg.command_type != "velocity_joint")
        {
            Debug.LogWarning($"Received unexpected command type: {msg.command_type}");
            return;
        }
        
        if (msg.joint_angles.Length != 7)
        {
            Debug.LogWarning($"Expected 7 joint angles, got {msg.joint_angles.Length}");
            return;
        }

        for (int i = 0; i < 7 && i < jointActuators.Length; i++)
        {
            if (jointActuators[i] != null)
            {
                // Set velocity instead of position
                jointActuators[i].Control = (float)msg.joint_angles[i];
            }
        }

        jointActuators[7].Control = (float)msg.gripper_pos; // Gripper control
    }
}