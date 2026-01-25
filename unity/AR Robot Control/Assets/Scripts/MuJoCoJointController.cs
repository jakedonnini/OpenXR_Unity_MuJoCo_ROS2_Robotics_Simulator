using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.PandaKinematics; // Your custom message namespace
using Mujoco;

public class MuJoCoJointController : MonoBehaviour
{
    [Header("ROS Settings")]
    [SerializeField] private string topicNamePos = "arm_joint_commands_pos";
    [SerializeField] private string topicNameVel = "arm_joint_commands_vel";
    [SerializeField] public enum ControlMode { Position, Velocity }

    [Header("MuJoCo Joints")]
    [SerializeField] public ControlMode controlMode = ControlMode.Velocity;

    
    [Header("MuJoCo Joints")]
    [SerializeField] private MjActuator[] jointActuators = new MjActuator[8]; // 8th is gripper
    
    // Alternative: if using MjHingeJoint directly
    // [SerializeField] private MjHingeJoint[] joints = new MjHingeJoint[7];
    
    private ROSConnection ros;
    
    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        if (controlMode == ControlMode.Position)
        {
            ros.RegisterPublisher<JointCommandMsg>(topicNamePos);
            ros.Subscribe<JointCommandMsg>(topicNamePos, OnJointCommandReceived);
            Debug.Log($"Subscribed to {topicNamePos}");

            // switch actuators to position mode if needed
            for (int i = 0; i < jointActuators.Length; i++)
            {
                if (jointActuators[i] != null)
                {
                    jointActuators[i].Type = MjActuator.ActuatorType.General;
                }
            }
        }
        else if (controlMode == ControlMode.Velocity)
        {
            ros.RegisterPublisher<JointCommandMsg>(topicNameVel);
            ros.Subscribe<JointCommandMsg>(topicNameVel, OnJointVelocityCommandReceived);

            Debug.Log($"Subscribed to {topicNameVel}");

            // switch actuators to velocity mode if needed
            for (int i = 0; i < jointActuators.Length; i++)
            {
                if (jointActuators[i] != null && i != 7) // assuming 8th actuator is gripper
                {
                    jointActuators[i].Type = MjActuator.ActuatorType.Velocity;
                }
            }

            // gripper is awlays in general mode
            jointActuators[7].Type = MjActuator.ActuatorType.General; // assuming 8th actuator is gripper
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
    
    // Alternative method if using velocity control
    void OnJointVelocityCommandReceived(JointCommandMsg msg)
    {
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