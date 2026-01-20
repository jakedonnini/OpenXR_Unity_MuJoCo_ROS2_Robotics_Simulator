using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.PandaKinematics; // Your custom message namespace
using Mujoco;

public class MuJoCoJointController : MonoBehaviour
{
    [Header("ROS Settings")]
    [SerializeField] private string topicName = "arm_joint_commands";
    
    [Header("MuJoCo Joints")]
    [SerializeField] private MjActuator[] jointActuators = new MjActuator[8]; // 8th is gripper
    
    // Alternative: if using MjHingeJoint directly
    // [SerializeField] private MjHingeJoint[] joints = new MjHingeJoint[7];
    
    private ROSConnection ros;
    
    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointCommandMsg>(topicName, OnJointCommandReceived);
        
        Debug.Log($"Subscribed to {topicName}");
    }
    
    void OnJointCommandReceived(JointCommandMsg msg)
    {
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
    void ApplyVelocityControl(JointCommandMsg msg)
    {
        for (int i = 0; i < 7 && i < jointActuators.Length; i++)
        {
            if (jointActuators[i] != null)
            {
                // Set velocity instead of position
                jointActuators[i].Control = (float)msg.joint_angles[i];
            }
        }
    }
}