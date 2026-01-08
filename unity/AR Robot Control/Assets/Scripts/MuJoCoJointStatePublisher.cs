using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.PandaKinematics;
using RosMessageTypes.Std;
using Mujoco;

public class MuJoCoJointStatePublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    [SerializeField] private string topicName = "panda_joint_states";
    [SerializeField] private float publishRate = 20f; // Hz
    
    [Header("MuJoCo Joints")]
    [SerializeField] private MjHingeJoint[] joints = new MjHingeJoint[7];
    
    // Alternative: if reading from actuators instead
    // [SerializeField] private MjActuator[] jointActuators = new MjActuator[7];
    
    private ROSConnection ros;
    private float publishTimer = 0f;
    
    void Start()
    {
        // Connect to ROS and register publisher
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointCommandMsg>(topicName);
        
        Debug.Log($"Publishing joint states to {topicName} at {publishRate} Hz");
    }
    
    void Update()
    {
        publishTimer += Time.deltaTime;
        
        if (publishTimer >= 1f / publishRate)
        {
            PublishJointStates();
            publishTimer = 0f;
        }
    }
    
    void PublishJointStates()
    {
        // Create message
        var msg = new JointCommandMsg();
        
        // Set header with timestamp
        msg.header = new HeaderMsg();
        var currentTime = Time.realtimeSinceStartup;
        msg.header.stamp.sec = (int)currentTime;
        msg.header.stamp.nanosec = (uint)((currentTime % 1) * 1e9);
        msg.header.frame_id = "panda_link0";
        
        msg.joint_angles = new double[7];
        msg.command_type = "state"; 
        
        // Read current joint angles from MuJoCo
        for (int i = 0; i < 7 && i < joints.Length; i++)
        {
            if (joints[i] != null)
            {
                // Get joint position in radians (try these alternatives):
                // Option 1: QposAddress
                msg.joint_angles[i] = joints[i].Configuration;
            }
            else
            {
                Debug.LogWarning($"Joint {i} is not assigned!");
                msg.joint_angles[i] = 0.0;
            }
        }
        
        // Publish to ROS
        ros.Publish(topicName, msg);
    }
    
}