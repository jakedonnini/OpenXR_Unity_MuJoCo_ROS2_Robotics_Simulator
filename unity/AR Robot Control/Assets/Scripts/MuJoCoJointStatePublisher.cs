using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.PandaKinematics;
using RosMessageTypes.Std;
using Mujoco;

public class MuJoCoJointStatePublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    [SerializeField] private string topicNamePos = "panda_joint_states_pos";
    [SerializeField] private string topicNameVel = "panda_joint_states_vel";
    [SerializeField] private float publishRate = 20f; // Hz
    
    [Header("MuJoCo Joints")]
    [SerializeField] private MjHingeJoint[] joints = new MjHingeJoint[7];
    [SerializeField] private MjSlideJoint gripperJoint;
    
    // Alternative: if reading from actuators instead
    [SerializeField] private MjActuator[] jointActuators = new MjActuator[7];

    public bool useAcutuators = false;
    
    private ROSConnection ros;
    private float publishTimer = 0f;
    
    void Start()
    {
        // Connect to ROS and register publisher
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointCommandMsg>(topicNamePos);
        ros.RegisterPublisher<JointCommandMsg>(topicNameVel);
        Debug.Log($"Publishing joint states to {topicNamePos} and {topicNameVel} at {publishRate} Hz");
    }
    
    void Update()
    {
        publishTimer += Time.deltaTime;
        
        if (publishTimer >= 1f / publishRate)
        {
            // Publish joint states based on selected mode
            if (useAcutuators)
                PublishAcutatorStates();
            else
                PublishJointStates();
                PublishJointVelocities();
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
        //msg.header.stamp.sec = (int)currentTime;
        //msg.header.stamp.nanosec = (uint)((currentTime % 1) * 1e9);
        msg.header.frame_id = "panda_link0";
        
        msg.joint_angles = new double[7];
        msg.command_type = "state_joint"; 
        
        // Read current joint angles from MuJoCo
        for (int i = 0; i < 7 && i < joints.Length; i++)
        {
            if (joints[i] != null)
            {
                // Get joint position in radians (try these alternatives):
                // convert to radians
                msg.joint_angles[i] = joints[i].Configuration * Mathf.Deg2Rad;
            }
            else
            {
                Debug.LogWarning($"Joint {i} is not assigned!");
                msg.joint_angles[i] = 0.0;
            }
        }

        // publish gripper position
        msg.gripper_pos = gripperJoint.Configuration;

        Debug.Log($"Publishing joint states: [{string.Join(", ", msg.joint_angles)}]");
        
        // Publish to ROS
        ros.Publish(topicNamePos, msg);
    }

    void PublishJointVelocities()
    {
        // Create message
        var msg = new JointCommandMsg();
        
        // Set header with timestamp
        msg.header = new HeaderMsg();
        var currentTime = Time.realtimeSinceStartup;
        //msg.header.stamp.sec = (int)currentTime;
        //msg.header.stamp.nanosec = (uint)((currentTime % 1) * 1e9);
        msg.header.frame_id = "panda_link0";
        
        msg.joint_angles = new double[7];
        msg.command_type = "state_velocity"; 
        
        // Read current joint velocities from MuJoCo
        for (int i = 0; i < 7 && i < joints.Length; i++)
        {
            if (joints[i] != null)
            {
                // Get joint velocity in radians per second (try these alternatives):
                // convert to radians
                msg.joint_angles[i] = joints[i].Velocity * Mathf.Deg2Rad;
            }
            else
            {
                Debug.LogWarning($"Joint {i} is not assigned!");
                msg.joint_angles[i] = 0.0;
            }
        }

        // publish gripper velocity
        msg.gripper_pos = gripperJoint.Velocity;

        // Debug.Log($"Publishing joint velocities: [{string.Join(", ", msg.joint_angles)}]");
        
        // Publish to ROS
        ros.Publish(topicNameVel, msg);
    }

    void PublishAcutatorStates()
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
        msg.command_type = "state_actuator";

        // Read current joint angles from MuJoCo actuators
        for (int i = 0; i < 7 && i < jointActuators.Length; i++)
        {
            if (jointActuators[i] != null)
            {
                // Get actuator position in radians (try these alternatives):
                // convert to radians
                msg.joint_angles[i] = jointActuators[i].Control;
            }
            else
            {
                Debug.LogWarning($"Actuator {i} is not assigned!");
                msg.joint_angles[i] = 0.0;
            }
        }
    }
}