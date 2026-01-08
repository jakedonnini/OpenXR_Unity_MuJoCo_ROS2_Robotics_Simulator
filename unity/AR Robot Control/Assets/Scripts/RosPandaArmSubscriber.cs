using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosJointPos = RosMessageTypes.PandaKinematics.JointCommandMsg;

public class RosPandaArmSubscriber : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<RosJointPos>("arm_joint_commands", JointCommandChange);
    }

    void JointCommandChange(RosJointPos jointCommandMessage)
    {
        // Example implementation: Log the received joint positions
        Debug.Log("Received Joint Positions: " + string.Join(", ", jointCommandMessage.joint_angles));
        
        // Here you would add code to update the Panda arm model in Unity
        // based on the received joint positions.
    }
}
