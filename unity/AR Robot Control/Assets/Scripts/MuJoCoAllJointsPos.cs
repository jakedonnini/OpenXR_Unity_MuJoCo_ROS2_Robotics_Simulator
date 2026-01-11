using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.PandaKinematics;

public class MuJoCoAllJointsPos : MonoBehaviour
{
    public string topicName = "arm_joint_pos";
    public GameObject spherePrefab;
    public float sphereScale = 0.05f;
    public Transform baseOffset;

    private ROSConnection ros;
    private GameObject[] jointSpheres = new GameObject[8];
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<AllJointPosMsg>(topicName, OnJointPosReceived);

        // Pre-spawn spheres
        for (int i = 0; i < 8; i++)
        {
            jointSpheres[i] = Instantiate(spherePrefab, Vector3.zero, Quaternion.identity);
            jointSpheres[i].name = $"JointSphere_{i}";
            jointSpheres[i].transform.localScale = Vector3.one * sphereScale;
        }
    }

    void OnJointPosReceived(AllJointPosMsg msg)
    {
        if (msg.data.Length < 24)
        {
            Debug.LogError("arm_joint_pos message has insufficient data!");
            return;
        }

        for (int i = 0; i < 8; i++)
        {
            int baseIdx = i * 3;

            float x = (float)msg.data[baseIdx + 0];
            float y = (float)msg.data[baseIdx + 1];
            float z = (float)msg.data[baseIdx + 2];

            // If these are ROS coordinates, convert ROS → Unity here
            Vector3 unityPos = new Vector3(
                x + baseOffset.position.x,  // ROS x → Unity x
                z + baseOffset.position.y,  // ROS y → Unity z
                y + baseOffset.position.z   // ROS z → Unity y
            );

            jointSpheres[i].transform.position = unityPos;
        }
    }
}
