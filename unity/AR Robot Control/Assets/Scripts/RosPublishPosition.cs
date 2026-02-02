using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;

public class RosPublishPosition : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "pos_rot";

    // The game object
    public GameObject trackedObject;

    public float[] positionOffset = new float[3] {0f, 0f, 0f};
    public Transform armOffset;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.05f;
    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            timeElapsed = 0;

            // Debug.Log($"Publishing position  {trackedObject.transform.position.x}, {trackedObject.transform.position.y}, {trackedObject.transform.position.z}");

            // convert from unity(RUF) to ROS(FLU)
            // Create a new message
            PosRotMsg posRotMsg = new PosRotMsg
            (
                trackedObject.transform.position.x - armOffset.position.x + positionOffset[0],
                trackedObject.transform.position.z - armOffset.position.z + positionOffset[2],
                trackedObject.transform.position.y - armOffset.position.y + positionOffset[1],
                -trackedObject.transform.rotation.x,
                -trackedObject.transform.rotation.z,
                -trackedObject.transform.rotation.y,
                trackedObject.transform.rotation.w
            );

            // Debug.Log($"Publishing position: ({trackedObject.transform.position.x + armOffset.position.x}, {trackedObject.transform.position.y + armOffset.position.y}, {trackedObject.transform.position.z + armOffset.position.z})");

            // Publish the message
            ros.Publish(topicName, posRotMsg);
        }
    }
}