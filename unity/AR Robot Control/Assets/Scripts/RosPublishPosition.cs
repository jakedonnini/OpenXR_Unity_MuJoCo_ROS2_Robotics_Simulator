using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;

public class RosPublishPosition : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "pos_rot";

    // The game object
    public GameObject trackedObject;
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

            // Create a new message
            PosRotMsg posRotMsg = new PosRotMsg
            (
                trackedObject.transform.position.x,
                trackedObject.transform.position.y,
                trackedObject.transform.position.z,
                trackedObject.transform.rotation.x,
                trackedObject.transform.rotation.y,
                trackedObject.transform.rotation.z,
                trackedObject.transform.rotation.w
            );

            // Publish the message
            ros.Publish(topicName, posRotMsg);
        }
    }
}