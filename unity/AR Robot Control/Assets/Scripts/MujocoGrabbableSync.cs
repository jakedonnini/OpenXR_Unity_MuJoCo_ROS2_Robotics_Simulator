using UnityEngine;
using Oculus.Interaction;
using Mujoco;

public class MujocoGrabbableSync : MonoBehaviour
{
    public MujocoBody mujocoBody; // your MuJoCo wrapper

    private bool isGrabbed;

    void Start()
    {
        var grabbable = GetComponent<Grabbable>();
        grabbable.WhenPointerEventRaised += OnPointerEvent;
    }

    void OnPointerEvent(PointerEvent evt)
    {
        if (evt.Type == PointerEventType.Select)
        {
            isGrabbed = true;
            mujocoBody.SetKinematic(true); // stop sim
        }
        else if (evt.Type == PointerEventType.Unselect)
        {
            isGrabbed = false;
            SyncUnityToMuJoCo();
            mujocoBody.SetKinematic(false); // resume sim
        }
    }
}
