using UnityEngine;
using Oculus.Interaction;
using Mujoco;

public class MujocoGrabbableSync : MonoBehaviour
{
    public MjBody mujocoBody;

    public OVRHand rightHand;
    public OVRHand leftHand;
    public Transform rightHandAnchor;
    public Transform leftHandAnchor;

    public float kp = 8f;   // stiffness
    public float kd = 2f;    // damping
    public float maxForce = 5f;

    public float rotation_kp = 2f;
    public float rotation_kd = 1f;
    public float rotation_maxForce = 5f;


    public float grabDistance = 2f;
    public float pinchDistance = 0.07f;

    public float[] positionOffset = new float[3] {1f, 1f, 1f};
    public float[] rotationOffset = new float[3] {0f, 0f, 0f};

    public bool debug = true;

    GameObject debugForceCylinder;

    private Transform cubeParent;

    void Start()
    {
        debugForceCylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        debugForceCylinder.name = "DebugForce";
        debugForceCylinder.transform.localScale = new Vector3(0.02f, 0.5f, 0.02f);

        var renderer = debugForceCylinder.GetComponent<Renderer>();
        renderer.material = new Material(Shader.Find("Unlit/Color"));
        renderer.material.color = Color.red;

        Destroy(debugForceCylinder.GetComponent<Collider>());

        cubeParent = mujocoBody.GetComponentInParent<Transform>();
    }

    
    void Update()
    {
        // Clear forces every frame (VERY IMPORTANT)
        ClearForces();

        if (IsGripping(rightHand))
        {
            ApplyForceFromHand(rightHand, rightHandAnchor);
        }
        else if (IsGripping(leftHand))
        {
            ApplyForceFromHand(leftHand, leftHandAnchor);
        } 
        else
        {
            if (debugForceCylinder != null)
                debugForceCylinder.SetActive(false);
        }
    }

    bool IsGripping(OVRHand hand)
    {
        return hand != null &&
               hand.GetFingerPinchStrength(OVRHand.HandFinger.Index) > 0.75f;
    }
    
    unsafe void ApplyForceFromHand(OVRHand hand, Transform handAnchor = null)
    {
        Vector3 handPos = hand.transform.position;
        Debug.Log($"Hand Position: {handPos}");
        Quaternion handRot = hand.transform.rotation;
        // use the hand anchor point instead if provided
        // if (handAnchor != null)
        // {
        //     handPos = handAnchor.position;
        //     handRot = handAnchor.rotation;
        // }

        handPos += handRot * Vector3.forward * pinchDistance; // offset forward a bit to match grab point
        handPos += handRot * Vector3.up * -pinchDistance; // offset up a bit to match grab point
        Debug.Log($"Hand Position afters: {handPos}");
        Vector3 objPos  = cubeParent.position;
        Quaternion objRot = cubeParent.rotation;

        // --- POSITION ERROR ---
        Vector3 posError = handPos - objPos;
        Quaternion rotError = handRot * Quaternion.Inverse(objRot);

        // --- OBJECT VELOCITY (MuJoCo world frame) ---
        int bodyId = mujocoBody.MujocoId;
        int velAdr = MjScene.Instance.Model->body_dofadr[bodyId];

        Vector3 objVel = Vector3.zero;
        Vector3 objEulerVel = Vector3.zero;
        if (velAdr >= 0)
        {
            // mujoco stores velocities in a different order than unity
            objVel = new Vector3(
                (float)MjScene.Instance.Data->qvel[velAdr + 0],
                (float)MjScene.Instance.Data->qvel[velAdr + 2],
                (float)MjScene.Instance.Data->qvel[velAdr + 1]
            );

            objEulerVel = new Vector3(
                (float)MjScene.Instance.Data->qvel[velAdr + 3],
                (float)MjScene.Instance.Data->qvel[velAdr + 5],
                (float)MjScene.Instance.Data->qvel[velAdr + 4]
            );
        }

        Debug.Log($"Obj Vel: {objVel}, Obj Euler Vel: {objEulerVel}");

        // convert euler to quaternion
        Quaternion objAngularVel = Quaternion.Euler(objEulerVel); // convert from rad/s to deg/s

        // --- PD FORCE ---
        Vector3 force = kp * posError - kd * objVel;
        Quaternion angularForce = Quaternion.Slerp(Quaternion.identity, rotError, rotation_kp) * Quaternion.Inverse(Quaternion.Slerp(Quaternion.identity, objAngularVel, rotation_kd));
        
        // clamp forces
        force = Vector3.ClampMagnitude(force, maxForce);
        angularForce = Quaternion.Slerp(Quaternion.identity, angularForce, rotation_maxForce);
        
        // draw debug distance to cube
        float debugLength = UpdateDebugForce(objPos, force, handPos);

        if (debugLength > grabDistance)
            return; // don't apply force if too far away

        Vector3 dir = force.normalized;

        // convert the torque to euler angles for application
        Vector3 torque = angularForce.eulerAngles;

        double gravityCompensation = 9.81 * mujocoBody.GetComponentInChildren<MjGeom>().Mass; // gravity in mujoco is in negative z direction

        // --- APPLY FORCE AT COM ---
        int forceIdx = 6 * bodyId;
        // unity coords are different from mujoco coords unity: x-right, y-up, z-forward
        // mujoco: x-forward, y-right, z-up
        MjScene.Instance.Data->xfrc_applied[forceIdx + 0] = positionOffset[0]*force.x;
        MjScene.Instance.Data->xfrc_applied[forceIdx + 1] = positionOffset[1]*force.z;
        MjScene.Instance.Data->xfrc_applied[forceIdx + 2] = positionOffset[2]*force.y + (float)gravityCompensation; // compensate for gravity
        MjScene.Instance.Data->xfrc_applied[forceIdx + 3] = rotationOffset[0]*torque.x;
        MjScene.Instance.Data->xfrc_applied[forceIdx + 4] = rotationOffset[1]*torque.z;
        MjScene.Instance.Data->xfrc_applied[forceIdx + 5] = rotationOffset[2]*torque.y;



        // ADD THIS DEBUG
        Debug.Log($"Hand pos: {handPos}, Obj pos: {objPos} Force: {force}");
    }

    unsafe void ClearForces()
    {
        int idx = 6 * mujocoBody.MujocoId;
        MjScene.Instance.Data->xfrc_applied[idx + 0] = 0;
        MjScene.Instance.Data->xfrc_applied[idx + 1] = 0;
        MjScene.Instance.Data->xfrc_applied[idx + 2] = 0;
        MjScene.Instance.Data->xfrc_applied[idx + 3] = 0;
        MjScene.Instance.Data->xfrc_applied[idx + 4] = 0;
        MjScene.Instance.Data->xfrc_applied[idx + 5] = 0;
    }

    float UpdateDebugForce(Vector3 origin, Vector3 force, Vector3 hand)
    {
        if (debugForceCylinder == null)
            return 0f;

        float mag = force.magnitude;

        if (mag < 1e-4f)
        {
            debugForceCylinder.SetActive(false);
            return 0f;
        }

        debugForceCylinder.SetActive(true);

        Vector3 dir = force.normalized;
        float length = Mathf.Sqrt(Mathf.Pow(hand.y - origin.y, 2) + Mathf.Pow(hand.x - origin.x, 2) + Mathf.Pow(hand.z - origin.z, 2));

        if (!debug)
            return length;

        // Cylinder is centered, Make it start at origin and end at hand
        debugForceCylinder.transform.position = origin + dir * (length * 0.5f);
        debugForceCylinder.transform.rotation = Quaternion.FromToRotation(Vector3.up, dir);
        debugForceCylinder.transform.localScale = new Vector3(
            0.02f,
            length*0.5f,
            0.02f
        );

        return length; // return the length for potential use
    }

}
