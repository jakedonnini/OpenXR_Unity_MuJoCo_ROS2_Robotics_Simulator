using UnityEngine;
using Oculus.Interaction;
using Mujoco;

public class MujocoGrabbableSync : MonoBehaviour
{
    public MjBody mujocoBody;

    public OVRHand rightHand;
    public OVRHand leftHand;

    public float kp = 10f;   // stiffness
    public float kd = 1f;    // damping
    public float maxForce = 10f;

    public float debugForceScale = 0.1f;
    public float[] positionOffset = new float[3] {0f, 0f, 0f};
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
            ApplyForceFromHand(rightHand);
        }
        else if (IsGripping(leftHand))
        {
            ApplyForceFromHand(leftHand);
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
    
    unsafe void ApplyForceFromHand(OVRHand hand)
    {
        Vector3 handPos = hand.transform.position;
        Vector3 objPos  = cubeParent.position;

        // --- POSITION ERROR ---
        Vector3 posError = handPos - objPos;

        // --- OBJECT VELOCITY (MuJoCo world frame) ---
        int bodyId = mujocoBody.MujocoId;
        int velAdr = MjScene.Instance.Model->body_dofadr[bodyId];

        Vector3 objVel = Vector3.zero;
        if (velAdr >= 0)
        {
            objVel = new Vector3(
                (float)MjScene.Instance.Data->qvel[velAdr + 0],
                (float)MjScene.Instance.Data->qvel[velAdr + 1],
                (float)MjScene.Instance.Data->qvel[velAdr + 2]
            );
        }

        // --- PD FORCE ---
        Vector3 force = kp * posError - kd * objVel;
        force = Vector3.ClampMagnitude(force, maxForce);
        UpdateDebugForce(objPos, force, handPos);

        Vector3 dir = force.normalized;

        // --- APPLY FORCE AT COM ---
        int forceIdx = 6 * bodyId;
        MjScene.Instance.Data->xfrc_applied[forceIdx + 0] = positionOffset[0]*force.x;
        MjScene.Instance.Data->xfrc_applied[forceIdx + 1] = positionOffset[1]*force.y;
        MjScene.Instance.Data->xfrc_applied[forceIdx + 2] = positionOffset[2]*force.z;

        // ADD THIS DEBUG
        Debug.Log($"Hand pos: {handPos}, Obj pos: {objPos} Force: {force}");
        // Debug.Log($"Position error: {posError}");
        // Debug.Log($"Calculated force: {force}");
        // Debug.Log($"Body ID: {bodyId}, Force index: {forceIdx}");
        
        // // Check if force is actually being written
        // Debug.Log($"xfrc_applied[{forceIdx}] = {force.x}");
        // Debug.Log($"xfrc_applied[{forceIdx+1}] = {force.y}");
        // Debug.Log($"xfrc_applied[{forceIdx+2}] = {force.z}");
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

    void UpdateDebugForce(Vector3 origin, Vector3 force, Vector3 hand)
    {
        if (debugForceCylinder == null)
            return;

        float mag = force.magnitude;

        if (mag < 1e-4f)
        {
            debugForceCylinder.SetActive(false);
            return;
        }

        debugForceCylinder.SetActive(true);

        Vector3 dir = force.normalized;
        // float length = mag * debugForceScale;
        float length = Mathf.Sqrt(Mathf.Pow(hand.y - origin.y, 2) + Mathf.Pow(hand.x - origin.x, 2) + Mathf.Pow(hand.z - origin.z, 2));

        // Cylinder is centered, Make it start at origin and end at hand
        debugForceCylinder.transform.position = origin + dir * (length * 0.5f);
        debugForceCylinder.transform.rotation = Quaternion.FromToRotation(Vector3.up, dir);
        debugForceCylinder.transform.localScale = new Vector3(
            0.02f,
            length,
            0.02f
        );
    }

}
