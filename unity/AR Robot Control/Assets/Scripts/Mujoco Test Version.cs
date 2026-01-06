using UnityEngine;
using Mujoco;

public class MujocoTypeCheck : MonoBehaviour
{
    void Start()
    {
        Debug.Log(typeof(MjScene).FullName);
    }
}
