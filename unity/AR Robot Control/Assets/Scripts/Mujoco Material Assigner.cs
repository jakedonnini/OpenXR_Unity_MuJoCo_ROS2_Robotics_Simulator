using UnityEngine;

public class MujocoMaterialAssignerByName : MonoBehaviour
{
    public Material white;
    public Material black;
    public Material lightBlue;
    public Material offWhite;
    public Material green;

    void Start()
    {
        foreach (Transform t in transform.GetComponentsInChildren<Transform>())
        {
            var renderer = t.GetComponent<MeshRenderer>();
            if (!renderer) continue;

            // Use the GameObject name to assign material
            string name = t.name.ToLower();
            if (name.Contains("link0_0") || name.Contains("link6_0") || name.Contains("hand_4"))
                renderer.material = offWhite;
            else if (name.Contains("link0_1") || name.Contains("link0_5") || name.Contains("link3_3"))
                renderer.material = black;
            else if (name.Contains("link6_7") || name.Contains("link6_8"))
                renderer.material = lightBlue;
            else if (name.Contains("link6_12"))
                renderer.material = green;
            else
                renderer.material = white;
        }
    }
}
