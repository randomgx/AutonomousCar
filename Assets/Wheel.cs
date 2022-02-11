using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wheel : MonoBehaviour
{
    public WheelCollider[] wcs;
    public Transform[] wheels;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    void UpdateWheel()
    {
        Vector3 pos;
        Quaternion rot;

        for(int i = 0; i < wcs.Length; i++)
        {
            wcs[i].GetWorldPose(out pos, out rot);
            wheels[i].position = pos;
            wheels[i].rotation = rot;
        }
    }

    // Update is called once per frame
    void Update()
    {
        UpdateWheel();
    }
}
