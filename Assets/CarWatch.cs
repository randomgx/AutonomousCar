using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarWatch : MonoBehaviour
{
    Car2 car;

    public Vector3 position;
    public float speed;
    public float throttle;
    public float brake;
    public bool autonomouslyDriving;
    public float autopilotBrakeDemand;
    public float autopilotThrottleDemand;

    public bool autopilotCarAhead;
    public float autopilotCarAheadSpeed;

    void Start()
    {
        car = GetComponent<Car2>();
    }

    void Update()
    {
        position = car.transform.position;
        autopilotCarAhead = car.carOnSight;
        speed = car.kmCurrentSpeed / 3.6f;
        throttle = car.throttle;
        brake = car.brake;
        autonomouslyDriving = car.autonomousDrive;
        autopilotThrottleDemand = car.autopilotThrottleDemand;
        autopilotBrakeDemand = car.autopilotBrakeDemand;


        if(car)
        {
            if(car.carOnSight)
            {
                autopilotCarAheadSpeed = car.carOnSightRb.transform.InverseTransformDirection(car.carOnSightRb.velocity).z;
            }
        }
    }
}
