using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class AxleInfo
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
}

public class Car2 : MonoBehaviour
{
    public float maxSteeringAngle;

    public float enginePower = 1500; //engine power, per wheel
    public float brakeTorque;

    [Range(0, 1)]
    public float throttle; //controls the acceleration, instant change

    [Range(0, 1)]
    public float brake; //controls the acceleration, instant change

    [Range(-1, 1)]
    public float wheelSteer;

    [Range(0, 100)]
    public float kmTargetSpeed;

    Rigidbody _rb;

    public List<AxleInfo> axleInfos;

    float currentSpeed;
    public float kmCurrentSpeed;
    float targetSpeed;

    public bool autonomousDrive;
    public float maxAutonomousDriveThrottle = .5f;
    public float maxAutonomousDriveBrake = .1f;
    public float maxSteerAngle = 30;
    public float steerSpeed = 0.56f;
    public bool autoLaneFollow;

    public bool carOnSight;
    public Rigidbody carOnSightRb;

    float currentCarSeparation;
    float initialCarSeparation;
    public float safeDistance;

    public Transform frontSensor;
    public Transform[] sensors; //0 - back / 1 - left / 2 - right
    public Transform[] laneSensors; //0 - left / 1 - right

    public float forwardSensorDistance = 100f;
    public float sideSensorDistance = 10f;

    [HideInInspector]
    public float autopilotThrottleDemand;
    public float autopilotBrakeDemand;

    Vector3 currentLaneCenter;

    void Start()
    {
        _rb = GetComponent<Rigidbody>();
    }

    private void Update()
    {
        kmCurrentSpeed = currentSpeed * 3.6f;
        targetSpeed = kmTargetSpeed / 3.6f;
    }

    void FixedUpdate()
    {
        HandleAccelerator(throttle);
        HandleBrake(brake);
        HandleSensors();
        CruiseControl();

        currentSpeed = transform.InverseTransformDirection(_rb.velocity).z;
    }

    private void CruiseControl()
    {
        if (autonomousDrive)
        {
            float speed = Vector3.Magnitude(_rb.velocity);

            if (carOnSightRb != null)
            {
                float carAheadSpeed = carOnSightRb.transform.InverseTransformDirection(carOnSightRb.velocity).z;
                if (carOnSight)
                {
                    float time = ((currentCarSeparation / currentSpeed));
                    if (speed < carAheadSpeed && carAheadSpeed <= targetSpeed + 10 && Mathf.Floor(carAheadSpeed) > 2)
                    {
                        float accelSpeed = carAheadSpeed - speed;
                        autopilotThrottleDemand = accelSpeed;
                        throttle = Mathf.Clamp(accelSpeed, 0, maxAutonomousDriveThrottle);
                    }
                    else
                    {
                        if(time < 10 & speed > 15)
                        {
                            throttle = 0;
                        }

                        if(speed < 1)
                        {
                            throttle = 0;
                            brake = 0;
                        }
                    }

                    //float brakeSpeed = (currentSpeed + carAheadSpeed) / (currentCarSeparation - safeDistance);

                    if(speed < 0.2f)
                    {
                        brake = 0;
                    }
                    else if (Mathf.Floor(carAheadSpeed) < 3 && currentCarSeparation < 7 && speed < 10)
                    {
                        Debug.Log("Getting closer, applying more brake, but we're at a lower speed");
                        float brakeSpeed = maxAutonomousDriveBrake - (currentCarSeparation / (initialCarSeparation));
                        autopilotBrakeDemand = brakeSpeed;
                        brake = Mathf.Clamp(brakeSpeed, 0, maxAutonomousDriveBrake);
                    }
                    else if (Mathf.Floor(carAheadSpeed) < 5 && currentCarSeparation < 10 && speed > 10)
                    {
                        Debug.Log("Getting closer, applying more brake, we're fast");
                        float brakeSpeed = maxAutonomousDriveBrake - (currentCarSeparation / (initialCarSeparation/0.13f));
                        autopilotBrakeDemand = brakeSpeed;
                        brake = Mathf.Clamp(brakeSpeed, 0, maxAutonomousDriveBrake);
                    }
                    else if(Mathf.Floor(carAheadSpeed) < 1 && currentCarSeparation < 50 && currentCarSeparation > 10)
                    {
                        Debug.Log("Long distance brake");
                        float brakeSpeed = maxAutonomousDriveBrake - (currentCarSeparation / (initialCarSeparation/3f));
                        autopilotBrakeDemand = brakeSpeed;
                        brake = Mathf.Clamp(brakeSpeed, 0, maxAutonomousDriveBrake);
                    }
                    else if(Mathf.Floor(carAheadSpeed) > 2)
                    {
                        float brakeSpeed = maxAutonomousDriveBrake - (currentCarSeparation / (initialCarSeparation + (safeDistance * 6)));
                        autopilotBrakeDemand = brakeSpeed;
                        brake = Mathf.Clamp(brakeSpeed, 0, maxAutonomousDriveBrake);
                    }
                    else
                    {
                        brake = 0;
                    }

                }
            }

            if (carOnSight)
                return;

            if (targetSpeed != 0)
            {
                float accelSpeed = targetSpeed - speed;
                autopilotThrottleDemand = accelSpeed;

                throttle = Mathf.Clamp(accelSpeed, 0, maxAutonomousDriveThrottle);
            }
        }
    }

    private void HandleAccelerator(float _throttle)
    {
        _throttle = Mathf.Clamp(_throttle, 0, 1);

        float motor;
        float steering;

        if (autonomousDrive)
        {
            motor = enginePower * _throttle;
            steering = wheelSteer * maxSteeringAngle;
        }
        else
        {
            if (Input.GetAxisRaw("Vertical") > 0)
            {
                motor = enginePower * Input.GetAxis("Vertical");
            }
            else if (Input.GetAxisRaw("Vertical") < 0)
            {
                motor = 1;
                brake = 1;
            }
            else
            {
                motor = 1;
                brake = 0;
            }
            steering = maxSteerAngle * Input.GetAxis("Horizontal");
        }

        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor && brake == 0)
            {
                axleInfo.rightWheel.motorTorque = motor;
                axleInfo.leftWheel.motorTorque = motor;
            }
        }
        throttle = _throttle;
    }

    private void HandleBrake(float _brake)
    {
        _brake = Mathf.Clamp(_brake, 0, 1);

        foreach (AxleInfo axleInfo in axleInfos)
        {
            axleInfo.leftWheel.brakeTorque = brakeTorque * _brake;
            axleInfo.rightWheel.brakeTorque = brakeTorque * _brake;
        }

        brake = _brake;
    }

    private void HandleSensors()
    {
        int layerMaskLane = 1 << 6;
        layerMaskLane = ~layerMaskLane;

        CalculateLane();
        CalculateForwardSensor(layerMaskLane);

        foreach (Transform t in sensors)
        {
            RaycastHit hit;
            if (Physics.Raycast(t.position, t.forward, out hit, sideSensorDistance, layerMaskLane))
            {
                if (hit.transform.CompareTag("Car"))
                {
                    Debug.DrawRay(t.position, t.forward * hit.distance, Color.red);
                    Debug.DrawRay(t.position, t.forward * hit.distance, Color.red);
                }
            }
        }
    }

    private void CalculateForwardSensor(int _mask)
    {
        RaycastHit forwardHit;

        if (Physics.Raycast(frontSensor.position, frontSensor.forward, out forwardHit, forwardSensorDistance, _mask))
        {
            if (forwardHit.transform.CompareTag("Car"))
            {
                if (!carOnSight)
                {
                    initialCarSeparation = forwardHit.distance;
                }
                carOnSight = true;
 
                carOnSightRb = forwardHit.transform.GetComponent<Rigidbody>();
                currentCarSeparation = forwardHit.distance;
                Debug.DrawRay(frontSensor.position, frontSensor.forward * forwardHit.distance, Color.red);
            }
        }
        else
        {
            carOnSight = false;
            carOnSightRb = null;
        }
    }

    private void CalculateLane()
    {
        int layerMask = 1 << 8;
        layerMask = ~layerMask;

        RaycastHit hitLeft;
        RaycastHit hitRight;

        if (Physics.Raycast(laneSensors[0].position, laneSensors[0].forward, out hitLeft, 30f, layerMask))
        {
            if (hitLeft.transform.CompareTag("Lane"))
            {
                Debug.DrawRay(laneSensors[0].position, laneSensors[0].forward * hitLeft.distance, Color.red);
                Debug.DrawRay(laneSensors[0].position, laneSensors[0].forward * hitLeft.distance, Color.red);
            }
        }
        else
        {
            Debug.DrawRay(laneSensors[0].position, laneSensors[0].forward * 50, Color.blue);
            Debug.DrawRay(laneSensors[0].position, laneSensors[0].forward * 50, Color.blue);
        }

        if (Physics.Raycast(laneSensors[1].position, laneSensors[1].forward, out hitRight, 30f, layerMask))
        {
            if (hitRight.transform.CompareTag("Lane"))
            {
                Debug.DrawRay(laneSensors[1].position, laneSensors[1].forward * hitRight.distance, Color.red);
                Debug.DrawRay(laneSensors[1].position, laneSensors[1].forward * hitRight.distance, Color.red);
            }
        }
        else
        {
            Debug.DrawRay(laneSensors[1].position, laneSensors[1].forward * 50, Color.blue);
            Debug.DrawRay(laneSensors[1].position, laneSensors[1].forward * 50, Color.blue);
        }

        if (autoLaneFollow)
        {
            if (hitLeft.transform != null && hitRight.transform != null)
            {
                //Debug.Log("L: " + hitLeft.distance + " R: " + hitRight.distance);
                currentLaneCenter = (hitLeft.transform.position + hitRight.transform.position) / 2;
                if (!Mathf.Approximately(hitLeft.distance, hitRight.distance))
                {
                    if (hitLeft.distance > hitRight.distance)
                    {
                        float f = (hitLeft.distance / hitRight.distance + 0.2f) - 1;
                        wheelSteer = Mathf.Clamp(-Mathf.Pow(f, 2) * (currentSpeed / 10), -maxSteerAngle, 0);
                        if (hitLeft.distance > hitRight.distance * 2f)
                        {
                            wheelSteer = Mathf.Clamp(-Mathf.Pow(f, 2) * (currentSpeed / 10) * 2, -maxSteerAngle, 0);
                        }
                    }
                    else if (hitRight.distance > hitLeft.distance + 0.2f)
                    {
                        float f = (hitRight.distance / hitLeft.distance) - 1;
                        wheelSteer = Mathf.Clamp(Mathf.Pow(f, 2) * (currentSpeed / 10), 0, maxSteerAngle);
                        if (hitRight.distance > hitLeft.distance * 2f)
                        {
                            wheelSteer = Mathf.Clamp(Mathf.Pow(f, 2) * (currentSpeed / 10) * 2, 0, maxSteerAngle);
                        }
                    }
                    else
                    {
                        wheelSteer = 0;
                    }
                }
                else
                {
                    wheelSteer = 0;
                }
            }
        }
    }
}
