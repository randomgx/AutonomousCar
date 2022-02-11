using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Car : MonoBehaviour
{
    public Transform frontSensor;
    public Transform[] sensors; //0 - back / 1 - left / 2 - right
    public Transform[] laneSensors; //0 - left / 1 - right

    public float forwardSensorDistance = 100f;
    public float sideSensorDistance = 10f;

    Vector3 currentLaneCenter;

    Rigidbody _rb;
    Vector3 rotationVector;

    public float power = 931; //car power, the greater the faster it will go when throttle is 1

    [Range(0, 1)]
    public float throttle; //controls the acceleration, instant change

    [Range(0, 1)]
    public float brake; //controls the acceleration, instant change

    [Range(-50, 50)]
    public float wheelSteer;

    [Range(0, 100)]
    public float kmTargetSpeed;

    float targetSpeed;

    public float kmCurrentSpeed;
    float currentSpeed;

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

    float timer;
    bool timercount;

    public bool startTest;

    private void Start()
    {
        _rb = GetComponent<Rigidbody>();
    }

    private void Update()
    {
        kmCurrentSpeed = currentSpeed * 3.6f;
        targetSpeed = kmTargetSpeed / 3.6f;

        if(startTest)
        {
            timercount = true;
            autonomousDrive = false;
            throttle = 0;
            brake = 0;
        }

        if(timercount)
        {
            timer += Time.deltaTime;
            if (Mathf.Floor(currentSpeed) == 0)
            {
                timercount = false;
                startTest = false;
                Debug.Log(timer);
            }
        }
    }

    private void FixedUpdate()
    {
        HandleAccelerator(throttle);
        HandleBrake(brake);
        HandleSteering(wheelSteer);
        HandleSensors();
        CruiseControl();

        currentSpeed = transform.InverseTransformDirection(_rb.velocity).z;
        //currentSpeed = _rb.velocity.magnitude;
    }

    private void CruiseControl()
    {
        if (autonomousDrive)
        {
            float speed = Vector3.Magnitude(_rb.velocity);

            if (carOnSightRb != null)
            {
                float carAheadSpeed = Vector3.Magnitude(carOnSightRb.velocity);
                if (carOnSight)
                {
                    float time = ((currentCarSeparation / currentSpeed));
                    //Debug.Log("Time to collision: " + time);
                    if (speed < carAheadSpeed && carAheadSpeed <= targetSpeed + 10 && Mathf.Floor(carAheadSpeed) != 0)
                    {
                        float accelSpeed = carAheadSpeed - speed;
                        throttle = Mathf.Clamp(accelSpeed, 0, maxAutonomousDriveThrottle);
                    }
                    else if(Mathf.Floor(carAheadSpeed) <= 0)
                    {
                        float accelSpeed = carAheadSpeed - speed;
                        throttle = Mathf.Clamp(accelSpeed, 0, maxAutonomousDriveThrottle);
                    }

                    //float brakeSpeed = (currentSpeed + carAheadSpeed) / (currentCarSeparation - safeDistance);
                    float brakeSpeed = maxAutonomousDriveBrake - (currentCarSeparation / (initialCarSeparation+(safeDistance*6)));


                    if (time < 3.5f && Mathf.Floor(carAheadSpeed) != 0)
                    {
                        //Debug.Log(brakeSpeed + " brake speed");
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

                throttle = Mathf.Clamp(accelSpeed, 0, maxAutonomousDriveThrottle);
            }

            /*if (speed > targetSpeed && targetSpeed != 0)
            {
                float brakeSpeed = speed - targetSpeed;
                if (speed > targetSpeed * 3)
                {
                    HandleBrake(Mathf.Clamp(brakeSpeed, 0, maxAutonomousDriveBrake));
                }
                else
                {
                    brake = 0;
                }
                throttle = Mathf.Clamp(1 - brakeSpeed, 0, maxAutonomousDriveThrottle);
            }
            else if (speed <= targetSpeed)
            {
                brake = 0;
            }*/
        }
    }

    private void HandleAccelerator(float _throttle)
    {
        _throttle = Mathf.Clamp(_throttle, 0, 1);

        _rb.AddForce((transform.forward * (_throttle * power) * _rb.mass) * Time.deltaTime, ForceMode.Force);
        //Debug.Log((_throttle * power) * _rb.mass); Torque

        throttle = _throttle;

    }

    private void HandleBrake(float _brake)
    {
        if(currentSpeed <= 0)
        {
            return;
        }

        _brake = Mathf.Clamp(_brake, 0, 1);

        brake = _brake;

        _rb.AddForce(-transform.forward * (_brake * power) * _rb.mass * 0.7f * Time.deltaTime, ForceMode.Force);
    }

    private void HandleSteering(float _steer)
    {
        if (currentSpeed > 1f)
        {
            rotationVector = new Vector3(0, _steer, 0);
        }
        else
        {
            rotationVector = new Vector3(0, _steer/3, 0);
        }
        Quaternion deltaRotationRight = Quaternion.Euler(rotationVector * Time.deltaTime * steerSpeed);
        GetComponent<Rigidbody>().MoveRotation(transform.rotation * deltaRotationRight);
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
                if(!carOnSight)
                {
                    initialCarSeparation = forwardHit.distance;
                }
                carOnSight = true;
                carOnSightRb = forwardHit.transform.GetComponent<Rigidbody>();
                currentCarSeparation = forwardHit.distance;
                Debug.DrawRay(frontSensor.position, frontSensor.forward * forwardHit.distance, Color.red);
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
                Debug.Log("L: " + hitLeft.distance + " R: " + hitRight.distance);
                currentLaneCenter = (hitLeft.transform.position + hitRight.transform.position) / 2;
                if(!Mathf.Approximately(hitLeft.distance, hitRight.distance))
                {
                    if(hitLeft.distance > hitRight.distance)
                    {
                        float f = (hitLeft.distance / hitRight.distance+0.2f) - 1;
                        wheelSteer = Mathf.Clamp(-Mathf.Pow(f, 2) * (currentSpeed / 10), -maxSteerAngle, 0);
                        if (hitLeft.distance > hitRight.distance*2f)
                        {
                            wheelSteer = Mathf.Clamp(-Mathf.Pow(f, 2) * (currentSpeed / 10) * 2, -maxSteerAngle, 0);
                        }
                    }
                    else if(hitRight.distance > hitLeft.distance+0.2f)
                    {
                        float f = (hitRight.distance / hitLeft.distance) - 1;
                        wheelSteer = Mathf.Clamp(Mathf.Pow(f, 2)* (currentSpeed/10), 0, maxSteerAngle);
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
