using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine.UI;

public abstract class Vehicle : MonoBehaviour
{
    /*public Text textSpeed;
    public Transform centerOfMass;
    public List<WheelPair> wheelPairList;
    public float forceMultiplierPerWheel, maxSteerAngle;
    public float downForce;

    private Rigidbody _rb;
    private Vector3 _currentSpeed;

    private void Start()
    {
        _rb = GetComponent<Rigidbody>();
        SetRigidbodyValues();
    }

    private void Update()
    {
        _currentSpeed = transform.InverseTransformDirection(_rb.velocity);
        textSpeed.text = (int)_currentSpeed.z + "";
    }

    private void SetRigidbodyValues()
    {
        _rb.centerOfMass = centerOfMass ? centerOfMass.localPosition : Vector3.zero;
        _rb.inertiaTensorRotation = Quaternion.identity;
        _rb.inertiaTensor = new Vector3(1, 1, 2) * _rb.mass;
    }

    public void Move(float accel, float brake, float handbrake, float steer)
    {
        accel *= forceMultiplierPerWheel;
        steer *= maxSteerAngle;
        _rb.AddTorque(0, steer, 0);
        bool isGrounded = false;
        foreach (var wPair in wheelPairList)
        {
            if (wPair.leftWheel.GetComponent<Wheel>().HasContactWithSurface() && wPair.rightWheel.GetComponent<Wheel>().HasContactWithSurface())
            {
                isGrounded = true;
                break;
            }
        }
        if (isGrounded)
        {
            _rb.AddRelativeForce(0, 0, accel);
        }
        ApplyDrive(accel,brake);
        ApplySteer(steer);
        AddDownForce();
    }

    private void ApplyDrive(float accel, float brake)
    {

    }

    private void AddDownForce()
    {
        _rb.AddForce(-transform.up * downForce * _rb.velocity.magnitude);
    }

    private void ApplySteer(float steer)
    {
        wheelPairList[0].leftWheel.transform.localEulerAngles = Vector3.up * steer;
        wheelPairList[0].rightWheel.transform.localEulerAngles = Vector3.up * steer;
    }*/

    public Text speedText;
    public float wheelRadius, downForce;
    public Transform centerOfMass;
    public List<Transform> wheelMeshList;
    public float maxAngleSteer, maxForce;

    protected Rigidbody _rb;

    protected void Start()
    {
        _rb = GetComponent<Rigidbody>();
        _rb.centerOfMass = centerOfMass.localPosition;
    }

    protected void Update()
    {
        speedText.text = _rb.velocity.magnitude + "";
        UpdateTyres();
    }

    protected void UpdateTyres()
    {
        /*var angularVelocity = _rb.angularVelocity.magnitude;
        var rpm = angularVelocity / (2 * Mathf.PI) * 60;
        foreach (var wheel in wheelMeshList)
        {
            wheel.Rotate(rpm*Time.deltaTime, 0, 0);
        }*/
    }

    public void Move(float accelInput, float brakeInput, float handbrakeInput, float steerInput)
    {
        var steerAngle = steerInput * maxAngleSteer;
        var forwardForce = accelInput * maxForce;
        ApplyDrive(forwardForce);
        ApplySteer(steerAngle);
        AddDownForce();
    }

    protected void AddDownForce()
    {
        _rb.AddForce(-transform.up * downForce * _rb.velocity.magnitude);
    }

    protected void ApplySteer(float steerAngle)
    {
        _rb.AddRelativeTorque(0, steerAngle, 0);
    }

    protected void ApplyDrive(float forwardForce)
    {
        _rb.AddRelativeForce(0, 0, forwardForce);
    }
}
