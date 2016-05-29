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
    public float wheelRadius, downForce, topSpeed;
    public Transform leftTurnWheelPosition, rightWheelTurnPosition;
    [Tooltip("1 sin friccion")]
    [Range(0, 1)]
    public float friction;
    public Transform centerOfMass;
    public List<Transform> wheelMeshList;
    public float maxSteerForce, maxForce, brakeForce;

    protected Rigidbody _rb;
    protected float _velZ;

    protected void Start()
    {
        _rb = GetComponent<Rigidbody>();
        _rb.centerOfMass = centerOfMass.localPosition;
    }

    protected void Update()
    {
        _velZ = transform.InverseTransformDirection(_rb.velocity).z;
        speedText.text = _velZ + "";
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
        var steerAngle = steerInput * maxSteerForce;
        var forwardForce = accelInput * maxForce;
        var brakeF = brakeInput * brakeForce;
        ApplyDrive(forwardForce, brakeF);
        ApplySteer(steerAngle);
        AddDownForce();
        Drag(accelInput, brakeInput);
        CapSpeed();
    }

    protected void Drag(float a, float b)
    {
        if (a == 0 && b == 0)
        {
            var vel = _rb.velocity;
            vel.x *= friction;
            vel.z *= friction;
            _rb.velocity = vel;
            _rb.angularVelocity *= friction;
        }
    }

    protected void AddDownForce()
    {
        _rb.AddForce(-Vector3.up * downForce * _rb.velocity.magnitude);
    }

    protected void ApplySteer(float steerAngle)
    {
        if (_rb.velocity.magnitude > 1)
        {
            if (steerAngle > 0)
            {
                _rb.AddForceAtPosition(leftTurnWheelPosition.right * maxSteerForce, leftTurnWheelPosition.position);
            }
            else if (steerAngle < 0)
            {
                _rb.AddForceAtPosition(-rightWheelTurnPosition.right * maxSteerForce, rightWheelTurnPosition.position);

            }
        }
    }

    protected void ApplyDrive(float forwardForce, float brake)
    {
        if (brake < 0)
        {
            _rb.AddRelativeForce(0, 0, brake / 4);
        }
        else {
            _rb.AddRelativeForce(0, 0, forwardForce);
        }
    }

    protected void CapSpeed()
    {
        if (_rb.velocity.magnitude > topSpeed) _rb.velocity = topSpeed * _rb.velocity.normalized;
    }
}
