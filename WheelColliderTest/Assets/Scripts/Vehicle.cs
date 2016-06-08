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
    public List<Suspension> wheelSuspensionList;
    public float maxSteerForce, maxForce, brakeForce;

    protected Rigidbody _rb;
    protected float _velZ;
    protected bool _isGrounded;

    protected void Start()
    {
        _rb = GetComponent<Rigidbody>();
        _rb.centerOfMass = centerOfMass.localPosition;
    }

    protected void Update()
    {
        speedText.text = (int)(_velZ * K.KPH_TO_MPS_MULTIPLIER) + "";

        UpdateTyres();
    }

    protected void UpdateTyres()
    {
        /*var angularVelocity = _rb.angularVelocity.magnitude;
        var rpm = angularVelocity / (2 * Mathf.PI);
        foreach (var wheel in wheelMeshList)
        {
            wheel.localRotation = Quaternion.Euler(rpm*Time.deltaTime,0,0);
        }*/
    }

    public void Move(float accelInput, float brakeInput, float handbrakeInput, float steerInput)
    {
        _velZ = transform.InverseTransformDirection(_rb.velocity).z;
        var steerForce = steerInput * maxSteerForce;
        var forwardForce = accelInput * maxForce;
        var brakeForce = brakeInput * this.brakeForce;
        _rb.drag = K.AIR_DRAG;
        foreach (var wheel in wheelSuspensionList)
        {
            _isGrounded = false;
            if (wheel.IsGrounded())
            {
                _isGrounded = true;
                _rb.drag = 1;
                //CalculateSteerForceWithVelocity(out steerForce,steerInput); 
                break;
            }
        }
        ApplyDrive(forwardForce, accelInput, brakeForce);
        ApplySteer(steerForce, steerInput);
        //Drag(accelInput, brakeInput);
        AddDownForce();
        CapSpeed();
    }

    protected void CalculateSteerForceWithVelocity(out float sf, float si)
    {
        var a = (_velZ / topSpeed);
        sf = (a * maxSteerForce);
        print(sf);
    }

    protected void Drag(float a, float b)
    {
        /*float tempFriction;
        if (_isGrounded)
        {
            tempFriction = friction;
        }
        else
        {
            tempFriction = friction * .1f;
        }*/

        var vel = _rb.velocity;
        vel.x *= friction;
        vel.z *= friction;
        _rb.velocity = vel;
        _rb.angularVelocity *= friction;
    }

    protected void AddDownForce()
    {
        _rb.AddForce(-Vector3.up * downForce);
    }

    protected void ApplySteer(float steerF, float steerI)
    {
        //print("Abs(steerForce):" + Mathf.Abs(steerF) + " * ((Mathf.Floor(_velZ) * 3.6f) / topSpeed):" + ((Mathf.Floor(_velZ) * 3.6f) / topSpeed) + " = " + Mathf.Abs(steerF) * ((Mathf.Floor(_velZ)*3.6f) / topSpeed));
        var tempForce = Mathf.Abs(steerF) * ((Mathf.Floor(_velZ) * 3.6f) / topSpeed);
        if (tempForce < maxSteerForce * K.MIN_FORCE_MULTIPLIER && steerI != 0) tempForce = maxSteerForce * K.MIN_FORCE_MULTIPLIER;
        if (_velZ > 1)
        {
            if (steerI > 0)
            {
                _rb.AddForceAtPosition(leftTurnWheelPosition.right * tempForce, leftTurnWheelPosition.position, ForceMode.Acceleration);
            }
            else if (steerI < 0)
            {
                _rb.AddForceAtPosition(-rightWheelTurnPosition.right * tempForce, rightWheelTurnPosition.position, ForceMode.Acceleration);
            }
        }
        else if (_velZ < -1)
        {
            if (steerI < 0)
            {
                _rb.AddForceAtPosition(leftTurnWheelPosition.right * tempForce, leftTurnWheelPosition.position, ForceMode.Acceleration);
            }
            else if (steerI > 0)
            {
                _rb.AddForceAtPosition(-rightWheelTurnPosition.right * tempForce, rightWheelTurnPosition.position, ForceMode.Acceleration);
            }
        }
    }

    protected void ApplyDrive(float forwardForce, float accI, float brakeF)
    {
        //print("VelZ:" + Mathf.Floor(_velZ) + " / topSpeed:" + topSpeed + " = " + (Mathf.Floor(_velZ) / topSpeed));
        var tempForce = forwardForce * (Mathf.Floor(_velZ) / topSpeed);
        if (tempForce < maxForce * K.MIN_FORCE_MULTIPLIER && accI > 0) tempForce = maxForce * K.MIN_FORCE_MULTIPLIER;
        if (brakeF < 0)
        {
            _rb.AddRelativeForce(0, 0, brakeF);
        }
        else {
            if (_isGrounded) _rb.AddRelativeForce(0, 0, tempForce);
            else _rb.AddRelativeForce(0, 0, tempForce * 1.1f);
        }
    }

    protected void CapSpeed()
    {
        if (_velZ > (topSpeed / K.KPH_TO_MPS_MULTIPLIER))
        {
            _rb.velocity = (topSpeed / K.KPH_TO_MPS_MULTIPLIER) * _rb.velocity.normalized;
            _velZ = transform.InverseTransformDirection(_rb.velocity).z;
        }
    }
}
