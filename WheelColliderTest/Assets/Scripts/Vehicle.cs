using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine.UI;

public abstract class Vehicle : MonoBehaviour
{
    public Text textSpeed;
    public List<WheelPair> wheelPairList;
    public float forceMultiplierPerWheel, maxSteerAngle;

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
        textSpeed.text = (int)_currentSpeed.z+"";
    }

    private void SetRigidbodyValues()
    {
        _rb.centerOfMass = new Vector3(0,0,0);
        _rb.inertiaTensorRotation = Quaternion.identity;
        _rb.inertiaTensor = new Vector3(1, 1, 2) * _rb.mass;
    }

    public void Move(float accel, float brake, float handbrake, float steer)
    {
        accel *= forceMultiplierPerWheel;
        steer *= maxSteerAngle;
        _rb.AddTorque(0,steer,0);
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
        ApplySteer(steer);
    }

    private void ApplySteer(float steer)
    {
        wheelPairList[0].leftWheel.transform.localEulerAngles += Vector3.up * steer;
        wheelPairList[0].rightWheel.transform.localEulerAngles += Vector3.up * steer;
    }
}
