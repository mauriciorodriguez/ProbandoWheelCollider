using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.UI;

public class VehicleController : MonoBehaviour 
{
    public Text speedText;
    public List<WheelPairs> wheelPairList;
    public float maxMotorTorque = 10;
    public float maxBrakeTorque = 100;
    public float maxSteerAngle = 20;

    private float _steer, _forward, _back, _motor, _brake, _speed = 0;
    private bool _reverse;
    private Rigidbody _rb;
    private Vector3 _centerOfMassPosition, _relativeSpeed;

    private void Start()
    {
        _centerOfMassPosition = transform.Find("CenterOfMass").GetComponent<Transform>().localPosition;
        _rb = GetComponent<Rigidbody>();
    }

    private void Update()
    {
        speedText.text = "Speed: " + (int)_speed;
        ApplyGraphicsToWheel();
    }

    private void FixedUpdate()
    {
        _speed = _rb.velocity.magnitude;    
        _steer = Mathf.Clamp(Input.GetAxis("Horizontal"),-1,1);
        _forward = Mathf.Clamp(Input.GetAxis("Vertical"),0,1);
        _back = -1 * Mathf.Clamp(Input.GetAxis("Vertical"),-1,0);
        _speed *= 3.6f;
        if (_speed > 120)
            _rb.velocity = (120 / 3.6f) * _rb.velocity.normalized;
        if (_speed < 1 && _speed > -1)
        {
            if (_back > 0) { _reverse = true; }
            if (_forward > 0) { _reverse = false; }
        }

        if (_reverse)
        {
            _motor = -1 * _back;
            _brake = _forward;
        }
        else
        {
            _motor = _forward;
;
            _brake = _back;
        }

        ApplyTorqueToWheel();
    }

    private void ApplyGraphicsToWheel()
    {
        foreach (var wheelP in wheelPairList)
        {
            Quaternion rot;
            Vector3 pos;
            wheelP.leftWheelCollider.GetWorldPose(out pos, out rot);
            wheelP.leftWheelModel.transform.position = pos;
            wheelP.leftWheelModel.transform.rotation = rot;

            wheelP.rightWheelCollider.GetWorldPose(out pos, out rot);
            wheelP.rightWheelModel.transform.position = pos;
            wheelP.rightWheelModel.transform.rotation = rot;
        }
    }

    private void ApplyTorqueToWheel()
    {
        foreach (var wheelP in wheelPairList)
        {
            wheelP.leftWheelCollider.brakeTorque = maxBrakeTorque * _brake;
            wheelP.rightWheelCollider.brakeTorque = maxBrakeTorque * _brake;
            if (wheelP.motor)
            {
                wheelP.leftWheelCollider.motorTorque = maxMotorTorque * _motor;
                wheelP.rightWheelCollider.motorTorque = maxMotorTorque * _motor;                
            }
            if (wheelP.steer)
            {
                //maxSteerAngle = (maxSteerAngle * (_spe)
                wheelP.leftWheelCollider.steerAngle = maxSteerAngle * _steer;
                wheelP.rightWheelCollider.steerAngle = maxSteerAngle * _steer;
            }
        }
    }
}
