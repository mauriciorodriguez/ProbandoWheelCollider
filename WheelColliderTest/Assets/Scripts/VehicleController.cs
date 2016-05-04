using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.UI;

public class VehicleController : MonoBehaviour
{
    public Text speedText;
    public List<WheelPairs> wheelPairList;

    public Vector3 centerOfMass;
    [Tooltip("0 Actua la fisica, 1 se dirige hacia donde esta mirando")]
    [Range(0, 1)]
    public float steerHelper;
    [Tooltip("0 Sin traccion, 1 Traccion completa")]
    [Range(0, 1)]
    public float tractionControl;
    public float maxSteerAngle, maxTorque, reverseTorque, brakeTorque, handbrakeTorque;
    [Tooltip("Fuerza para pegar el auto al piso")]
    public float downForce, topSpeed, slipLimit;

    private Rigidbody _rb;
    private float _currentTorque, _oldRotation, _accelInput, _brakeInput, _handbrakeInput, _steerInput;
    private float _currentSpeed { get { return _rb.velocity.magnitude * 3.6f; } }
    private float _antiRoll;

    protected void Start()
    {
        wheelPairList[0].rightWheelCollider.attachedRigidbody.centerOfMass = centerOfMass;
        handbrakeTorque = float.MaxValue;
        _rb = GetComponent<Rigidbody>();
        _currentTorque = _currentTorque - (tractionControl * _currentTorque);
        _antiRoll = 5000;//wheelPairList[0].rightWheelCollider.suspensionSpring.spring;
    }

    protected void Update()
    {
        speedText.text = "Speed " + _currentSpeed;
    }

    protected void FixedUpdate()
    {
        ClampImputValues();
        AntiRollBar();

        ApplySteering();
        SteerHelper();
        ApplyDrive();
        CapSpeed();
        HandbrakeForce();
        AddDownForce();
        ApplyPosAndRotToMesh();
        TractionControl();
    }

    protected void AntiRollBar()
    {
        WheelHit hit;
        float travelL = 1;
        float travelR = 1;
        foreach (var wheelP in wheelPairList)
        {
            var groundedL = wheelP.leftWheelCollider.GetGroundHit(out hit);
            if (groundedL) travelL = (-wheelP.leftWheelCollider.transform.InverseTransformPoint(hit.point).y - wheelP.leftWheelCollider.radius) / wheelP.leftWheelCollider.suspensionDistance;
            var groundedR = wheelP.rightWheelCollider.GetGroundHit(out hit);
            if (groundedR) travelR = (-wheelP.rightWheelCollider.transform.InverseTransformPoint(hit.point).y - wheelP.rightWheelCollider.radius) / wheelP.rightWheelCollider.suspensionDistance;

            var antiRollForce = (travelL - travelR) * _antiRoll;

            if (groundedL) _rb.AddForceAtPosition(wheelP.leftWheelCollider.transform.up * -antiRollForce, wheelP.leftWheelCollider.transform.position);
            if (groundedR) _rb.AddForceAtPosition(wheelP.rightWheelCollider.transform.up * antiRollForce, wheelP.rightWheelCollider.transform.position);
        }
    }

    protected void TractionControl()
    {
        WheelHit hit;
        WheelHit hit2;
        foreach (var wheelP in wheelPairList)
        {
            wheelP.rightWheelCollider.GetGroundHit(out hit);
            wheelP.leftWheelCollider.GetGroundHit(out hit2);
            AdjustTorque(hit.forwardSlip);
            AdjustTorque(hit2.forwardSlip);
        }
    }

    protected void AdjustTorque(float forwardSlip)
    {
        if (forwardSlip >= slipLimit && _currentTorque >= 0)
        {
            _currentTorque -= 10 * tractionControl;
        }
        else
        {
            _currentTorque += 10 * tractionControl;
            if (_currentTorque > maxTorque)
            {
                _currentTorque = maxTorque;
            }
        }
    }

    protected void CapSpeed()
    {
        float speed = _rb.velocity.magnitude;
        speed *= 3.6f;
        if (speed > topSpeed)
        {
            _rb.velocity = (topSpeed / 3.6f) * _rb.velocity.normalized;
        }
    }

    protected void ApplyPosAndRotToMesh()
    {
        foreach (var wheelP in wheelPairList)
        {
            Quaternion rot;
            Vector3 pos;
            wheelP.rightWheelCollider.GetWorldPose(out pos, out rot);
            wheelP.rightWheelModel.transform.position = pos;
            wheelP.rightWheelModel.transform.rotation = rot;
            wheelP.leftWheelCollider.GetWorldPose(out pos, out rot);
            wheelP.leftWheelModel.transform.position = pos;
            wheelP.leftWheelModel.transform.rotation = rot;
        }
    }

    protected void ApplySteering()
    {
        foreach (var wheelP in wheelPairList)
        {
            if (wheelP.steer)
            {
                wheelP.leftWheelCollider.steerAngle = _steerInput * maxSteerAngle;
                wheelP.rightWheelCollider.steerAngle = _steerInput * maxSteerAngle;
            }
        }
    }

    protected void HandbrakeForce()
    {
        if (_handbrakeInput > 0)
        {
            var hbTorque = _handbrakeInput * handbrakeTorque;
            foreach (var wheelP in wheelPairList)
            {
                wheelP.rightWheelCollider.brakeTorque = hbTorque;
                wheelP.leftWheelCollider.brakeTorque = hbTorque;
            }
        }
    }

    protected void ClampImputValues()
    {
        _steerInput = Mathf.Clamp(Input.GetAxis("Horizontal"), -1, 1);
        _accelInput = Mathf.Clamp(Input.GetAxis("Vertical"), 0, 1);
        _brakeInput = -1 * Mathf.Clamp(Input.GetAxis("Vertical"), -1, 0);
        _handbrakeInput = Mathf.Clamp(Input.GetAxis("Jump"), 0, 1);
    }

    protected void ApplyDrive()
    {
        float thrustTorque = _accelInput * (_currentTorque / 4);
        print(thrustTorque);
        foreach (var wheelP in wheelPairList)
        {
            wheelP.rightWheelCollider.motorTorque = thrustTorque;
            wheelP.leftWheelCollider.motorTorque = thrustTorque;
        }

        foreach (var wheelP in wheelPairList)
        {
            if (_currentSpeed > 5 && Vector3.Angle(transform.forward, _rb.velocity) < 50)
            {
                wheelP.rightWheelCollider.brakeTorque = brakeTorque * _brakeInput;
                wheelP.leftWheelCollider.brakeTorque = brakeTorque * _brakeInput;
            }
            else if (_brakeInput > 0)
            {
                wheelP.rightWheelCollider.brakeTorque = 0;
                wheelP.leftWheelCollider.brakeTorque = 0;
                wheelP.rightWheelCollider.motorTorque = -reverseTorque * _brakeInput;
                wheelP.leftWheelCollider.motorTorque = -reverseTorque * _brakeInput;
            }
        }
    }

    protected void AddDownForce()
    {
        wheelPairList[0].rightWheelCollider.attachedRigidbody.AddForce(-transform.up * downForce * wheelPairList[0].rightWheelCollider.attachedRigidbody.velocity.magnitude);
    }

    protected void SteerHelper()
    {
        foreach (var wheelP in wheelPairList)
        {
            WheelHit hit;
            WheelHit hit2;
            wheelP.rightWheelCollider.GetGroundHit(out hit);
            wheelP.leftWheelCollider.GetGroundHit(out hit2);
            if (hit.normal == Vector3.zero || hit2.normal == Vector3.zero) return;
        }

        if (Mathf.Abs(_oldRotation - transform.eulerAngles.y) < 10)
        {
            var turnAdjust = (transform.eulerAngles.y - _oldRotation) * steerHelper;
            Quaternion velRotation = Quaternion.AngleAxis(turnAdjust, Vector3.up);
            _rb.velocity = velRotation * _rb.velocity;
        }
        _oldRotation = transform.eulerAngles.y;
    }

    /*public float maxMotorTorque = 10;
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
        _steer = Mathf.Clamp(Input.GetAxis("Horizontal"), -1, 1);
        _forward = Mathf.Clamp(Input.GetAxis("Vertical"), 0, 1);
        _back = -1 * Mathf.Clamp(Input.GetAxis("Vertical"), -1, 0);
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
            

                wheelP.leftWheelCollider.motorTorque = maxMotorTorque * _motor;
                wheelP.rightWheelCollider.motorTorque = maxMotorTorque * _motor;
            

            if (wheelP.steer)
            {
                //maxSteerAngle = (maxSteerAngle * (_spe)
                wheelP.leftWheelCollider.steerAngle = maxSteerAngle * _steer;
                wheelP.rightWheelCollider.steerAngle = maxSteerAngle * _steer;
            }
        }
    }*/
}
