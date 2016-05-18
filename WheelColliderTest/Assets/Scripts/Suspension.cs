using UnityEngine;
using System.Collections;

public class Suspension : MonoBehaviour
{
    public float springForce, damperForce, springConstant, damperConstant, restLenght;

    private float previousLength, currentLength, springVelocity;
    private Rigidbody _rb;
    private Vehicle _vehicleScript;

    private void Start()
    {
        _rb = transform.parent.parent.GetComponent<Rigidbody>();
        _vehicleScript = transform.parent.parent.GetComponent<Vehicle>();
    }

    private void FixedUpdate()
    {

        RaycastHit hit;
        if (Physics.Raycast(transform.position,-transform.up, out hit, restLenght+_vehicleScript.wheelRadius))
        {
            previousLength = currentLength;
            currentLength = restLenght - (hit.distance - _vehicleScript.wheelRadius);
            springVelocity = (currentLength - previousLength) / Time.fixedDeltaTime;
            springForce = springConstant * currentLength;
            damperForce = damperConstant * springVelocity;
            _rb.AddForceAtPosition(transform.up * (springForce + damperForce), transform.position);
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        if (_vehicleScript) Gizmos.DrawLine(transform.position, transform.position + (restLenght + _vehicleScript.wheelRadius) * -Vector3.up);
    }
}
