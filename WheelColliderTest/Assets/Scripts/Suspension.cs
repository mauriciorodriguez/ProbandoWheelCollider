using UnityEngine;
using System.Collections;

public class Suspension : MonoBehaviour
{
    public float springForce, damperForce, springConstant, damperConstant, restLenght;

    private float previousLength, currentLength, springVelocity;
    private Rigidbody _rb;
    private Vehicle _vehicleScript;
    private bool _isGrounded;

    private void Start()
    {
        _isGrounded = false;
        _rb = transform.parent.parent.GetComponent<Rigidbody>();
        _vehicleScript = transform.parent.parent.GetComponent<Vehicle>();
    }

    private void FixedUpdate()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, -transform.up, out hit, restLenght + _vehicleScript.wheelRadius))
        {
            previousLength = currentLength;
            currentLength = restLenght - (hit.distance - _vehicleScript.wheelRadius);
            springVelocity = (currentLength - previousLength) / Time.fixedDeltaTime;
            springForce = springConstant * currentLength;
            damperForce = damperConstant * springVelocity;
            _rb.AddForceAtPosition(transform.up * (springForce + damperForce), transform.position);
            print(hit.collider.gameObject);
            if (hit.collider.gameObject.layer == K.LAYER_GROUND)
            {
                _isGrounded = true;
            }
            else
            {
                _isGrounded = false;
            }
        }
        else
        {
            _isGrounded = false;
        }
    }

    public bool IsGrounded()
    {
        return _isGrounded;
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        if (_vehicleScript) Gizmos.DrawLine(transform.position, transform.position + (restLenght + _vehicleScript.wheelRadius) * -Vector3.up);
    }
}
