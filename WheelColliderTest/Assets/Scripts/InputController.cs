using UnityEngine;
using System.Collections;

public class InputController : MonoBehaviour
{
    private Vehicle _vehicleReference;
    private float _accel, _brake, _handbrake, _steer;

    private void Start()
    {
        _vehicleReference = GetComponent<Vehicle>();
    }

    private void FixedUpdate()
    {
         _accel = Mathf.Clamp(Input.GetAxis(K.INPUT_VERTICAL),0,1);
         _brake = Mathf.Clamp(Input.GetAxis(K.INPUT_VERTICAL),-1,0);
         _steer = Input.GetAxis(K.INPUT_HORIZONTAL);
         _handbrake = Input.GetAxis(K.INPUT_SPACEBAR);
    }
}
