using UnityEngine;
using System.Collections;

[System.Serializable]
public class WheelPairs
{
    public WheelCollider leftWheelCollider, rightWheelCollider;
    public Transform leftWheelModel, rightWheelModel;
    public bool motor, steer;
}
