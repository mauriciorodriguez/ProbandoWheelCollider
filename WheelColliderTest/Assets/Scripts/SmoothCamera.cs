using UnityEngine;
using System.Collections;

public class SmoothCamera : MonoBehaviour
{
    public Transform target;
    public float distance, height, heightDamping, rotationDamping;

    private void LateUpdate()
    {
        if (!target) return;
        /*var wantedRotationAngle = target.eulerAngles.y;
        var wantedHeight = target.position.y * height;
        var currentRotationAngle = transform.eulerAngles.y;
        var currentHeight = transform.position.y;
        currentRotationAngle = Mathf.LerpAngle(currentRotationAngle,wantedRotationAngle,rotationDamping*Time.deltaTime);
        currentHeight = Mathf.Lerp(currentHeight,wantedHeight,heightDamping*Time.deltaTime);
        var currentRotation = Quaternion.Euler(0,currentRotationAngle,0);
        transform.position = target.position;
        transform.position -= currentRotation * Vector3.forward * distance;
        transform.position = Vector3.up * currentHeight;
        transform.LookAt(target);*/

        var forward = target.TransformDirection(Vector3.forward);
        // Get the target position
        var targetPosition = target.position;

        // Place the camera distance behind the target
        transform.position = targetPosition - forward * distance;
        // And move the camera a bit up
        transform.position += Vector3.up * height;
        // Always look at the target
        transform.LookAt(target);
    }
}
