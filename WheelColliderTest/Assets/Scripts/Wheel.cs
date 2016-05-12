using UnityEngine;
using System.Collections;

public class Wheel : MonoBehaviour
{
    private bool _hasContact;

    private void OnCollisionEnter(Collision other)
    {
        _hasContact = true;
    }

    private void OnCollisionExit(Collision other)
    {
        _hasContact = false;
    }

    public bool HasContactWithSurface()
    {
        return _hasContact;
    }
}
