using UnityEngine;
using System.Collections;
[System.Serializable]
public class WheelPair
{   
    public GameObject leftWheel, rightWheel;
    public bool hasContactSurface { get; private set; }

    
}
