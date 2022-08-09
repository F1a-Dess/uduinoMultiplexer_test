using UnityEngine;
using Uduino;

public class ArduinoMPUTest2 : MonoBehaviour {

    Vector3 position;
    Vector3 rotation;
    public Vector3 rotationOffset;
    public float speedFactor = 15.0f;
    public string imuName; // You should ignore this if there is one IMU.
    // public bool setOrigin = false;
    // Quaternion rotOffset = Quaternion.identity;


    void Start () {
    //    UduinoManager.Instance.OnDataReceived += ReadIMU0;
    //    UduinoManager.Instance.OnDataReceived += ReadIMU1;
    //    UduinoManager.Instance.OnDataReceived += ReadIMU2;
      //  Note that here, we don't use the delegate but the Events, assigned in the Inpsector Panel
    }

    void Update() { 

        // if(setOrigin) {

        //     rotOffset = this.transform.rotation;
        //     setOrigin = false;
        // }
    }

    public void ReadIMU0 (string data, UduinoDevice device) {
        //Debug.Log(data);
        string[] values = data.Split('/');
        if (values.Length == 5 && values[0] == imuName) // Rotation of the first one 
        {
            float w1 = float.Parse(values[1]);
            float x1 = float.Parse(values[2]);
            float y1 = float.Parse(values[3]);
            float z1 = float.Parse(values[4]);
            this.transform.localRotation = Quaternion.Lerp(this.transform.localRotation,  new Quaternion(w1, y1, x1, z1), Time.deltaTime * speedFactor);
        } else if (values.Length != 5)
        {
            Debug.LogWarning(data);
        }
        this.transform.parent.transform.eulerAngles = rotationOffset;
      //  Log.Debug("The new rotation is : " + transform.Find("IMU_Object").eulerAngles);

        // this.transform.rotation = new Quaternion(w1, x1, y1, z1) * Quaternion.Inverse(rotOffset);
    }
    
    public void ReadIMU1 (string data, UduinoDevice device) {
        //Debug.Log(data);
        string[] values = data.Split('/');
        if (values.Length == 5 && values[0] == imuName) // Rotation of the first one 
        {
            float w2 = float.Parse(values[1]);
            float x2 = float.Parse(values[2]);
            float y2 = float.Parse(values[3]);
            float z2 = float.Parse(values[4]);
            this.transform.localRotation = Quaternion.Lerp(this.transform.localRotation,  new Quaternion(w2, y2, x2, z2), Time.deltaTime * speedFactor);
        } else if (values.Length != 5)
        {
            Debug.LogWarning(data);
        }
        this.transform.parent.transform.eulerAngles = rotationOffset;
      //  Log.Debug("The new rotation is : " + transform.Find("IMU_Object").eulerAngles);

        // this.transform.rotation = new Quaternion(w2, x2, y2, z2) * Quaternion.Inverse(rotOffset);
    }
    
    public void ReadIMU2 (string data, UduinoDevice device) {
        //Debug.Log(data);
        string[] values = data.Split('/');
        if (values.Length == 5 && values[0] == imuName) // Rotation of the first one 
        {
            float w3 = float.Parse(values[1]);
            float x3 = float.Parse(values[2]);
            float y3 = float.Parse(values[3]);
            float z3 = float.Parse(values[4]);
            this.transform.localRotation = Quaternion.Lerp(this.transform.localRotation,  new Quaternion(w3, y3, x3, z3), Time.deltaTime * speedFactor);
        } else if (values.Length != 5)
        {
            Debug.LogWarning(data);
        }
        this.transform.parent.transform.eulerAngles = rotationOffset;
      //  Log.Debug("The new rotation is : " + transform.Find("IMU_Object").eulerAngles);
      
        // this.transform.rotation = new Quaternion(w3, x3, y3, z3) * Quaternion.Inverse(rotOffset);
    }
}
