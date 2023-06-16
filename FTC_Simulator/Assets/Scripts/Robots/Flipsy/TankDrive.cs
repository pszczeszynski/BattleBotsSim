using UnityEngine;

public class TankDrive : MonoBehaviour
{
    public HingeJoint wheelTL;
    public HingeJoint wheelTR;
    public HingeJoint wheelBL;
    public HingeJoint wheelBR;
    private RobotInterface3D _robotInterface3D;
    private Vector3 _resetPosition;
    private Quaternion _resetRotation;
    public Transform body;
    public float SPEED = 4000;

    void Start()
    {
        _robotInterface3D = GetComponent<RobotInterface3D>();
    }


    void FixedUpdate()
    {
        UnityEngine.Debug.Log("FixedUpdate");
        float movement_amount = _robotInterface3D.gamepad1_left_stick_y;
        float turn_amount = _robotInterface3D.gamepad1_right_stick_x;
        UnityEngine.Debug.Log("Movement amount: " + movement_amount);

        // apply movement
        float left_wheel_torque = movement_amount;
        float right_wheel_torque = movement_amount;
        // apply turning
        left_wheel_torque -= turn_amount;
        right_wheel_torque += turn_amount;

        float reduce_amount = Mathf.Max(1, Mathf.Abs(left_wheel_torque), Mathf.Abs(right_wheel_torque));

        right_wheel_torque /= reduce_amount;
        left_wheel_torque /= reduce_amount;

        // apply torques
        ApplyWheelTorque(wheelTL, left_wheel_torque);
        ApplyWheelTorque(wheelTR, right_wheel_torque);
        ApplyWheelTorque(wheelBL, left_wheel_torque);
        ApplyWheelTorque(wheelBR, right_wheel_torque);
    }

    void ApplyWheelTorque(HingeJoint wheel, float torque)
    {
        JointMotor motor = wheel.motor;
        motor.targetVelocity = (float)torque * SPEED;
        wheel.motor = motor;
    }
    
    void Update()
    {
        // set reset if R
        if (Input.GetKeyDown(KeyCode.R))
        {
            _resetPosition = body.position;
            _resetRotation = body.rotation;
        }

        // go to reset if space
        if (Input.GetKeyDown(KeyCode.Space) && _resetPosition != null)
        {
            body.position = _resetPosition + new Vector3(0, 0.35f, 0);
            body.rotation = _resetRotation;

            // reset velocity
            Rigidbody rb = body.GetComponent<Rigidbody>();
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }
}