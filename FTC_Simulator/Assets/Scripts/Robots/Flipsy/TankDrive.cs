using UnityEngine;

public class TankDrive : MonoBehaviour
{
    public HingeJoint wheelTL;
    public HingeJoint wheelTR;
    public HingeJoint wheelBL;
    public HingeJoint wheelBR;
    public Transform body;
    public float SPEED = 4000;
    private float movement_amount;
    private float turn_amount;

    // Call this to move the robot
    public void Drive(float movement_amount, float turn_amount)
    {
        this.movement_amount = -movement_amount;
        this.turn_amount = -turn_amount;
    }


    void FixedUpdate()
    {
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
}