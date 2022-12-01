using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DriveTrainCalcs : MonoBehaviour
{

    public Transform T_Drivetrain;

    private void Start()
    {
        UpdateAllCalcs();
    }

    // OnValue Changed functions for menu changes
    public void UpdateAllCalcs()
    {
        // Get all the values
        // Check for sane values and correct if insane


        // Unfortunetally if the DriveTrain hasn't been enabled, we won't be able to find some of these panels
        if (T_Drivetrain.Find("MotorType/Dropdown") == null) { return; }

        int motortypeindex = T_Drivetrain.Find("MotorType/Dropdown").GetComponent<Dropdown>().value;
        string motortypename = T_Drivetrain.Find("MotorType/Dropdown").GetComponent<Dropdown>().options[motortypeindex].text;

        float gear_ratio = 20f;
        if (!float.TryParse(T_Drivetrain.Find("GearRatio").GetComponent<InputField>().text, out gear_ratio))
        {
            gear_ratio = -1f;
        };

        float wheel_diameter = 4f;
        if (!float.TryParse(T_Drivetrain.Find("wheelDiameter").GetComponent<InputField>().text, out wheel_diameter))
        {
            wheel_diameter = -1f;
        };

        float weight = 42f;
        if (!float.TryParse(T_Drivetrain.Find("Weight").GetComponent<InputField>().text, out weight))
        {
            weight = -1f;
        }

        // Sanity check these
        if (gear_ratio < 5f)
        {
            T_Drivetrain.Find("GearRatio").GetComponent<InputField>().text = "5";
            gear_ratio = 5f;
        }

        if (gear_ratio > 999f)
        {
            T_Drivetrain.Find("GearRatio").GetComponent<InputField>().text = "999";
            gear_ratio = 999f;
        }

        if (wheel_diameter < 1f)
        {
            T_Drivetrain.Find("wheelDiameter").GetComponent<InputField>().text = "1";
            wheel_diameter = 15f;
        }

        if (wheel_diameter > 9f)
        {
            T_Drivetrain.Find("wheelDiameter").GetComponent<InputField>().text = "9";
            wheel_diameter = 9f;
        }

        if (weight < 15f)
        {
            T_Drivetrain.Find("Weight").GetComponent<InputField>().text = "15";
            weight = 15f;
        }

        if (weight > 42f)
        {
            T_Drivetrain.Find("Weight").GetComponent<InputField>().text = "42";
            weight = 42f;
        }

        // Now Calculate full-speed and acceleration

        // Calculate free speed (ft/s)
        float acc;
        float speed;

        CalcDriveTrain(wheel_diameter, gear_ratio, motortypeindex, weight, out speed, out acc);

        // Save values in globals
        GLOBALS.speed = speed;
        GLOBALS.acceleration = acc;

        speed *= acc/(acc+GLOBALS.friction);  // Reduce before displaying full speed to correct for friction losses


        // Output value to user
        T_Drivetrain.Find("Speed").GetComponent<InputField>().text = speed.ToString("0.#");
        T_Drivetrain.Find("Acceleration").GetComponent<InputField>().text = acc.ToString("0.#");

        // Generate Speed curve
        List<float> xvalues = new List<float>();
        List<float> yvalues = new List<float>();
        xvalues.Add(0f);
        yvalues.Add(0f);

        for ( float time = 0; time <= 2.1f; time += 0.05f)
         {
             xvalues.Add(time);
            float distance = speed * ((-1f + Mathf.Exp(-acc * time / speed)) * speed / acc + time);
             yvalues.Add(distance);
         }

        /* Iterative method below used before figured out closed equation
         float curr_speed = 0;
        float old_speed = 0;
        float curr_acc = acc;
        float distance = 0;
        float tdelta = 0.001f;
        int counter = 0;
        for ( float time = tdelta; time <=3.1f; time += tdelta)
        {
            old_speed = curr_speed;
            curr_speed += tdelta * curr_acc;
            curr_acc = acc * (speed - curr_speed) / speed;
            distance += (curr_speed + old_speed)/2f* tdelta;

            counter += 1;
            if( counter >= 0.2f/tdelta)
            {
                xvalues.Add(time);
                yvalues.Add(distance);
                counter = 0;
            }

        }
        */

        // Update graph
        T_Drivetrain.Find("Graph").GetComponent<Graph>().UpdateGraph(xvalues, yvalues, 1.8f, 6f);
    }

    public static void CalcDriveTrain(float wheel_diameter, float gear_ratio, int motortypeindex, float weight, out float speed, out float acc )
    {
        float free_rpm = 5475f; // rotations per minute
        float stall_torque = 0.173f; // N*m

        // Lookup values for the given motor
        switch (motortypeindex)
        {
            // case 1:
            //     // NeveRest spec
            //     free_rpm = 6600f;
            //     stall_torque = 0.064f;
            //     break;
            case 1: // Tetrix MAX
                free_rpm = 6000f;
                stall_torque = 0.1125f;
                break;
            case 2: // GoBilda Yellow Jacket
                free_rpm = 6021f;
                stall_torque = 0.1407f;
                break;
            case 3: // Rev HD
                free_rpm = 6000f;
                stall_torque = 0.105f;
                break;
            case 4: // V5 Smart Motor
                free_rpm = 4800f; 
                stall_torque = 0.178f;
                break;
            default: // Case 0 covered here
                     // NeveRest Measured
                free_rpm = 7200f;
                stall_torque = 0.208f;
                break;
        }

        // Calculate free speed (ft/s)
        speed = wheel_diameter / 12f * Mathf.PI * free_rpm / 60f / gear_ratio;

        // Calculate acceleration
        // Stall force = Newtons, acc = ft/s2
        float stall_force = stall_torque * gear_ratio / (wheel_diameter * Mathf.PI / 39.3701f) * GLOBALS.motor_count;
        acc = stall_force / weight * 2.20462f * 3.28083f; // 2.2 converts lbs to kg, 3.28 converts m/s^2 to ft/s^2
    }
}
