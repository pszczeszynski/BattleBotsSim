using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RigidBodyInit : MonoBehaviour
{
    public float max_angular_velocity = 0f;
    public float max_depenetration_velocity = 0f;
    public int solver_iterations = 0;
    public int solver_velocity_iterations = 0;
    public float sleep_threshold = 0f;
    public Vector3 center_of_mass = new Vector3(0, 0, 0);
    public Vector3 inertia_tensor = new Vector3(0, 0, 0);
    public float scale_inertia = 1f; // Increase/Decreases moment of innertia
    public Vector3 final_center_of_mass = new Vector3(0, 0, 0);
    public Vector3 final_inertia_tensor = new Vector3(0, 0, 0);
    public bool comWasChanged = false;
    public bool itWasChanged = false;

    bool initialized = false;

    // Do this on initialization so that this takes precedence over any start function
    void Update()
    {
        if (!initialized)
        {
            initialized = true;
            Rigidbody myrb = GetComponent<Rigidbody>();

            if (myrb == null) { return; }

            // Set max angular rotation if set
            if (max_angular_velocity != 0)
            {
                myrb.maxAngularVelocity = max_angular_velocity;
            }

            // Set max velocity when bouncing out of something
            if (max_depenetration_velocity != 0)
            {
                myrb.maxDepenetrationVelocity = max_depenetration_velocity;
            }

            if (solver_iterations != 0)
            {
                myrb.solverIterations = solver_iterations;
            }

            if (solver_velocity_iterations != 0)
            {
                myrb.solverVelocityIterations = solver_velocity_iterations;
            }

            if (sleep_threshold != 0)
            {
                myrb.sleepThreshold = sleep_threshold;
            }

            if (center_of_mass != Vector3.zero)
            {
                myrb.centerOfMass = center_of_mass;
                comWasChanged = true;

            }

            if (inertia_tensor != Vector3.zero)
            {
                myrb.inertiaTensor = inertia_tensor;
                itWasChanged = true;
            }

            if ((scale_inertia != 0f) && (scale_inertia != 1f))
            {
                Vector3 moment_of_inertia = myrb.inertiaTensor;
                moment_of_inertia.Scale(new Vector3(scale_inertia, scale_inertia, scale_inertia));
                myrb.inertiaTensor = moment_of_inertia;
            }

            final_center_of_mass = myrb.centerOfMass;
            final_inertia_tensor = myrb.inertiaTensor;
        }
    }


}
