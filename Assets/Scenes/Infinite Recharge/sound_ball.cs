using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class sound_ball : MonoBehaviour
{
    private bool isInRobot = false;
    public float clipped_volume = 0.6f;
    public float min_speed = 0.5f;
    public float blanking_time = 0.2f; //seconds to ignore more hits
    public float inside_robot_scale = 0.4f;
    public float sound_scale = 1f;

    private void OnTriggerEnter(Collider other) {
        if(other.name.StartsWith("collisionBoundry")) {
            isInRobot = true;
        }
    }
    private void OnTriggerExit(Collider other) {
        if(other.name.StartsWith("collisionBoundry")) {
            isInRobot = false;
        }
    }

    private void Start()
    {
        // Get the sound manager if not already found
        if (!sm_cached) { sm_cached = GetComponent<AudioManager>(); }
    }

    private AudioManager sm_cached = null;

    private float time_of_update = 0;

    private void OnCollisionEnter(Collision other) {


        if((other.relativeVelocity.magnitude > min_speed) &&
            ((Time.time - time_of_update) > blanking_time) ) {

            float robotScalar = isInRobot ? inside_robot_scale : 1f;
            Sound hitSound = sm_cached.findSound("ballhit2");

            if(hitSound.isPlaying) {
                hitSound = sm_cached.findSound("ballhit3");
            }

            float volume  = other.relativeVelocity.magnitude * hitSound.get_init_volume() * robotScalar * sound_scale;
            if ( volume > clipped_volume) { volume = clipped_volume;  }

            // Clip volume
            sm_cached.Play(hitSound.name,0, volume);
            time_of_update = Time.time;
        }
    }
}
