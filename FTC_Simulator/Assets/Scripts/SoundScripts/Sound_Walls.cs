using System;
using UnityEngine;
using UnityEngine.Events;

public class Sound_Walls : MonoBehaviour
{
    float lastTime = 0;
    public float volume = 0.25f;
    public float clipped_volume = 0.6f;
    public float min_speed = 0.5f;
    public float blanking_time = 0.2f; //seconds to ignore more hits
    AudioManager myaudiomanager = null;

    private void Start()
    {
        myaudiomanager = transform.GetComponent<AudioManager>();
    }

    private float time_of_update = 0;

    public void OnCollisionEnter(Collision collision) {
        float relativeVelocity = collision.relativeVelocity.magnitude;

        if ((collision.relativeVelocity.magnitude > min_speed) &&
            ((Time.time - time_of_update) > blanking_time))
        {
            Sound hitSound = myaudiomanager.findSound("hit1");

            float final_volume = (collision.relativeVelocity.magnitude- min_speed) * volume;
            if (final_volume > clipped_volume) { final_volume = clipped_volume; }

            // Clip volume
            myaudiomanager.Play(hitSound.name, 0, final_volume);
            time_of_update = Time.time;
        }
    }
}
