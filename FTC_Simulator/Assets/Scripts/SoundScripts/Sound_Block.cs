using UnityEngine;


public class Sound_Block : MonoBehaviour
{
    public float volume = 0.3f;

    float lastTime = 0;
    void OnCollisionEnter(Collision collision) {
        float relativeVelocity = collision.relativeVelocity.magnitude;
        // Debug.Log($"Relative Velocity: {relativeVelocity}");

        bool didPlay = true;
        if(Time.time - lastTime < 0.2) return;
        if(relativeVelocity > 2) {
            transform.GetComponent<AudioManager>().Play("hit4");
            transform.GetComponent<AudioManager>().findSound("hit4")._volume = relativeVelocity * volume;
        } else if(relativeVelocity > 1.6) {
            transform.GetComponent<AudioManager>().Play("hit3");
            transform.GetComponent<AudioManager>().findSound("hit3")._volume = relativeVelocity * volume;

        } else if(relativeVelocity > 1.2) {
            transform.GetComponent<AudioManager>().Play("hit2");
            transform.GetComponent<AudioManager>().findSound("hit2")._volume = relativeVelocity * volume;
        } else if(relativeVelocity > 0.8) {
            transform.GetComponent<AudioManager>().Play("hit1");
            transform.GetComponent<AudioManager>().findSound("hit1")._volume = relativeVelocity * volume;
        } else {
            didPlay = false;
        }

        if(didPlay) {
            lastTime = Time.time;
        }
    }
}
