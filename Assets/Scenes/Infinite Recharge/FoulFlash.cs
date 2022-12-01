using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FoulFlash : MonoBehaviour
{
    public long start_time;
    public float rot_speed = 720f;
    public long duration = 500;
    public float y_add = 2f;
    public float y_initial;

    // Start is called before the first frame update
    void Start()
    {
        start_time = MyUtils.GetTimeMillis();

        // Record initial y position
        y_initial = transform.position.y;
    }

    // Update is called once per frame
    void Update()
    {
        // Get the current time
        long curr_time = MyUtils.GetTimeMillis();
        long delta_time = (curr_time - start_time);

        if( delta_time < duration)
        {
            // Make it grow up and then shrink down
            // Calculate new size scaler
            float mid_scale = 2f * delta_time / duration - 1f;
            float scale_y = 1f - Mathf.Abs(mid_scale);

            // Calculate new postion
            Vector3 new_pos = transform.position;
            new_pos.y = y_initial + 0.5f*(mid_scale+1f)* y_add;
            transform.position = new_pos;

            // Assign new scale
            Vector3 new_scale = transform.localScale;
            new_scale.y = scale_y;
            transform.localScale = new_scale;

            // Apply rotation
            Vector3 new_rotation = transform.localRotation.eulerAngles;
            new_rotation.y = rot_speed * delta_time/1000;
            transform.localRotation = Quaternion.Euler(new_rotation);
        }
        else
        {
            gameObject.SetActive(false);
            Destroy(this.gameObject);
        }
    }
}
