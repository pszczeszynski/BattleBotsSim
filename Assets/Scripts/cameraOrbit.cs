using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class cameraOrbit : MonoBehaviour {
    public new bool enabled = true;
	public Transform orbitAround;
    public float speed = 10.0f;
	public float aspect_ratio = 0f;

	// Use this for initialization
	void Start () {
		if( aspect_ratio > 0f)
		{
			Camera mycamera = GetComponent<Camera>();
			if( mycamera == null ) { return; }

			mycamera.aspect = aspect_ratio;
		}
	}
	
	// Update is called once per frame
	float elapsedTime =0;
    void Update() {
        if (!enabled) { return; } 
		float deltaTime = Time.deltaTime;
		elapsedTime += Time.deltaTime;
		transform.RotateAround (orbitAround.position, Vector3.up, speed *  deltaTime);
	}
}
