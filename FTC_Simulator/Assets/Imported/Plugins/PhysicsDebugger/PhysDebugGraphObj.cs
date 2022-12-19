using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

[AddComponentMenu("Physics/Physics Debug Graph Obj")]
public class PhysDebugGraphObj : MonoBehaviour {

	[HideInInspector] public GameObject gO;
	[HideInInspector] public Rigidbody rb;
	[HideInInspector] new public string name = "";

	[HideInInspector] public List<float> velocityGraph = new List<float>();
	[HideInInspector] public List<float> accelerationGraph = new List<float>();
	[HideInInspector] public List<Vector3> velocityAxisGraph = new List<Vector3>();
	[HideInInspector] public List<Vector3> gGraph = new List<Vector3>();

	[HideInInspector] public Vector3 lastVelocity = Vector3.zero;

	[HideInInspector] public int graphResolution = 100;
	[HideInInspector] public int dataRate = 60;


	public void Initialize () {	
		gO = gameObject;
		rb = gO.GetComponent<Rigidbody>();
		name = gO.name;
	}

	int frameCounter = 0;
	void FixedUpdate () {
		colliding = false;
		triggering = false;
		frameCounter++;
		if(frameCounter % (61 - dataRate) == 0){
			frameCounter = 0;
			getData();
		}
	}



	[HideInInspector] public int collisionCount = 0;
	[HideInInspector] public bool colliding = false;
	[HideInInspector] public string collidingObject = "";
	[HideInInspector] public float collisionForce = 0f;
	[HideInInspector] public float minCollisionForce = Mathf.Infinity;
	[HideInInspector] public float maxCollisionForce = 0f;
	[HideInInspector] public Vector3 collisionVelocity = Vector3.zero;
	[HideInInspector] public float collisionDuration = 0f;
	[HideInInspector] public float noCollisionDuration = 0f;
	float lastCollideTime = 0f;
	float lastNoCollideTime = 0f;
	void OnCollisionEnter(Collision collisionInfo){
		collisionCount++;
		collisionForce = collisionInfo.relativeVelocity.magnitude;
		collisionVelocity = collisionInfo.relativeVelocity;
		collidingObject = collisionInfo.gameObject.name;
		lastCollideTime = Time.time;
		lastNoCollideTime = Time.time;

		if(collisionForce < minCollisionForce)
			minCollisionForce = collisionForce;

		if(collisionForce > maxCollisionForce)
			maxCollisionForce = collisionForce;
	}

	void OnCollisionStay(Collision collisionInfo){
		colliding = true;
		collidingObject = collisionInfo.gameObject.name;
		collisionDuration = Time.time - lastCollideTime;
		lastNoCollideTime = Time.time;
	}

	void OnCollisionExit(Collision collisionInfo){
		collisionDuration = Time.time - lastCollideTime;
		lastNoCollideTime = Time.time;
	}


	[HideInInspector] public int triggerCount = 0;
	[HideInInspector] public bool triggering = false;
	[HideInInspector] public string triggeringObject = "";
	[HideInInspector] public float triggerDuration = 0f;
	float lastTriggerTime = 0f;
	void OnTriggerEnter(Collider other) {
		triggerCount++;
		triggeringObject = other.name;
		lastTriggerTime = Time.time;
	}

	void OnTriggerStay(Collider other) {
		triggering = true;
		triggeringObject = other.name;
		triggerDuration = Time.time - lastTriggerTime;
	}

	void OnTriggerExit(Collider other) {
		triggerDuration = Time.time - lastTriggerTime;
	}

	[HideInInspector] public float minSpeed = Mathf.Infinity;
	[HideInInspector] public float maxSpeed = 0f;
	[HideInInspector] public Vector3 currentVelocity = Vector3.zero;
	[HideInInspector] public Vector3 currentAcceleration = Vector3.zero;
	[HideInInspector] public Vector3 minVelocity = Vector3.zero;
	[HideInInspector] public Vector3 maxVelocity = Vector3.zero;
	public void getData(){
		if(rb){
			currentVelocity = rb.velocity;
			currentAcceleration = lastVelocity - currentVelocity;
			if(accelerationGraph.Count > graphResolution)
				accelerationGraph.RemoveRange(0, accelerationGraph.Count - graphResolution);
			
			if(velocityGraph.Count > 0)
				accelerationGraph.Add(velocityGraph.Last() - currentVelocity.magnitude);
			else
				accelerationGraph.Add(0f);
			
			if(velocityGraph.Count > graphResolution)
				velocityGraph.RemoveRange(0, velocityGraph.Count - graphResolution);
			velocityGraph.Add(currentVelocity.magnitude);

			if(velocityAxisGraph.Count > graphResolution)
				velocityAxisGraph.RemoveRange(0, velocityAxisGraph.Count - graphResolution);
			velocityAxisGraph.Add(currentVelocity);

			if(gGraph.Count > graphResolution)
				gGraph.RemoveRange(0, gGraph.Count - graphResolution);			
			gGraph.Add(currentAcceleration);

			if(currentVelocity.magnitude < minSpeed)
				minSpeed = currentVelocity.magnitude;

			if(currentVelocity.magnitude > maxSpeed)
				maxSpeed = currentVelocity.magnitude;

			if(currentVelocity.x < minVelocity.x)
				minVelocity.x = currentVelocity.x;
			if(currentVelocity.y < minVelocity.y)
				minVelocity.y = currentVelocity.y;
			if(currentVelocity.z < minVelocity.z)
				minVelocity.z = currentVelocity.z;

			if(currentVelocity.x > maxVelocity.x)
				maxVelocity.x = currentVelocity.x;
			if(currentVelocity.y > maxVelocity.y)
				maxVelocity.y = currentVelocity.y;
			if(currentVelocity.z > maxVelocity.z)
				maxVelocity.z = currentVelocity.z;

			noCollisionDuration = Time.time - lastNoCollideTime;

			lastVelocity = currentVelocity;
		}
	}		

}
