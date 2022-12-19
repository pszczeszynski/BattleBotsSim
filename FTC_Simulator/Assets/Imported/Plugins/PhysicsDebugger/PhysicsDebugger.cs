using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[AddComponentMenu("Physics/Physics Debugger")]
public class PhysicsDebugger : MonoBehaviour {

	[Header("Line Settings")]

	[Tooltip("Time in seconds before all debug lines get destroyed.")]
	public float debugDuration = 10f;
	[Tooltip("The width of all debug lines.")]
	public float lineWidth = 0.05f;
	[Tooltip("The length of collision normals will be multiplied by this number when drawn.")]
	public float lineLengthMultiplier = 1f;
	[Tooltip("If true, debug lines for this object's collision normals will be drawn in world space.")]
	public bool drawWorldCollisionNormals = true;
	[Tooltip("If true, debug lines for this object's collision normals will be drawn in local space.")]
	public bool drawLocalCollisionNormals;
	[Tooltip("If true, debug lines will be scaled by impact force, i.e. the faster the object collides, the longer the line.")]
	public bool scaleLinesByForce = true;
	[Tooltip("If true, all debug lines will be colored based on the objects velocity in a gradient between \"slowSpeedColor\" and \"fastSpeedColor\"."
		+ " slowSpeedColor will start at slowSpeedValue and fade to fastSpeedColor as the object's velocity approaches fastSpeedValue.")]
	public bool useVelocityDebugColor = true;

	[Header("Trail Settings")]
	[Tooltip("If true, this object will leave a trail. The trail's color can be mapped to the objects velocity using the \"useVelocityDebugColor\" option.")]
	public bool useTrail = false;
	[Tooltip("If true, the trail will be colored by velocity, otherwise use default color.")]
	public bool useVelocityTrailColor = true;
	[Tooltip("Time in seconds before the trail starts to disappear.")]
	public float trailDuration = 8f;

	[Header("Trajectory Settings")]
	[Tooltip("If true, a line displaying this objected probable trajectory will be drawn.")]
	public bool drawTrajectory = false;
	[Range(0.7f, 0.9999f)]
	[Tooltip("The accuracy of the trajectory. This controls the distance between steps in calculation.")]
	public float accuracy = 0.985f;
	[Tooltip("Limit on how many steps the trajectory can take before stopping.")]
	public int iterationLimit = 150;
	[Tooltip("Stop the trajectory where the line hits an object? Objects can be set to ignore this collision by putting them on the Ignore Raycast layer.")]
	public bool stopOnCollision = true;
	[Tooltip("If true, the trajectory line will be colored by velocity, otherwise use default color. This option is separate from "
		+ "\"useVelocityDebugColor\" because it's slower to draw the trajectory line with this option enabled, so keep that in mind.")]
	public bool useVelocityForColor = false;

	[Header("Colors/Mats")]
	[Tooltip("The lower speed bound for determining velocity debug color.")]
	public float slowSpeedValue = 0f;
	[Tooltip("The upper speed bound for determining velocity debug color.")]
	public float fastSpeedValue = 15f;
	[Tooltip("The lower bound color to use for debug lines when using the \"useVelocityDebugColor\" option.")]
	public Color slowSpeedColor = new Color(75f/255, 255f/255, 75f/255);
	[Tooltip("The upper bound color to use for debug lines when using the \"useVelocityDebugColor\" option.")]
	public Color fastSpeedColor = new Color(255f/255, 75f/255, 75f/255);
	[Tooltip("The color to use for debug lines when not using the \"useVelocityDebugColor\" option.")]
	public Color defaultColor = new Color(210f/255, 70f/255, 255f/255);
	[Tooltip("Material to use for the debug line renderer. This is best left on Sprites/Default, but a custom one can be added if desired.")]
	public Material debugLineMaterial;

	[HideInInspector] public Color velociColor;
	Color lastVColor = Color.black;

	int collisionCount = 0;

	void Reset(){
		debugLineMaterial = new Material(Shader.Find("Sprites/Default"));
	}

	Rigidbody rb;
	void Start () {
		rb = GetComponent<Rigidbody>();
		lastVColor = slowSpeedColor;
		lastPos = transform.position;

		if(!rb)
			Debug.Log(gameObject.name + ": Physics Debugger does not work without a rigidBody!");
	}
	
	Vector3 lastPos = Vector3.zero;
	void LateUpdate () {
		if(rb){
			float velPct = VelocityPercentage(rb.velocity.magnitude);
			lastVColor = velociColor;
			velociColor = GradientPercent(slowSpeedColor, fastSpeedColor, velPct);

			if(velPct > 0f){
				if(useTrail)
				if(useVelocityTrailColor)
					DebugTwoLine(lastPos, transform.position, lastVColor, velociColor);
				else
					DebugTwoLine(lastPos, transform.position, defaultColor, defaultColor);

				if(drawTrajectory){
					pos = transform.position;
					vel = rb.velocity;
					trajectoryDuration = Time.unscaledDeltaTime;
					PerformPrediction();
				}
			}

			lastPos = transform.position;
		}
	}

	public Color GetVelocicolor(float inVel){
		if(rb){
			float tPct = VelocityPercentage(inVel);
			return GradientPercent(slowSpeedColor, fastSpeedColor, tPct);
		}else{
			return Color.clear;
		}
	}

	void OnCollisionEnter(Collision collision) {

		collisionCount++;

		float collisionMag = collision.relativeVelocity.magnitude;
		if(collisionMag >= slowSpeedValue){
			foreach (ContactPoint contact in collision.contacts) {
				Vector3 adjustedNormal = contact.normal;
				if(scaleLinesByForce)
					adjustedNormal *= collisionMag / 2f;
				adjustedNormal *= lineLengthMultiplier;
				if(drawWorldCollisionNormals)
					DebugLine(contact.point, adjustedNormal, defaultColor, collisionMag);
				if(drawLocalCollisionNormals)
					DebugLine(contact.point, adjustedNormal, defaultColor, collisionMag, false);
			}
		}

	}

	Transform lineParent;
	void DebugLine(Vector3 point, Vector3 normal, Color color, float force, bool world = true){
		Vector3 sPos = point;
		Vector3  ePos = point + normal;

		if(!world){
			ePos = point + -normal;
			sPos = transform.InverseTransformPoint(sPos);
			ePos = transform.InverseTransformPoint(ePos);
		}


		if(!lineParent){
			lineParent = transform.Find(transform.name + "_Lines");
			if(!lineParent){
				lineParent = new GameObject(transform.name + "_Lines").transform;
				lineParent.SetParent(transform);
				lineParent.localPosition = Vector3.zero;
				lineParent.localScale = Vector3.one;
				lineParent.localEulerAngles = Vector3.zero;
			}
		}

		GameObject lineObj = new GameObject(transform.name + "_DebugLine");
		lineObj.transform.SetParent(lineParent);
		lineObj.transform.localPosition = Vector3.zero;
     	lineObj.transform.localScale = Vector3.one;
     	lineObj.transform.localEulerAngles = Vector3.zero;

		LineRenderer debugLR = lineObj.AddComponent<LineRenderer>();
		debugLR.useWorldSpace = world;
		debugLR.receiveShadows = false;
		debugLR.sharedMaterial = debugLineMaterial;
		debugLR.startWidth = lineWidth;
		debugLR.endWidth = lineWidth;
        debugLR.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;


        if(useVelocityDebugColor) {
            Color colorGrad = GradientPercent(slowSpeedColor, fastSpeedColor, VelocityPercentage(force));
            debugLR.startColor = colorGrad;
            debugLR.endColor = colorGrad;
        } else {
            debugLR.startColor = color;
            debugLR.endColor = color;
        }

		debugLR.SetPositions(new Vector3[]{sPos, ePos});

		PhysicsDebug.DelayDestroy delayDestroy = lineObj.AddComponent<PhysicsDebug.DelayDestroy>();
		delayDestroy.delay = debugDuration;
		delayDestroy.StartUp();

	}

	void DebugTwoLine(Vector3 startPoint, Vector3 endPoint, Color sColor, Color eColor){

		if(!lineParent){
			lineParent = transform.Find(transform.name + "_Lines");
			if(!lineParent){
				lineParent = new GameObject(transform.name + "_Lines").transform;
				lineParent.SetParent(transform);
				lineParent.localPosition = Vector3.zero;
				lineParent.localScale = Vector3.one;
				lineParent.localEulerAngles = Vector3.zero;
			}
		}

		GameObject lineObj = new GameObject(transform.name + "_DebugLine");
		lineObj.transform.SetParent(lineParent);
		lineObj.transform.localPosition = Vector3.zero;
		lineObj.transform.localScale = Vector3.one;
		lineObj.transform.localEulerAngles = Vector3.zero;

		LineRenderer debugLR = lineObj.AddComponent<LineRenderer>();
		debugLR.useWorldSpace = true;
		debugLR.receiveShadows = false;
		debugLR.sharedMaterial = debugLineMaterial;
		debugLR.startWidth = lineWidth;
		debugLR.endWidth = lineWidth;
        debugLR.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
		debugLR.startColor = sColor;
		debugLR.endColor = eColor;

        debugLR.SetPositions(new Vector3[]{startPoint, endPoint});

		PhysicsDebug.DelayDestroy delayDestroy = lineObj.AddComponent<PhysicsDebug.DelayDestroy>();
		delayDestroy.delay = trailDuration;
		delayDestroy.StartUp();

	}

	public float VelocityPercentage(float force){
		float dist = fastSpeedValue - slowSpeedValue;
		float forceAdjust = force - slowSpeedValue;
		return Mathf.Clamp01(forceAdjust / dist);
	}

	public Color GradientPercent(Color lower, Color upper, float percentage){
		return new Color(lower.r + percentage * (upper.r - lower.r),
			lower.g + percentage * (upper.g - lower.g),
			lower.b + percentage * (upper.b - lower.b));		
	}

	float RangePercent(float min, float max, float percentage){
		return Mathf.Clamp(min + percentage * (max - min), min, max);
	}
		
	List<Vector3> predictionPoints = new List<Vector3>();
	Vector3 vel = Vector3.zero; Vector3 pos = Vector3.zero;
	bool twoDim = false;
	private void PerformPrediction(){
		Vector3 dir = Vector3.zero;
		Vector3 toPos;
		bool done = false;
		int iter = 0;

		float compAcc = 1f - accuracy;
		Vector3 gravAdd = Physics.gravity * compAcc;
		float dragMult = Mathf.Clamp01(1f - rb.drag * compAcc);
		predictionPoints.Clear();
		while (!done && iter < iterationLimit) {
			vel += gravAdd;
			vel *= dragMult;
			toPos = pos + vel * compAcc;
			dir = toPos - pos;
			predictionPoints.Add(pos);

			float dist = Vector3.Distance(pos, toPos);
			if(stopOnCollision){
				if(twoDim){
					RaycastHit2D hit = Physics2D.Raycast(pos, dir, dist);
					if(hit){
						if(hit.collider.transform)
						if(hit.collider.transform != transform){
							done = true;
							predictionPoints.Add(hit.point);
						}
					}
				}else{
					Ray ray = new Ray(pos, dir); 
					RaycastHit hit;
					if(Physics.Raycast(ray, out hit, dist)){
						done = true;
						predictionPoints.Add(hit.point);
					}
				}
			}

			pos = toPos;
			iter++;
		}

		if(useVelocityForColor)
			LineDebugColored(predictionPoints);
		else
			LineDebug (predictionPoints);
	}

	private GameObject trajectoryLineObj;
	float trajectoryDuration = 0.05f;
	private void LineDebug(List<Vector3> pointList){
		StopAllCoroutines();
		if(trajectoryLineObj)
			Destroy(trajectoryLineObj);
		trajectoryLineObj = new GameObject();
		trajectoryLineObj.name = "Trajectory Line";
		trajectoryLineObj.transform.SetParent(transform);
		LineRenderer debugLine = trajectoryLineObj.AddComponent<LineRenderer>();
		debugLine.startColor = defaultColor;
		debugLine.endColor = defaultColor;
        debugLine.startWidth = lineWidth;
        debugLine.endWidth = lineWidth;
        debugLine.useWorldSpace = true;
		debugLine.receiveShadows = false;
		debugLine.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
		Shader spriteDefault = Shader.Find("Sprites/Default");
		if(spriteDefault)
			debugLine.sharedMaterial = new Material(spriteDefault);
		debugLine.positionCount = pointList.Count;
		debugLine.SetPositions(pointList.ToArray());
		StartCoroutine(KillTrajectoryDelay(trajectoryDuration));
	}

	private void LineDebugColored(List<Vector3> pointList){
		if(pointList.Count < 2)
			return;
		StopAllCoroutines();
		if(trajectoryLineObj)
			Destroy(trajectoryLineObj);
		trajectoryLineObj = new GameObject();
		trajectoryLineObj.name = "Trajectory Line";
		trajectoryLineObj.transform.SetParent(transform);

		Color lastTColor = slowSpeedColor;
		Vector3 lastPos = pointList[0];
		float compAcc = 1f - accuracy;

		for(int i = 1; i < pointList.Count; i++){
			GameObject tLineObj = new GameObject();
			tLineObj.name = "Trajectory segment";
			tLineObj.transform.SetParent(trajectoryLineObj.transform);
			Vector3 curPos = pointList[i];
			float curVel = (curPos - lastPos).magnitude / compAcc;
			Color curTColor = GetVelocicolor(curVel);

			LineRenderer debugLine = tLineObj.AddComponent<LineRenderer>();
			debugLine.startColor = lastTColor;
			debugLine.endColor = curTColor;

            debugLine.startWidth = lineWidth;
            debugLine.endWidth = lineWidth;
            debugLine.useWorldSpace = true;
			debugLine.receiveShadows = false;
			debugLine.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
			Shader spriteDefault = Shader.Find("Sprites/Default");
			if(spriteDefault)
				debugLine.sharedMaterial = new Material(spriteDefault);
			debugLine.positionCount = 2;
			debugLine.SetPosition(0, lastPos);
			debugLine.SetPosition(1, curPos);

			lastPos = curPos;
			lastTColor = curTColor;
		}
		StartCoroutine(KillTrajectoryDelay(trajectoryDuration));
	}

	private IEnumerator KillTrajectoryDelay(float delay){
		yield return new WaitForSeconds(delay);
		if(trajectoryLineObj)
			Destroy(trajectoryLineObj);
	}

	private void OnDestroy(){
		if(trajectoryLineObj){
			if(trajectoryLineObj)
				Destroy(trajectoryLineObj);
			if(lineParent)
				Destroy(lineParent);
			Resources.UnloadUnusedAssets();
		}
	}

	//endClass
}
