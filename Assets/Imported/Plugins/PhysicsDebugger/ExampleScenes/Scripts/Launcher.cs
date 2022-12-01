using UnityEngine;
using System.Collections;


namespace PhysicsDebug {
	public class Launcher : MonoBehaviour {
	
		public GameObject[] objsToLaunch;
		public Vector2 forceRange = new Vector2(100,200);
			
		void Start(){
			Time.fixedDeltaTime = 0.01f;
		}

		// Update is called once per frame
		void Update () {
			if(Input.GetKeyDown(KeyCode.T))
				Launch();
			//if(Input.GetKey(KeyCode.Y))
			//	Launch();
		}
	
		GameObject launchObjParent;
		int launchIndex = 0;
		void Launch(){
			if(!launchObjParent){
				launchObjParent = new GameObject();
				launchObjParent.name = "Launched Objects";
			}
			GameObject lInst = Instantiate (objsToLaunch[launchIndex]);
			launchIndex++;
			if(launchIndex >= objsToLaunch.Length)
				launchIndex = 0;
			lInst.transform.SetParent(launchObjParent.transform);
			Rigidbody rbi = lInst.GetComponent<Rigidbody> ();
			lInst.transform.position = transform.Find ("LP").position;
			lInst.transform.rotation = transform.Find ("LP").rotation;
			rbi.AddRelativeForce(new Vector3(Random.Range(0.0f, 10.0f), Random.Range(0.0f, 10.0f),Random.Range(forceRange.x, forceRange.y)));
	
			Renderer tR = lInst.GetComponent<Renderer>();
			tR.material = Instantiate(tR.material) as Material;
			tR.material.color = RandomColor();
	
			lInst.GetComponent<PhysicsDebug.DelayDestroy>().StartUp();
		}
	
		Color RandomColor(){
			float r = Random.Range (0.0f, 1.0f);
			float g = Random.Range (0.0f, 1.0f);
			float b = Random.Range (0.0f, 1.0f);
			return new Color(r,g,b);
		}
	
	}
}