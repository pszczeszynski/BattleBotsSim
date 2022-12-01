using UnityEngine;
using System.Collections;

namespace PhysicsDebug {
	public class DelayDestroy : MonoBehaviour {
	
		public float delay = 7.5f;

		public void StartUp () {
			StartCoroutine(DestroyCo());
		}
	
		IEnumerator DestroyCo(){
			yield return new WaitForSeconds(delay);
			Destroy(gameObject);
		}
	}
}