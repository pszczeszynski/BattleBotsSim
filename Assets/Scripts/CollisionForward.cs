
using UnityEngine;
/// <summary>
/// Forwards collision events to it's parent
/// </summary>
public class CollisionForward : MonoBehaviour
{
    void OnCollisionEnter(Collision collision) {
        transform.parent.GetComponent<Sound_Walls>().OnCollisionEnter(collision);
    }
}
