using UnityEngine;

public class Spinner : MonoBehaviour
{
    [SerializeField] private float forceMagnitude = 1000f;

    void OnCollisionEnter(Collision collision)
    {
        UnityEngine.Debug.Log("collision!");
        Rigidbody collidedRigidbody = collision.gameObject.GetComponent<Rigidbody>();
        if (collidedRigidbody != null)
        {
            UnityEngine.Debug.Log("nonnull!");
            Vector3 collisionPoint = collision.GetContact(0).point;
            Vector3 upwardForce = transform.up * forceMagnitude;
            Vector3 torque = Vector3.Cross(transform.position - collisionPoint, upwardForce);
            collidedRigidbody.AddForceAtPosition(upwardForce, collisionPoint, ForceMode.Impulse);
            collidedRigidbody.AddTorque(torque, ForceMode.Impulse);
        }
    }
}