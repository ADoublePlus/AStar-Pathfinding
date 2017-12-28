using UnityEngine;

namespace AStarPathfinding
{
    public class Player : MonoBehaviour
    {
        public float movementSpeed = 20f;
        public float maxSpeed = 5f;

        private Rigidbody rigid;

        // Use this for initialization
        void Start ()
        {
            rigid = GetComponent<Rigidbody>();
        }

        // Update is called once per frame
        void Update ()
        {
            Movement();
        }

        void Movement ()
        {

            float inputHoriz = Input.GetAxis("Horizontal");
            float inputVert = Input.GetAxis("Vertical");

            Vector3 force = new Vector3(inputHoriz, 0, inputVert);

            if (rigid.velocity.magnitude < maxSpeed)
            {
                rigid.AddForce(force * movementSpeed);
            }

        }
    }
}