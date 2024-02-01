using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Obi;

[RequireComponent(typeof(ObiSolver))]
public class AttachmentHandler : MonoBehaviour {

 	ObiSolver solver;

	Obi.ObiSolver.ObiCollisionEventArgs collisionEvent;

    ObiCollider target_collider;

    // public Transform target;
    private ObiParticleAttachment attachment;

    // public GameObject table;

	void Awake(){
        // table = GameObject.Find("Table");

		solver = GetComponent<Obi.ObiSolver>();
        attachment = solver.GetComponentInChildren<ObiParticleAttachment>();
		target_collider = GameObject.Find("Test Cube").GetComponent<ObiCollider>();
        // target_collider = target.gameObject.GetComponent<ObiCollider>();
		solver.OnCollision += Solver_OnCollision;
	}

	void Solver_OnCollision (object sender, Obi.ObiSolver.ObiCollisionEventArgs e)
	{
		var world = ObiColliderWorld.GetInstance();
		foreach (Oni.Contact contact in e.contacts)
		{
			// this one is an actual collision:
			if (contact.distance < 0.01)
			{
				ObiColliderBase collider = world.colliderHandles[contact.bodyB].owner;
				// if (collider != null && collider != table.GetComponent<ObiCollider>())
				// {
				// 	Debug.Log("Collision with " + collider.gameObject.name);
				// }
				if (collider == target_collider) {
                    // Debug.Log("Found It!!!!!!!!!!!!");
                    attachment.enabled = true;
                }
                // else {
                //     attachment.enabled = false;
                // }

			}
		}
	}

}
	