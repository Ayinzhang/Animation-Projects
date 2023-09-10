using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class block_motion : MonoBehaviour
{
	bool pressed = false;
	public bool cube_move = false;
	Vector3 offset;

	Mesh mesh;
	Collider collider;

	GameObject wave;

	float h = 0.01f;
	float mass = 1.0f;
	float g = -9.8f;
	Vector3 v = Vector3.zero;
	Vector3 w = Vector3.zero;

	Vector3 force;
	Vector3 flotage;
	Vector3 torque;
	Matrix4x4 inertia = new Matrix4x4();

	Quaternion QuatAdd(Quaternion a, Quaternion b)
	{
		Quaternion q = new Quaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
		return q;
	}

	// Start is called before the first frame update
	void Start()
	{
		wave = GameObject.Find("Plane");
		collider = GetComponent<Collider>();
		float i = mass / 12;
		inertia[0, 0] = i;
		inertia[1, 1] = i;
		inertia[2, 2] = i;
		inertia[3, 3] = 1;
	}

	void UpdateForce(Vector3 _flotage)
	{
		flotage = _flotage;
	}

	void updateTorque(Vector3 _torque)
	{
		torque = _torque;
	}

	// Update is called once per frame
	void Update()
	{
		wave.SendMessage("UpdateBlockBounds", collider.bounds);

		force = Vector3.zero;
		force.y += mass * g;
		force.y += flotage.y;

		v += h * force;

		v *= 0.99f;

		transform.position += h * v;

		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		Matrix4x4 I = R * inertia * R.transpose;
		Vector3 dw = h * I.inverse.MultiplyPoint(torque);
		w += dw;

		w *= 0.99f;

		Quaternion curr_q = transform.rotation;
		float dt2 = h / 2;
		Quaternion tmp_q = new Quaternion(dt2 * w.x, dt2 * w.y, dt2 * w.z, 0);
		transform.rotation = Quaternion.Normalize(QuatAdd(curr_q, tmp_q * curr_q));

		if (Input.GetMouseButtonDown(0))
		{
			pressed = true;
			Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
			if (Vector3.Cross(ray.direction, transform.position - ray.origin).magnitude < 0.8f) cube_move = true;
			else cube_move = false;
			offset = Input.mousePosition - Camera.main.WorldToScreenPoint(transform.position);
		}
		if (Input.GetMouseButtonUp(0))
		{
			pressed = false;
			cube_move = false;
		}

		if (pressed)
		{
			if (cube_move)
			{
				Vector3 mouse = Input.mousePosition;
				mouse -= offset;
				mouse.z = Camera.main.WorldToScreenPoint(transform.position).z;
				Vector3 p = Camera.main.ScreenToWorldPoint(mouse);
				p.y = transform.position.y;
				transform.position = p;
			}
			else
			{
				//float h = 2.0f * Input.GetAxis("Mouse X");
				//Camera.main.transform.RotateAround(new Vector3(0, 0, 0), Vector3.up, h);
			}
		}
	}
}
