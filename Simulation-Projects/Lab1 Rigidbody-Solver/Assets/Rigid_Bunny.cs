using UnityEngine;
using System.Collections;

public class Rigid_Bunny: MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0); // angular velocity
	Vector3 g			= new Vector3(0, -9.8f, 0); // gravity
	Vector3[] vertices;

	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;                 // for collision
	float friction = 0.2f;                      // for friction

	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	Quaternion Quaternion_Add(Quaternion a, Quaternion b)
    {
		a.x += b.x;
		a.y += b.y;
		a.z += b.z;
		a.w += b.w;
		return a;
    }

	Matrix4x4 Matrix_Subtract(Matrix4x4 a, Matrix4x4 b)
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				a[i, j] -= b[i, j];
			}
		}
		return a;
	}

	Matrix4x4 Matrix_Miltiply(Matrix4x4 a, float b)
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				a[i, j] *= b;
			}
		}
		return a;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Matrix4x4 r = Matrix4x4.Rotate(transform.rotation);
		Vector3 t = transform.position;
		int num = 0; // calculate virtual collision point
		Vector3 sum = new Vector3(0, 0, 0);
		for (int i = 0; i < vertices.Length; i++)
		{
			Vector3 r_i = vertices[i];
			Vector3 Rri = r.MultiplyVector(r_i);
			Vector3 x_i = t + Rri;
			float d = Vector3.Dot(x_i - P, N);
			if (d < 0) 
            {
				Vector3 v_i = v + Vector3.Cross(w, Rri);
				if (Vector3.Dot(v_i, N) < 0)
                {
					sum += r_i; num++;
                }
			}
		}
		if (num == 0) return;
		Vector3 r_collision = sum / num; // virtual collision point
		Vector3 Rr_collision = r.MultiplyVector(r_collision);
		Vector3 v_collision = v + Vector3.Cross(w, Rr_collision);
		Matrix4x4 I_ten = r * I_ref * Matrix4x4.Transpose(r); // inertia tensor
		Vector3 v_n = Vector3.Dot(v_collision, N) * N; // update velocity
		Vector3 v_t = v_collision - v_n;
		Vector3 v_n_new = -1 * restitution * v_n;
		float a = Mathf.Max(1 - friction * (1 + restitution) * v_n.magnitude / v_n.magnitude, 0);
		Vector3 v_t_new = a * v_t;
		Vector3 v_new = v_n_new + v_n_new;
		Matrix4x4 Rri_star = Get_Cross_Matrix(Rr_collision); // compute impulse j
		Matrix4x4 k = Matrix_Subtract(Matrix_Miltiply(Matrix4x4.identity, 1 / mass),Rri_star * Matrix4x4.Inverse(I_ten) * Rri_star);
		Vector3 j = k.inverse.MultiplyVector(v_new - v_collision);
		v += 1 / mass * j;// update v and w
		w += Matrix4x4.Inverse(I_ten).MultiplyVector(Vector3.Cross(Rr_collision, j));
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			v = w = new Vector3(0, 0, 0);
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (3, 2, 0);
			restitution = 0.5f; launched =true;
		}

		if (!launched) return;
		if (v.magnitude < 1) restitution = 0;

		// Part I: Update velocities
		v += dt * g;
		v *= linear_decay;
		w *= angular_decay;

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		Vector3 x_0 = transform.position;
		Quaternion q_0 = transform.rotation;
		Vector3 x = x_0 + dt * v;
		Vector3 dw = 0.5f * dt * w;
		Quaternion qw = new Quaternion(dw.x, dw.y, dw.z, 0);
		Quaternion q = Quaternion_Add(q_0, qw * q_0);

		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
}
