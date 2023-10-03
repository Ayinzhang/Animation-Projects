using UnityEngine;
using System.Collections;

public class wave_motion : MonoBehaviour 
{
	const int size 		= 100;
	float rate 		= 0.005f;
	float gamma		= 0.004f;
	float damping 	= 0.996f;
	float g = 9.8f;
	float p = 0.01f;
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;
	float[,]	h = new float[size, size];
	float[,]	new_h = new float[size, size];

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	bool tag = true;
	bool isRainy = false;

	Vector3 	cube_v = Vector3.zero;
	Vector3 	cube_w = Vector3.zero;
	Vector3 cubeMin, cubeMax, blockMin, blockMax;

	Mesh mesh;
	AudioSource audioSource;
	GameObject cube;
	GameObject block;

	Bounds cubeBounds;
	int cubeLi, cubeUi, cubeLj, cubeUj;

	Bounds blockBounds;
	int blockLi, blockUi, blockLj, blockUj;

	IEnumerator Rainy()
    {
		while (true)
		{
			Vector3[] X = mesh.vertices;

			for (int i = 0; i < size; i++)
				for (int j = 0; j < size; j++)
					h[i, j] = X[i * size + j].y;

			int i1 = Random.Range(0, size);
			int j1 = Random.Range(0, size);
			float r = Random.Range(0.1f, 0.3f);

			h[i1, j1] += r;

			for (int i = 0; i < size; i++)
				for (int j = 0; j < size; j++)
					if (i != i1 || j != j1) h[i, j] -= r / (size * size - 1);

			for (int i = 0; i < 6; i++) Shallow_Wave(old_h, h, new_h);

			for (int i = 0; i < size; i++)
				for (int j = 0; j < size; j++)
					X[i * size + j].y = h[i, j];

			mesh.vertices = X;
			mesh.RecalculateNormals();

			yield return new WaitForSeconds(Random.Range(0.01f, 0.03f));
		}
    }


	// Use this for initialization
	void Start () 
	{
		audioSource = GetComponent<AudioSource>();
		mesh = GetComponent<MeshFilter> ().mesh; mesh.Clear ();
		cube = GameObject.Find("Cube");
		block = GameObject.Find("Block");

		Vector3[] X=new Vector3[size*size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0;
			X[i*size+j].z=j*0.1f-size*0.05f;
		}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}
	}

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			Ax[i,j]=0;
			if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
			if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
			if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
			if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
		}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				x[i,j]   +=alpha*cg_p[i,j];
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j];
			}
		}

	}

	void Shallow_Wave(float[,] old_h, float[,] h, float [,] new_h)
	{		
		//Step 1:
		//TODO: Compute new_h based on the shallow wave model.
		for(int i=0;i<size;i++)
			for(int j=0;j<size;j++)
            {
				new_h[i, j] = h[i, j] + damping * (h[i, j] - old_h[i, j]);
				if (i - 1 >= 0) new_h[i, j] += rate * (h[i - 1, j] - h[i, j]);
				if (i + 1 < size) new_h[i, j] += rate * (h[i + 1, j] - h[i, j]);
				if (j - 1 >= 0) new_h[i, j] += rate * (h[i, j - 1] - h[i, j]);
				if (j + 1 < size) new_h[i, j] += rate * (h[i, j + 1] - h[i, j]);
			}

		//Step 2: Block->Water coupling
		//TODO: for block 1, calculate low_h.
		//TODO: then set up b and cg_mask for conjugate gradient.
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		//TODO: for block 2, calculate low_h.
		//TODO: then set up b and cg_mask for conjugate gradient.
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		Vector3[] X = mesh.vertices;
		cubeMin = cubeBounds.min; cubeMax = cubeBounds.max;
		blockMin = blockBounds.min; blockMax = blockBounds.max;

		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				cg_mask[i, j] = false; b[i, j] = low_h[i, j] = vh[i, j] = 0;
			}

		if (cubeMin.y < 0)
		{
			cubeLi = size - 1; cubeLj = size - 1;
			cubeUi = 0; cubeUj = 0;

			for (int i = 0; i < size * size; i++)
				if (X[i].x < cubeMax.x && X[i].x > cubeMin.x && X[i].z < cubeMax.z && X[i].z > cubeMin.z)
				{
					int i1 = i / size;
					int j1 = i % size;
					cg_mask[i1, j1] = true;

					if (i1 < cubeLi) cubeLi = i1;
					if (i1 > cubeUi) cubeUi = i1;
					if (j1 < cubeLj) cubeLj = j1;
					if (j1 > cubeUj) cubeUj = j1;
				}

			for (int i = cubeLi; i <= cubeUi; i++)
				for (int j = cubeLj; j <= cubeUj; j++)
				{
					RaycastHit hit;
					Ray ray = new Ray(new Vector3(X[i * size + j].x, -2, X[i * size + j].z), new Vector3(0, 1, 0));
					if (Physics.Raycast(ray, out hit) && hit.collider != null) low_h[i, j] = hit.point.y;
				}
		}

		if (blockMin.y < 0)
		{
			blockLi = size - 1; blockLj = size - 1;
			blockUi = 0; blockUj = 0;

			for (int i = 0; i < size * size; i++)
				if (X[i].x < blockMax.x && X[i].x > blockMin.x && X[i].z < blockMax.z && X[i].z > blockMin.z)
				{
					int i1 = i / size;
					int j1 = i % size;
					cg_mask[i1, j1] = true;

					if (i1 < blockLi) blockLi = i1;
					if (i1 > blockUi) blockUi = i1;
					if (j1 < blockLj) blockLj = j1;
					if (j1 > blockUj) blockUj = j1;
				}

			for (int i = blockLi; i <= blockUi; i++)
				for (int j = blockLj; j <= blockUj; j++)
				{
					RaycastHit hit;
					Ray ray = new Ray(new Vector3(X[i * size + j].x, -2, X[i * size + j].z), new Vector3(0, 1, 0));
					if (Physics.Raycast(ray, out hit) && hit.collider != null) low_h[i, j] = hit.point.y;
				}
		}

		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
				if (cg_mask[i, j]) b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;

		Conjugate_Gradient(cg_mask, b, vh, cubeLi, cubeUi, cubeLj, cubeUj);
		Conjugate_Gradient(cg_mask, b, vh, blockLi, blockUi, blockLj, blockUj);

		//TODO: Diminish vh.
		//TODO: Update new_h by vh.
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				if (i - 1 >= 0) new_h[i, j] += gamma * rate * (vh[i - 1, j] - vh[i, j]);
				if (i + 1 < size) new_h[i, j] += gamma * rate * (vh[i + 1, j] - vh[i, j]);
				if (j - 1 >= 0) new_h[i, j] += gamma * rate * (vh[i, j - 1] - vh[i, j]);
				if (j + 1 < size) new_h[i, j] += gamma * rate * (vh[i, j + 1] - vh[i, j]);
			}

		X = mesh.vertices;
		Vector3 cubeFlotage = new Vector3();
		Vector3 blockFlotage = new Vector3();
		Vector3 cubeTorque = new Vector3();
		Vector3 blockTorque = new Vector3();

		for (int i = cubeLi; i <= cubeUi; i++)
		{
			for (int j = cubeLj; j <= cubeUj; j++)
			{
				Vector3 _f = new Vector3(0, p * 0.01f * vh[i, j] * g, 0);
				cubeFlotage += _f;

				Vector3 _hitPoint = new Vector3(X[i * size + j].x, low_h[i, j], X[i * size + j].z);
				Vector3 _r = _hitPoint - cube.transform.position;

				Vector3 _torque = Vector3.Cross(_r, _f);
				cubeTorque += _torque;
			}
		}
		for (int i = blockLi; i <= blockUi; i++)
		{
			for (int j = blockLj; j <= blockUj; j++)
			{
				Vector3 _f = new Vector3(0, p * 0.01f * vh[i, j] * g, 0);
				blockFlotage += _f;

				Vector3 _hitPoint = new Vector3(X[i * size + j].x, low_h[i, j], X[i * size + j].z);
				Vector3 _r = _hitPoint - block.transform.position;

				Vector3 _torque = Vector3.Cross(_r, _f);
				blockTorque += _torque;
			}
		}

		cube.SendMessage("UpdateForce", cubeFlotage);
		block.SendMessage("UpdateForce", blockFlotage);

		cube.SendMessage("updateTorque", cubeTorque);
		block.SendMessage("updateTorque", blockTorque);

		//Step 3
		//TODO: old_h <- h; h <- new_h;
		for (int i=0;i<size;i++)
			for(int j=0;j<size;j++)
            {
				old_h[i, j] = h[i, j];h[i, j] = new_h[i, j];
            }

		//Step 4: Water->Block coupling.
		//More TODO here.

	}

	// Update is called once per frame
	void Update () 
	{
		Vector3[] X = mesh.vertices;

		//TODO: Load X.y into h.
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
				h[i, j] = X[i * size + j].y;

		if (Input.GetKeyDown ("d")) 
		{
			//TODO: Add random water.
			int i1 = Random.Range(0, size);
			int j1 = Random.Range(0, size);
			float r = Random.Range(0.1f, 0.3f);

			h[i1, j1] += r;

			for (int i = 0; i < size; i++)
				for (int j = 0; j < size; j++)
					if (i != i1 || j != j1) h[i,j] -= r / (size * size - 1);
		}

		if(Input.GetKeyDown("r"))
        {
			if (!isRainy) { isRainy = true; audioSource.Play(); StartCoroutine("Rainy"); }
			else { isRainy = false; audioSource.Stop(); StopCoroutine("Rainy"); }
        }
	
		for(int l=0; l<6; l++) Shallow_Wave(old_h, h, new_h);

		//TODO: Store h back into X.y and recalculate normal.
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
				X[i * size + j].y = h[i, j];

		mesh.vertices = X;
		mesh.RecalculateNormals();
	}

	void UpdateCubeBounds(Bounds bounds)
	{
		cubeBounds = bounds;
	}

	void UpdateBlockBounds(Bounds bounds)
	{
		blockBounds = bounds;
	}
}
