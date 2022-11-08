using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class map_texture : MonoBehaviour
{
    void Start()
    {
        var mf = GetComponent<MeshFilter>();
        var mesh = new Mesh();
        if (mf != null)
            mesh = mf.mesh;

        if (mesh == null || mesh.uv.Length != 24)
        {
            Debug.Log("Attach To Cube");
            return;
        }

        var uvs = mesh.uv;

        //back
        uvs[0] = new Vector2(0.0f, 0.0f);
        uvs[1] = new Vector2(0.333f, 0.0f);
        uvs[2] = new Vector2(0.0f, 0.333f);
        uvs[3] = new Vector2(0.333f, 0.333f);

        //top
        uvs[8] = new Vector2(0.334f, 0.0f);
        uvs[9] = new Vector2(0.666f, 0.0f);
        uvs[4] = new Vector2(0.334f, 0.333f);
        uvs[5] = new Vector2(0.666f, 0.333f);

        //front
        uvs[10] = new Vector2(0.667f, 0.0f);
        uvs[11] = new Vector2(1.0f, 0.0f);
        uvs[6] = new Vector2(0.667f, 0.333f);
        uvs[7] = new Vector2(1.0f, 0.333f);

        //bottom
        uvs[12] = new Vector2(0.0f, 0.334f);
        uvs[15] = new Vector2(0.333f, 0.334f);
        uvs[13] = new Vector2(0.0f, 0.666f);
        uvs[14] = new Vector2(0.333f, 0.666f);

        //left
        uvs[16] = new Vector2(0.334f, 0.334f);
        uvs[19] = new Vector2(0.666f, 0.334f);
        uvs[17] = new Vector2(0.334f, 0.666f);
        uvs[18] = new Vector2(0.666f, 0.666f);

        //right
        uvs[20] = new Vector2(0.667f, 0.334f);
        uvs[23] = new Vector2(1.0f, 0.334f);
        uvs[21] = new Vector2(0.667f, 0.666f);
        uvs[22] = new Vector2(1.0f, 0.666f);

        mesh.uv = uvs;
    }
}
