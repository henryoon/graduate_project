using UnityEngine;
using System.IO;
using System.Collections;
using System.Collections.Generic;

public class PointCloudRenderer : MonoBehaviour
{
    public PointCloudSubscriber subscriber;
    public Material pointCloudMaterial;

    private int last_point_length = 0;

    Mesh mesh;
    MeshRenderer meshRenderer;
    MeshFilter mf;

    // The size, positions and colours of each of the pointcloud
    public float pointSize = 1f;

    

    [Header("MAKE SURE THESE LISTS ARE MINIMISED OR EDITOR WILL CRASH")]
    private Vector3[] positions = new Vector3[] { new Vector3(0, 0, 0), new Vector3(0, 1, 0) };
    private Color[] colours = new Color[] { new Color(1f, 0f, 0f), new Color(0f, 1f, 0f) };


    public Transform offset; // Put any gameobject that faciliatates adjusting the origin of the pointcloud in VR. 

    void Start()
    {
        // Give all the required components to the gameObject
        meshRenderer = gameObject.AddComponent<MeshRenderer>();
        mf = gameObject.AddComponent<MeshFilter>();
        meshRenderer.material = pointCloudMaterial;


        mesh = new Mesh
        {
            // Use 32 bit integer values for the mesh, allows for stupid amount of vertices (2,147,483,647 I think?)
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };

        transform.position = offset.position;
        transform.rotation = offset.rotation;
    }

    void UpdateMesh()
    {
        positions = subscriber.GetPCL();
        colours = subscriber.GetPCLColor();
        
        
        if (positions == null)
        {
            return;
        }

        last_point_length = positions.Length;

        mesh.Clear();
        mesh.vertices = positions;
        mesh.colors = colours;
        int[] indices = new int[positions.Length];

        for (int i = 0; i < positions.Length; i++)
        {
            indices[i] = i;
        }

        mesh.SetIndices(indices, MeshTopology.Points, 0);
        mf.mesh = mesh;
        
    }

    void Update()
    {
        transform.position = offset.position;
        transform.rotation = offset.rotation;
        meshRenderer.material.SetFloat("_PointSize", pointSize);

<<<<<<< HEAD
        if (subscriber.GetPCL() == null)
        {
            return;
        }
        
=======
>>>>>>> 7987e2a0465d33f1386f9fb5d57f5444d4022374
        if (last_point_length != subscriber.GetPCL().Length)
        {
            UpdateMesh();
        }   
        // UpdateMesh();
    }
}
