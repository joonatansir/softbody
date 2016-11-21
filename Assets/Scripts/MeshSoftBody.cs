using UnityEngine;
using System.Collections.Generic;

public class MeshSoftBody : MonoBehaviour
{
  //Public
  public int Iterations;
  public float Stiffness;

  //Private
  private MeshFilter m_meshFilter;
  private Mesh m_mesh;
  private Particle[] m_particles;
  private StretchConstraint[] m_constraints;
  private Vector2 m_centerOfMass;

  void Start()
  {
    m_meshFilter = GetComponent<MeshFilter>();

    if (!m_meshFilter)
    {
      Debug.LogError("No MeshFilter!");
      return;
    }

    m_mesh = m_meshFilter.mesh;

    if (!m_mesh)
    {
      Debug.LogError("No Mesh!");
      return;
    }

    m_centerOfMass = new Vector2();
    Vector3[] vertices = m_mesh.vertices;
    m_particles = new Particle[vertices.Length];
    for (int i = 0; i < vertices.Length; i++)
    {
      Vector2 pos = m_meshFilter.transform.TransformPoint(vertices[i]);
      m_particles[i].x = pos;
      m_particles[i].v = Vector2.zero;
      m_particles[i].m = 1f;
      m_centerOfMass += pos;
    }

    m_centerOfMass /= m_particles.Length;

    for (int i = 0; i < m_particles.Length; i++)
    {
      m_particles[i].q = m_particles[i].x - m_centerOfMass;
    }

    //m_particles[0].m = 0;

    int[] triangles = m_mesh.triangles;
    float d = Vector3.Magnitude(m_meshFilter.transform.TransformPoint(vertices[0]) - m_meshFilter.transform.TransformPoint(vertices[1]));
    HashSet<StretchConstraint> set = new HashSet<StretchConstraint>();
    for (int i = 0; i < triangles.Length; i += 3)
    {
      StretchConstraint s = new StretchConstraint();
      s.x1 = Mathf.Min(triangles[i], triangles[i + 1]);
      s.x2 = Mathf.Max(triangles[i], triangles[i + 1]);
      s.d = Vector3.Magnitude(m_meshFilter.transform.TransformPoint(vertices[triangles[i]]) -
                              m_meshFilter.transform.TransformPoint(vertices[triangles[i + 1]]));
      set.Add(s);

      StretchConstraint s1 = new StretchConstraint();
      s1.x1 = Mathf.Min(triangles[i], triangles[i + 2]);
      s1.x2 = Mathf.Max(triangles[i], triangles[i + 2]);
      s1.d = Vector3.Magnitude(m_meshFilter.transform.TransformPoint(vertices[triangles[i]]) -
                               m_meshFilter.transform.TransformPoint(vertices[triangles[i + 2]])); ;
      set.Add(s1);

      StretchConstraint s2 = new StretchConstraint();
      s2.x1 = Mathf.Min(triangles[i + 1], triangles[i + 2]);
      s2.x2 = Mathf.Max(triangles[i + 1], triangles[i + 2]);
      s2.d = Vector3.Magnitude(m_meshFilter.transform.TransformPoint(vertices[triangles[i + 1]]) -
                               m_meshFilter.transform.TransformPoint(vertices[triangles[i + 2]])); ;
      set.Add(s2);
    }

    m_constraints = new StretchConstraint[set.Count];
    set.CopyTo(m_constraints);
  }

  void Update()
  {
    float invStiffness = 1 / (Stiffness * Stiffness);
    Softbody.SoftBodyUpdate(m_particles, m_constraints, Iterations, invStiffness);

    Vector3[] vertices = m_mesh.vertices;
    for (int i = 0; i < vertices.Length; i++)
    {
      vertices[i] = m_meshFilter.transform.InverseTransformPoint(m_particles[i].x);
    }
    m_mesh.vertices = vertices;
  }
}