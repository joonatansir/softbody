using UnityEngine;
using System.Collections;

[RequireComponent(typeof(Softbody))]
public class Visualize : MonoBehaviour
{
  //Public
  public GameObject ParticleMesh = null;

  //Private
  Softbody m_softbody;
  Particle[] m_particles;
  GameObject[] m_particleMeshes;

  void Start()
  {
    m_softbody = GetComponent<Softbody>();
    //m_particles = m_softbody.GetParticles();

    m_particleMeshes = new GameObject[m_particles.Length];

    for (int i = 0; i < m_particleMeshes.Length; i++)
    {
      m_particleMeshes[i] = Instantiate(ParticleMesh, new Vector3(m_particles[i].x.x, m_particles[i].x.y, 0), Quaternion.identity) as GameObject;
    }
  }

  void Update()
  {
    for (int i = 0; i < m_particleMeshes.Length; i++)
    {
      m_particleMeshes[i].transform.position = m_particles[i].x;
    }
  }
}
