using UnityEngine;
using System.Collections;

public class RopeSoftBody : MonoBehaviour
{
  public int Iteration;
  public float InvStiffness;

  Particle[] m_particles;
  StretchConstraint[] m_constraints;
  int numSegments = 20;

	void Start()
  {
    m_particles = new Particle[numSegments];
    m_constraints = new StretchConstraint[numSegments - 1];
    float restLength = 0.25f;
    for (int i = 0; i < numSegments; i++)
    {
      m_particles[i].m = 1f;
      m_particles[i].x = transform.position + new Vector3(restLength * i, 0);

      if(i < numSegments - 1)
      {
        StretchConstraint s = new StretchConstraint();
        s.d = restLength;
        s.x1 = i;
        s.x2 = i + 1;
        m_constraints[i] = s;
      }
    }
    m_particles[0].m = 0;
	}
	
	void Update()
  {
    Softbody.SoftBodyUpdate(m_particles, m_constraints, Iteration, InvStiffness);

    for (int i = 0; i < numSegments - 1; i++)
    {
      float c = 1f;//i / (float)numSegments;
      Debug.DrawLine(m_particles[i].x, m_particles[i + 1].x, new Color(c, c, c));
    }
	}
}
