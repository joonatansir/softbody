using UnityEngine;
using System.Collections.Generic;

public struct Particle
{
  public Vector2 x;
  public Vector2 v;
  public float m;
  public Vector2 q; //Vector from center of mass to this particle
}

public struct StretchConstraint
{
  public int x1;
  public int x2;
  public float d;
  public float l;
}

public struct PlaneCollisionConstraint
{
  public Vector2 n;
  public Vector2 p;
  public int x;
  public float d;
}

public struct Plane
{
  public Vector2 p;
  public Vector2 n;
  public float d;
}

public class Softbody : MonoBehaviour
{
  //Private
  static bool init = false;
  static Plane[] m_planes;
  static PlaneCollisionConstraint[] m_planeCollConstraints;
  static int m_numPlaneCollisions = 0;

  static void InitCollisionPlanes()
  {
    init = true;

    m_planeCollConstraints = new PlaneCollisionConstraint[500];

    //Some collision planes
    Plane p = new Plane();
    p.n = Vector2.up;
    p.p = new Vector2(0, -9);
    p.d = 0f;

    Plane p2 = new Plane();
    p2.n = Vector2.down;
    p2.p = new Vector2(0, Camera.main.orthographicSize);
    p2.d = 0f;

    //Plane p3 = new Plane();
    //p3.n = new Vector2(-1, 1).normalized;
    //p3.p = new Vector2(0, -9);
    //p3.d = 0f;

    m_planes = new Plane[] { p, p2 };
  }

  public static void SoftBodyUpdate(Particle[] particles, StretchConstraint[] constraints, int iterations, float invStiffness)
  {
    if (!init)
      InitCollisionPlanes();

    Vector2 g = Vector2.down * 9.81f;
    float dt = Time.deltaTime;

    Vector2[] predictedPositions = new Vector2[particles.Length];

    for (int i = 0; i < particles.Length; i++)
      predictedPositions[i] = particles[i].x + dt * particles[i].v + dt * dt * particles[i].m * g;

    for (int i = 0; i < constraints.Length; i++)
      constraints[i].l = 0;

    GenerateCollisionConstraints(particles);

    for (int i = 0; i < iterations; i++)
    {
      SolvePlaneCollisions(predictedPositions);
      SolveConstraints(predictedPositions, constraints, particles, invStiffness);
      //ShapeMatch(particles, predictedPositions, invStiffness);
    }

    for (int i = 0; i < particles.Length; i++)
    {
      particles[i].v = (1.0f / dt) * (predictedPositions[i] - particles[i].x);
      particles[i].x = predictedPositions[i];
    }
  }

  static void ShapeMatch(Particle[] particles, Vector2[] predictedPositions, float InvStiffness)
  {
    float centerOfMassx = 0;
    float centerOfMassy = 0;
    for (int i = 0; i < predictedPositions.Length; i++)
    {
      centerOfMassx += predictedPositions[i].x;
      centerOfMassy += predictedPositions[i].y;
    }
    centerOfMassx /= predictedPositions.Length;
    centerOfMassy /= predictedPositions.Length;

    float[] A = new float[4];
    for (int i = 0; i < predictedPositions.Length; i++)
    {
      A[0] += particles[i].m * (predictedPositions[i].x - centerOfMassx) * particles[i].q.x;
      A[1] += particles[i].m * (predictedPositions[i].x - centerOfMassx) * particles[i].q.y;
      A[2] += particles[i].m * (predictedPositions[i].y - centerOfMassy) * particles[i].q.x;
      A[3] += particles[i].m * (predictedPositions[i].y - centerOfMassy) * particles[i].q.y;
    }

    float det = Mathf.Sign(A[0] * A[3] - A[1] * A[2]);
    A[0] += det * A[3];
    A[1] += det * -A[2];
    A[2] += det * -A[1];
    A[3] += det * A[0];

    Vector2 n1 = new Vector2(A[0], A[1]).normalized;
    Vector2 n2 = new Vector2(A[2], A[3]).normalized;

    A[0] = n1.x;
    A[1] = n1.y;
    A[2] = n2.x;
    A[3] = n2.y;

    float magic = 0.007f;

    for (int i = 0; i < particles.Length; i++)
    {
      //Vector2 q = particles[i].q;
      //Vector2 g = new Vector2(A[0] * q.x + A[1] * q.y, A[2] * q.x + A[3] * q.y) + centerOfMass;
      float gx = (A[0] * particles[i].q.x + A[1] * particles[i].q.y) + centerOfMassx;
      float gy = (A[2] * particles[i].q.x + A[3] * particles[i].q.y) + centerOfMassy;
      predictedPositions[i].x += magic * (gx - predictedPositions[i].x);
      predictedPositions[i].y += magic * (gy - predictedPositions[i].y);
    }
  }

  static void SolveConstraints(Vector2[] predictedPositions,
                               StretchConstraint[] constraints,
                               Particle[] particles,
                               float invStiffness)
  {
    float a = invStiffness / (Time.deltaTime * Time.deltaTime);

    for (int i = 0; i < constraints.Length; i++)
    {
      float nx = predictedPositions[constraints[i].x1].x - predictedPositions[constraints[i].x2].x;
      float ny = predictedPositions[constraints[i].x1].y - predictedPositions[constraints[i].x2].y;
      float length = Mathf.Sqrt(nx*nx+ny*ny);
      float dd = length - constraints[i].d;
      nx /= length;
      ny /= length;

      float w1 = particles[constraints[i].x1].m;
      float w2 = particles[constraints[i].x2].m;

      //Calculate Lagrange multiplier delta
      float dl = (dd - a * constraints[i].l) / ((w1 + w2) + a);
      constraints[i].l += dl;

      //Update positions
      predictedPositions[constraints[i].x1].x -= nx * (dl * w1);
      predictedPositions[constraints[i].x1].y -= ny * (dl * w1);
      predictedPositions[constraints[i].x2].x += nx * (dl * w2);
      predictedPositions[constraints[i].x2].y += ny * (dl * w2);
    }
  }

  static void SolvePlaneCollisions(Vector2[] predictedPositions)
  {
    for (int i = 0; i < m_numPlaneCollisions; i++)
    {
      Vector2 v = predictedPositions[m_planeCollConstraints[i].x] - m_planeCollConstraints[i].p;
      float VdotN = Vector2.Dot(v, -m_planeCollConstraints[i].n);
      predictedPositions[m_planeCollConstraints[i].x] += m_planeCollConstraints[i].n * (VdotN + m_planeCollConstraints[i].d);
    }
  }

  static void GenerateCollisionConstraints(Particle[] particles)
  {
    m_numPlaneCollisions = 0;

    for (int i = 0; i < m_planes.Length; i++)
    {
      for (int j = 0; j < particles.Length; j++)
      {
        Vector2 v = particles[j].x - m_planes[i].p;
        float vDotn = Vector2.Dot(v, -m_planes[i].n);
        if (vDotn > 0)
        {
          m_planeCollConstraints[m_numPlaneCollisions].x = j;
          m_planeCollConstraints[m_numPlaneCollisions].p = m_planes[i].p;
          m_planeCollConstraints[m_numPlaneCollisions].d = m_planes[i].d;
          m_planeCollConstraints[m_numPlaneCollisions].n = m_planes[i].n;
          m_numPlaneCollisions = Mathf.Clamp(++m_numPlaneCollisions, 0, m_planeCollConstraints.Length);
        }
      }
    }
  }
}
