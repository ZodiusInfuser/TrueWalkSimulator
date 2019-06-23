using System.Collections.Generic;
using UnityEngine;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public class FootMotionRegion : MonoBehaviour, IMotionRegion
{
    [SerializeField]
    private float dirX = 0.0f;

    [SerializeField]
    private float dirZ = 0.0f;

    [SerializeField]
    private float radius = 1.0f;

    [SerializeField]
    private float idleRadius = 0.1f;

    [SerializeField]
    protected Color debugColor = new Color(1, 1, 1, 1);

    [SerializeField]
    public List<Vector2> strides;


    public float DirX
    {
        get { return dirX; }
    }

    public float DirZ
    {
        get { return dirX; }
    }

    public float Radius
    {
        get { return radius; }
    }

    public float IdleRadius
    {
        get { return idleRadius; }
    }

    public Color DbgCol
    {
        get { return debugColor; }
    }


    void OnValidate()
    {
        radius = Mathf.Max(radius, 0);
    }

    public FootMotionRegion(float dirX, float dirZ, float radius, float idleRadius)
    {
        this.dirX = dirX;
        this.dirZ = dirZ;
        this.radius = radius;
        this.idleRadius = idleRadius;
    }

    void OnDrawGizmos()
    {
        Transform parentTrans = transform.parent.transform;
        Vector3 dir = new Vector3(dirX, 0.0f, dirZ);
        if (dir.sqrMagnitude == 0.0f)
        {
            Util.DrawEllipse(transform.position, parentTrans.TransformDirection(Vector3.forward), parentTrans.TransformDirection(Vector3.up), radius, radius, 32, debugColor);
        }
        else
        {
            //Debug.DrawRay(transform.TransformPoint(point.position), transform.TransformDirection(dir));
            Util.DrawArc(transform.position + parentTrans.TransformDirection(dir), parentTrans.TransformDirection(dir.normalized), parentTrans.TransformDirection(Vector3.up), radius, radius, 32, debugColor, -90, 180);
            Util.DrawArc(transform.position - parentTrans.TransformDirection(dir), parentTrans.TransformDirection(-dir.normalized), parentTrans.TransformDirection(Vector3.up), radius, radius, 32, debugColor, -90, 180);

            Vector3 radiusVector = new Vector3(dirZ, 0.0f, -dirX);
            radiusVector.Normalize();
            radiusVector *= radius;
            Debug.DrawLine(transform.position - parentTrans.TransformDirection(dir - radiusVector), transform.position + parentTrans.TransformDirection(dir + radiusVector), debugColor);
            Debug.DrawLine(transform.position - parentTrans.TransformDirection(dir + radiusVector), transform.position + parentTrans.TransformDirection(dir - radiusVector), debugColor);
        }
        Util.DrawEllipse(transform.position, parentTrans.TransformDirection(Vector3.forward), parentTrans.TransformDirection(Vector3.up), idleRadius, idleRadius, 32, debugColor);
    }

    private void AngleCheck(float ang, ref float minAng, ref float maxAng, Vector3 inter, ref Vector3 intersection1, ref Vector3 intersection2)
    {
        if (ang <= 0.0f && ang > minAng)
        {
            minAng = ang;
            intersection1 = inter;
        }
        if (ang >= 0.0f && ang < maxAng)
        {
            maxAng = ang;
            intersection2 = inter;
        }
    }

    private int FindIntersections(MotionPoint point, float vx, float vz, out Vector3 intersection1, out Vector3 intersection2, Transform transform, Color col)
    {
        intersection1 = new Vector3(float.NaN, 0.0f, float.NaN);
        intersection2 = new Vector3(float.NaN, 0.0f, float.NaN);

        Vector3 vec = new Vector3(vx, 0.0f, vz);
        Vector3 dir = new Vector3(dirX, 0.0f, dirZ);
        if (dir.sqrMagnitude == 0.0f)
        {
            Vector3 int1, int2;
            int i = Util.FindLineCircleIntersections(point.Position.x, point.Position.z, radius, point.Position, point.Position + vec, out int1, out int2, true);
            if (i >= 1)
            {
                //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                intersection1 = int1;
            }
            if (i >= 2)
            {
                //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                intersection2 = int2;
            }

            return i;
        }
        else
        {
            Vector3 radiusVector = new Vector3(dirZ, 0.0f, -dirX);
            radiusVector.Normalize();
            radiusVector *= radius;
            Vector3 pointA = point.Position - dir;
            Vector3 point1A = point.Position - dir - radiusVector;
            Vector3 point1B = point.Position + dir - radiusVector;

            Vector3 pointB = point.Position + dir;
            Vector3 point2A = point.Position - dir + radiusVector;
            Vector3 point2B = point.Position + dir + radiusVector;

            int bPointsFound = 0;

            {
                Vector3 int1;
                if (Util.FindRayLineIntersection(point.Position, vec, point1A, point1B, out int1))
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                    intersection1 = int1;
                    bPointsFound++;
                }
            }

            {
                Vector3 int1;
                if (Util.FindRayLineIntersection(point.Position, vec, point2A, point2B, out int1))
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                    intersection2 = int1;
                    bPointsFound++;
                }
            }

            if (bPointsFound < 2)
            {
                {
                    Vector3 int1, int2;
                    int i = Util.FindLineCircleIntersections(pointA.x, pointA.z, radius, point.Position, point.Position + vec, out int1, out int2, true);
                    if ((i >= 1) && (Vector3.Dot(int1 - (pointA), dir) < 0.0f))
                    {
                        //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                        intersection1 = int1;
                        bPointsFound++;
                    }
                    if ((i >= 2) && (Vector3.Dot(int2 - (pointA), dir) < 0.0f))
                    {
                        //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                        intersection1 = int2;
                        bPointsFound++;
                    }
                }

                {
                    Vector3 int1, int2;
                    int i = Util.FindLineCircleIntersections(pointB.x, pointB.z, radius, point.Position, point.Position + vec, out int1, out int2, true);
                    if ((i >= 1) && (Vector3.Dot(int1 - (pointB), dir) > 0.0f))
                    {
                        //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                        intersection2 = int1;
                        bPointsFound++;
                    }
                    if ((i >= 2) && (Vector3.Dot(int2 - (pointB), dir) > 0.0f))
                    {
                        //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                        intersection2 = int2;
                        bPointsFound++;
                    }
                }
            }

            if (bPointsFound >= 2)
                return 2;

            return 0;
        }
    }

    private int FindIntersections(MotionPoint point, float cx, float cz, float cradius, out Vector3 intersection1, out Vector3 intersection2, out float minAng, out float maxAng, Transform transform, Color col)
    {
        minAng = float.NegativeInfinity;
        maxAng = float.PositiveInfinity;

        intersection1 = new Vector3(float.NaN, 0.0f, float.NaN);
        intersection2 = new Vector3(float.NaN, 0.0f, float.NaN);

        Vector3 cvec = new Vector3(cx, 0.0f, cz);
        Vector3 dir = new Vector3(dirX, 0.0f, dirZ);
        if (dir.sqrMagnitude == 0.0f)
        {
            Vector3 int1, int2;
            int i = Util.FindCircleCircleIntersections(cx, cz, cradius, point.Position.x, point.Position.z, radius, out int1, out int2);
            if (i >= 1)
            {
                //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                float ang = Vector3.SignedAngle(point.Position - cvec, int1 - cvec, Vector3.up);
                AngleCheck(ang, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
            }
            if (i >= 2)
            {
                //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                float ang = Vector3.SignedAngle(point.Position - cvec, int2 - cvec, Vector3.up);
                AngleCheck(ang, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
            }

            Util.DrawArc(transform.TransformPoint(cvec), transform.TransformDirection(intersection1 - cvec), transform.TransformDirection(Vector3.up), cradius, cradius, 32, col, 0, maxAng - minAng);

            return i;
        }
        else
        {
            Vector3 radiusVector = new Vector3(dirZ, 0.0f, -dirX);
            radiusVector.Normalize();
            radiusVector *= radius;
            Vector3 point1A = point.Position - dir - radiusVector;
            Vector3 point1B = point.Position + dir - radiusVector;

            Vector3 point2A = point.Position - dir + radiusVector;
            Vector3 point2B = point.Position + dir + radiusVector;

            bool bPointsFound = false;

            {
                Vector3 int1, int2;
                int i = Util.FindLineCircleIntersections(cx, cz, cradius, point1A, point1B, out int1, out int2);
                if (i >= 1)
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                    float ang = Vector3.SignedAngle(point.Position - cvec, int1 - cvec, Vector3.up);
                    AngleCheck(ang, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
                if (i >= 2)
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                    float ang = Vector3.SignedAngle(point.Position - cvec, int2 - cvec, Vector3.up);
                    AngleCheck(ang, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
            }

            {
                Vector3 int1, int2;
                int i = Util.FindLineCircleIntersections(cx, cz, cradius, point2A, point2B, out int1, out int2);
                if (i >= 1)
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                    float ang = Vector3.SignedAngle(point.Position - cvec, int1 - cvec, Vector3.up);
                    AngleCheck(ang, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
                if (i >= 2)
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                    float ang = Vector3.SignedAngle(point.Position - cvec, int2 - cvec, Vector3.up);
                    AngleCheck(ang, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
            }

            {
                Vector3 int1, int2;
                int i = Util.FindCircleCircleIntersections(cx, cz, cradius, point.Position.x + dirX, point.Position.z + dirZ, radius, out int1, out int2);
                if ((i >= 1) && (Vector3.Dot(int1 - (point.Position + dir), dir) > 0.0f))
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                    float ang = Vector3.SignedAngle(point.Position - cvec, int1 - cvec, Vector3.up);
                    AngleCheck(ang, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
                if ((i >= 2) && (Vector3.Dot(int2 - (point.Position + dir), dir) > 0.0f))
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                    float ang = Vector3.SignedAngle(point.Position - cvec, int2 - cvec, Vector3.up);
                    AngleCheck(ang, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
            }

            {
                Vector3 int1, int2;
                int i = Util.FindCircleCircleIntersections(cx, cz, cradius, point.Position.x - dirX, point.Position.z - dirZ, radius, out int1, out int2);
                if ((i >= 1) && (Vector3.Dot(int1 - (point.Position - dir), dir) < 0.0f))
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                    float ang = Vector3.SignedAngle(point.Position - cvec, int1 - cvec, Vector3.up);
                    AngleCheck(ang, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
                if ((i >= 2) && (Vector3.Dot(int2 - (point.Position - dir), dir) < 0.0f))
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                    float ang = Vector3.SignedAngle(point.Position - cvec, int2 - cvec, Vector3.up);
                    AngleCheck(ang, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
            }

            if (bPointsFound)
                return 2;

            return 0;
        }
    }

    /// <summary>
    /// Given a point and a relative motion to apply to it, calculate the path that is bounded by this motion region.
    /// Note that rather than returning the length of the path, speed and time are returned instead. This is to enable
    /// easier introduction of other factors into the system before final path lengths are computed.
    /// </summary>
    /// <param name="point">The point that is to have the motion applied to</param>
    /// <param name="motion">The relative motion to apply to the point</param>
    /// <param name="speedMult">A multiplier of the point speed (and divisor of the path time)</param>
    /// <param name="pointSpeed">The speed the point would need to move at to travel along the path in the returned time</param>
    /// <param name="trans">TEMP for debugging</param>
    /// <returns>The time it would take the point to follow the path at the returned speed</returns>
    public float BoundedPathFactors(MotionPoint point, Motion motion, float speedMult, out float pointSpeed, Transform trans)
    {
        float pathTime = float.PositiveInfinity;

        Vector3 linVelocity = motion.LinearVelocity;
        Vector3 origin = motion.RotationOrigin;

        Vector3 intersection1, intersection2;
        if (motion.HasRotation())
        {
            float pathRadius = (point.Position - origin).magnitude;

            float minAngle, maxAngle;
            if (FindIntersections(point, origin.x, origin.z, pathRadius, out intersection1, out intersection2, out minAngle, out maxAngle, trans, debugColor) == 2)
            {
                Util.DrawArc(trans.TransformPoint(origin), trans.TransformDirection(intersection1 - origin), trans.TransformDirection(Vector3.up), pathRadius, pathRadius, 32, debugColor, 0, maxAngle - minAngle);

                Util.DrawEllipse(trans.TransformPoint(origin - Vector3.up * 0.05f), trans.TransformDirection(Vector3.forward), trans.TransformDirection(Vector3.up), pathRadius, pathRadius, 128, Color.grey);
                Debug.DrawLine(trans.TransformPoint(origin), trans.TransformPoint(intersection1), debugColor);
                Debug.DrawLine(trans.TransformPoint(origin), trans.TransformPoint(intersection2), debugColor);
            }
            else
            {
                Util.DrawEllipse(trans.TransformPoint(origin), trans.TransformDirection(Vector3.forward), trans.TransformDirection(Vector3.up), pathRadius, pathRadius, 32, debugColor);
            }

            float angleRange = Mathf.Min(-minAngle, maxAngle);
            float halfPathLength = pathRadius * angleRange * Mathf.Deg2Rad;

            pointSpeed = linVelocity.magnitude * speedMult;
            if (pointSpeed > 0.0f)
                pathTime = halfPathLength / pointSpeed;
        }
        else
        {
            if (FindIntersections(point, linVelocity.x, linVelocity.z, out intersection1, out intersection2, trans, debugColor) == 2)
            {
                Debug.DrawLine(trans.TransformPoint(intersection1), trans.TransformPoint(intersection2), debugColor);

                float halfPathLength = (intersection2 - intersection1).magnitude / 2.0f;

                pointSpeed = linVelocity.magnitude * speedMult;
                if (pointSpeed > 0.0f)
                    pathTime = halfPathLength / pointSpeed;
            }
            else
            {
                pointSpeed = linVelocity.magnitude * speedMult;
                if (pointSpeed > 0.0f)
                    pathTime = 0.0f;
            }
        }

        return pathTime;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////