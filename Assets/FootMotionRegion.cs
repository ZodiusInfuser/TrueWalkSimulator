using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// A Unity script definining a region in which the target position of a leg will reside
/// </summary>
public class FootMotionRegion : MonoBehaviour
{
    //--------------------------------------------------
    // Variables
    //--------------------------------------------------

    /// <summary>
    /// An X axis length of the motion region, creating a 2D capsule 
    /// </summary>
    [SerializeField]
    private float _dirX = 0.0f;

    /// <summary>
    /// A Z axis length of the motion region, creating a 2D capsule 
    /// </summary>
    [SerializeField]
    private float _dirZ = 0.0f;

    /// <summary>
    /// The radius of the motion region
    /// </summary>
    [SerializeField]
    private float _radius = 1.0f;

    /// <summary>
    /// The radius that will trigger walking to start if the target position goes outside it
    /// </summary>
    [SerializeField]
    private float _startRadius = 0.1f;

    /// <summary>
    /// The radius that will indicate that walking can stop if the target position is within it
    /// </summary>
    [SerializeField]
    private float _stopRadius = 0.1f;

    /// <summary>
    /// The colour to use for the debug lines this motion region can show
    /// </summary>
    [SerializeField]
    private Color _debugColor = new Color(1, 1, 1, 1);

    /// <summary>
    /// The peak height strides associated with this motion region should go up to
    /// </summary>
    [SerializeField]
    [Range(0, 5)]
    private float _peakHeight = 1.0f;

    /// <summary>
    /// A list of unity stride definitions used to form a channel of a walking gait
    /// </summary>
    [SerializeField]
    public List<GaitStrideData> strides;

    /// <summary>
    /// A list of 2D points defining the shape of the balance area associated with this motion region.
    /// By default this can be empty but it can be useful for simulating biped robots
    /// </summary>
    [SerializeField]
    public List<Vector2> _shape;


    //--------------------------------------------------
    // Properties
    //--------------------------------------------------

    /// <summary>
    /// The X axis length of this motion region
    /// </summary>
    public float DirX
    {
        get { return _dirX; }
    }

    /// <summary>
    /// The Z axis length of this motion region
    /// </summary>
    public float DirZ
    {
        get { return _dirX; }
    }

    /// <summary>
    /// The radius of this motion region
    /// </summary>
    public float Radius
    {
        get { return _radius; }
    }

    /// <summary>
    /// The radius that will trigger walking to start if the target position goes outside it
    /// </summary>
    public float StartRadius
    {
        get { return _startRadius; }
    }

    /// <summary>
    /// The radius that will indicate that walking can stop if the target position is within it
    /// </summary>
    public float StopRadius
    {
        get { return _stopRadius; }
    }

    /// <summary>
    /// The colour used for the debug lines this motion region can show
    /// </summary>
    public Color DbgCol
    {
        get { return _debugColor; }
    }

    /// <summary>
    /// The peak height strides associated with this motion region should go up to
    /// </summary>
    public float PeakHeight
    {
        get { return _peakHeight; }
    }

    /// <summary>
    /// A list of 2D points defining the shape of the balance area associated with this motion region
    /// </summary>
    public List<Vector2> Shape
    {
        get { return _shape; }
    }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// Clamp the radius values to be valid
    /// </summary>
    void OnValidate()
    {
        _radius = Mathf.Max(_radius, 0);
        _startRadius = Mathf.Clamp(_startRadius, 0, _radius);
        _stopRadius = Mathf.Clamp(_stopRadius, _startRadius, _radius);
    }

    /// <summary>
    /// Draw the debug lines and shapes for this motion region
    /// </summary>
    void OnDrawGizmos()
    {
        Transform parentTrans = transform.parent.transform;
        Vector3 dir = new Vector3(_dirX, 0.0f, _dirZ);
        if (dir.sqrMagnitude == 0.0f)
        {
            Util.DrawEllipse(transform.position, parentTrans.TransformDirection(Vector3.forward), parentTrans.TransformDirection(Vector3.up), _radius, _radius, 32, _debugColor);
        }
        else
        {
            //Debug.DrawRay(transform.TransformPoint(point.position), transform.TransformDirection(dir));
            Util.DrawArc(transform.position + parentTrans.TransformDirection(dir), parentTrans.TransformDirection(dir.normalized), parentTrans.TransformDirection(Vector3.up), _radius, _radius, 32, _debugColor, -90, 180);
            Util.DrawArc(transform.position - parentTrans.TransformDirection(dir), parentTrans.TransformDirection(-dir.normalized), parentTrans.TransformDirection(Vector3.up), _radius, _radius, 32, _debugColor, -90, 180);

            Vector3 radiusVector = new Vector3(_dirZ, 0.0f, -_dirX);
            radiusVector.Normalize();
            radiusVector *= _radius;
            Debug.DrawLine(transform.position - parentTrans.TransformDirection(dir - radiusVector), transform.position + parentTrans.TransformDirection(dir + radiusVector), _debugColor);
            Debug.DrawLine(transform.position - parentTrans.TransformDirection(dir + radiusVector), transform.position + parentTrans.TransformDirection(dir - radiusVector), _debugColor);
        }
        Util.DrawEllipseDashed(transform.position, parentTrans.TransformDirection(Vector3.forward), parentTrans.TransformDirection(Vector3.up), _startRadius, _startRadius, 32, _debugColor, false);
        Util.DrawEllipseDashed(transform.position, parentTrans.TransformDirection(Vector3.forward), parentTrans.TransformDirection(Vector3.up), _stopRadius, _stopRadius, 32, _debugColor, true);

        if (transform.childCount == 1)
        {
            Transform target = transform.GetChild(0);

            Vector3 worldFinalPosition = transform.TransformPoint(target.localPosition);

            List<Vector3> shape = new List<Vector3>();
            shape.Add(worldFinalPosition);
            foreach (Vector2 vertex in _shape)
            {
                Debug.DrawLine(worldFinalPosition, worldFinalPosition + transform.TransformDirection(target.localRotation * new Vector3(vertex.x, 0, vertex.y)), _debugColor);
                shape.Add(worldFinalPosition + transform.TransformDirection(target.localRotation * new Vector3(vertex.x, 0, vertex.y)));
            }

            List<Vector3> hull;
            Util.ConvexHull(shape, out hull);
            if (hull != null)
            {
                Vector3[] pointsOut = new Vector3[hull.Count];
                int i = 0;
                foreach (Vector3 temp in hull)
                {
                    pointsOut[i] = temp;
                    i++;
                }

                Handles.color = new Color(_debugColor.r, _debugColor.g, _debugColor.b, 0.25f);
                Handles.DrawAAConvexPolygon(pointsOut);
            }

            Debug.DrawRay(worldFinalPosition, transform.TransformDirection(target.localRotation * new Vector3(0.1f, 0, 0.1f)), Color.yellow);
            Debug.DrawRay(worldFinalPosition, transform.TransformDirection(target.localRotation * new Vector3(-0.1f, 0, 0.1f)), Color.yellow);
            Debug.DrawRay(worldFinalPosition, transform.TransformDirection(target.localRotation * new Vector3(0.1f, 0, -0.1f)), Color.yellow);
            Debug.DrawRay(worldFinalPosition, transform.TransformDirection(target.localRotation * new Vector3(-0.1f, 0, -0.1f)), Color.yellow);

            Debug.DrawLine(worldFinalPosition + transform.TransformDirection(target.localRotation * new Vector3(0.1f, 0, 0.1f)),
                           worldFinalPosition + transform.TransformDirection(target.localRotation * new Vector3(-0.1f, 0, 0.1f)), Color.yellow);

            Debug.DrawLine(transform.position, worldFinalPosition, Color.gray);
        }
    }

    /// <summary>
    /// Peform checks on the provided angle against a minimum and maximum angle, to determine which intersections are most likely.
    /// This modifies the min and max angles if the angle is greater or less than them, respectively,
    /// and will replace intersection1 and/or intersection2 with the value of inter
    /// </summary>
    /// <param name="ang">The angle to check</param>
    /// <param name="minAng">The minimum angle value to check against (and modify if smaller)</param>
    /// <param name="maxAng">The maximum angle value to check against (and modify if larger)</param>
    /// <param name="inter">The intersection point to override the other two with if either the min or max angle is modified</param>
    /// <param name="intersection1">The previously calculated first intersection point</param>
    /// <param name="intersection2">The previously calculated second intersection point</param>
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

    /// <summary>
    /// Find the 2D intersections of this motion region around the provided MotionPoint with the provided vector, through the motion point's centre
    /// </summary>
    /// <param name="point">The point to have this motion region placed around</param>
    /// <param name="vx">The X component of the vector</param>
    /// <param name="vz">The Z component of the vector</param>
    /// <param name="intersection1">The first intersection output</param>
    /// <param name="intersection2">The second intersection output</param>
    /// <param name="transform">The transform to use for debugging purposes</param>
    /// <param name="col">The colour to draw debug lines and shapes with</param>
    /// <returns>The number of intersections found (0, 1, or 2)</returns>
    private int FindIntersections(MotionPoint point, float vx, float vz, out Vector3 intersection1, out Vector3 intersection2, Transform transform, Color col)
    {
        intersection1 = new Vector3(float.NaN, 0.0f, float.NaN);
        intersection2 = new Vector3(float.NaN, 0.0f, float.NaN);

        Vector3 vec = Util.Rotate(new Vector3(vx, 0.0f, vz), point.Angle);
        Vector3 dir = new Vector3(_dirX, 0.0f, _dirZ);
        if (dir.sqrMagnitude == 0.0f)
        {
            Vector3 int1, int2;
            int i = Util.FindLineCircleIntersections(point.Position.x, point.Position.z, _radius, point.Position, point.Position + vec, out int1, out int2, true);
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
            Vector3 radiusVector = new Vector3(_dirZ, 0.0f, -_dirX);
            radiusVector.Normalize();
            radiusVector *= _radius;
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
                    int i = Util.FindLineCircleIntersections(pointA.x, pointA.z, _radius, point.Position, point.Position + vec, out int1, out int2, true);
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
                    int i = Util.FindLineCircleIntersections(pointB.x, pointB.z, _radius, point.Position, point.Position + vec, out int1, out int2, true);
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

    /// <summary>
    /// Find the 2D intersections of this motion region around the provided MotionPoint with the provided circle
    /// </summary>
    /// <param name="point">The point to have this motion region placed around</param>
    /// <param name="cx">The X coordinate of the circle's centre</param>
    /// <param name="cz">The Y coordinate of the circle's centre</param>
    /// <param name="cradius">The radius of the circle</param>
    /// <param name="intersection1">The first intersection output</param>
    /// <param name="intersection2">The second intersection output</param>
    /// <param name="minAng">The resulting minimum angle of the intersected circle arc</param>
    /// <param name="maxAng">The resulting maximum angle of the intersected circle arc</param>
    /// <param name="transform">The transform to use for debugging purposes</param>
    /// <param name="col">The colour to draw debug lines and shapes with</param>
    /// <returns>The number of intersections found (0, 1, or 2)</returns>
    private int FindIntersections(MotionPoint point, float cx, float cz, float cradius, out Vector3 intersection1, out Vector3 intersection2, out float minAng, out float maxAng, Transform transform, Color col)
    {
        minAng = float.NegativeInfinity;
        maxAng = float.PositiveInfinity;

        intersection1 = new Vector3(float.NaN, 0.0f, float.NaN);
        intersection2 = new Vector3(float.NaN, 0.0f, float.NaN);

        Vector3 cvec = new Vector3(cx, 0.0f, cz);
        Vector3 dir = new Vector3(_dirX, 0.0f, _dirZ);
        if (dir.sqrMagnitude == 0.0f)
        {
            Vector3 int1, int2;
            int i = Util.FindCircleCircleIntersections(cx, cz, cradius, point.Position.x, point.Position.z, _radius, out int1, out int2);
            if (i >= 1)
            {
                //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                float ang = Util.SignedYAngle(point.Position - cvec, int1 - cvec);
                AngleCheck(ang, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
            }
            if (i >= 2)
            {
                //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                float ang = Util.SignedYAngle(point.Position - cvec, int2 - cvec);
                AngleCheck(ang, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
            }

            Util.DrawArc(transform.TransformPoint(cvec), transform.TransformDirection(intersection1 - cvec), transform.TransformDirection(Vector3.up), cradius, cradius, 32, col, 0, maxAng - minAng);

            return i;
        }
        else
        {
            Vector3 radiusVector = new Vector3(_dirZ, 0.0f, -_dirX);
            radiusVector.Normalize();
            radiusVector *= _radius;
            Vector3 point1A = point.Position - dir - radiusVector;
            Vector3 point1B = point.Position + dir - radiusVector;

            Vector3 point2A = point.Position - dir + radiusVector;
            Vector3 point2B = point.Position + dir + radiusVector;

            bool bPointsFound = false;

            //I will admit this code is a bit of a mess, but doing capsule to circle intersection checks can produce at least four intersection points,
            //so need to check the angles of the intersections relative to the circle centre in order to find the correct two intersection points to return
            {
                Vector3 int1, int2;
                int i = Util.FindLineCircleIntersections(cx, cz, cradius, point1A, point1B, out int1, out int2);
                if (i >= 1)
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                    float ang = Util.SignedYAngle(point.Position - cvec, int1 - cvec);
                    AngleCheck(ang, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
                if (i >= 2)
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                    float ang = Util.SignedYAngle(point.Position - cvec, int2 - cvec);
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
                    float ang = Util.SignedYAngle(point.Position - cvec, int1 - cvec);
                    AngleCheck(ang, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
                if (i >= 2)
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                    float ang = Util.SignedYAngle(point.Position - cvec, int2 - cvec);
                    AngleCheck(ang, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
            }

            {
                Vector3 int1, int2;
                int i = Util.FindCircleCircleIntersections(cx, cz, cradius, point.Position.x + _dirX, point.Position.z + _dirZ, _radius, out int1, out int2);
                if ((i >= 1) && (Vector3.Dot(int1 - (point.Position + dir), dir) > 0.0f))
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                    float ang = Util.SignedYAngle(point.Position - cvec, int1 - cvec);
                    AngleCheck(ang, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
                if ((i >= 2) && (Vector3.Dot(int2 - (point.Position + dir), dir) > 0.0f))
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                    float ang = Util.SignedYAngle(point.Position - cvec, int2 - cvec);
                    AngleCheck(ang, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int2, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
            }

            {
                Vector3 int1, int2;
                int i = Util.FindCircleCircleIntersections(cx, cz, cradius, point.Position.x - _dirX, point.Position.z - _dirZ, _radius, out int1, out int2);
                if ((i >= 1) && (Vector3.Dot(int1 - (point.Position - dir), dir) < 0.0f))
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int1));
                    float ang = Util.SignedYAngle(point.Position - cvec, int1 - cvec);
                    AngleCheck(ang, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang + 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    AngleCheck(ang - 360, ref minAng, ref maxAng, int1, ref intersection1, ref intersection2);
                    bPointsFound = true;
                }
                if ((i >= 2) && (Vector3.Dot(int2 - (point.Position - dir), dir) < 0.0f))
                {
                    //Debug.DrawLine(transform.TransformPoint(point.position), transform.TransformPoint(int2));
                    float ang = Util.SignedYAngle(point.Position - cvec, int2 - cvec);
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
    /// easier introduction of other factors into the system before final path lengths are computed
    /// </summary>
    /// <param name="point">The point that is to have the motion applied to</param>
    /// <param name="motion">The relative motion to apply to the point</param>
    /// <param name="speedMult">A multiplier of the point speed (and divisor of the path time)</param>
    /// <param name="pointSpeed">The speed the point would need to move at to travel along the path in the returned time</param>
    /// <param name="transform">The transform to use for debugging purposes</param>
    /// <returns>The time it would take the point to follow the path at the returned speed</returns>
    public float BoundedPathFactors(MotionPoint point, Motion motion, float speedMult, out float pointSpeed, Transform trans)
    {
        float pathTime = float.PositiveInfinity;

        Vector3 linVelocity = motion.LinearVelocity / Time.deltaTime;
        if (motion.HasRotation())
        {
            Vector3 origin = motion.RotationOrigin;
            float pathRadius = (point.Position - origin).magnitude;

            float minAngle, maxAngle;
            Vector3 intersection1, intersection2;
            if (FindIntersections(point, origin.x, origin.z, pathRadius, out intersection1, out intersection2, out minAngle, out maxAngle, trans, _debugColor) == 2)
            {
                Util.DrawArc(trans.TransformPoint(origin), trans.TransformDirection(intersection1 - origin), trans.TransformDirection(Vector3.up), pathRadius, pathRadius, 32, _debugColor, 0, maxAngle - minAngle);

                Util.DrawEllipse(trans.TransformPoint(origin - Vector3.up * 0.05f), trans.TransformDirection(Vector3.forward), trans.TransformDirection(Vector3.up), pathRadius, pathRadius, 128, Color.grey);
                Debug.DrawLine(trans.TransformPoint(origin), trans.TransformPoint(intersection1), _debugColor);
                Debug.DrawLine(trans.TransformPoint(origin), trans.TransformPoint(intersection2), _debugColor);
            }
            else
            {
                Util.DrawEllipse(trans.TransformPoint(origin), trans.TransformDirection(Vector3.forward), trans.TransformDirection(Vector3.up), pathRadius, pathRadius, 32, _debugColor);
            }

            float angleRange = Mathf.Min(-minAngle, maxAngle);
            float halfPathLength = pathRadius * angleRange * Mathf.Deg2Rad;

            pointSpeed = linVelocity.magnitude * speedMult;
            if (pointSpeed > 0.0f)
                pathTime = halfPathLength / pointSpeed;
        }
        else
        {
            Vector3 intersection1, intersection2;
            if (FindIntersections(point, linVelocity.x, linVelocity.z, out intersection1, out intersection2, trans, _debugColor) == 2)
            {
                Debug.DrawLine(trans.TransformPoint(intersection1), trans.TransformPoint(intersection2), _debugColor);

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

    /// <summary>
    /// Update the position of the first child target Transform with the provided body relative position and angle
    /// </summary>
    /// <param name="bodyRelativePos">The body relative position to set the target to</param>
    /// <param name="bodyRelativeAngle">The body relative Y axis angle to set the target to</param>
    /// <param name="bodyTrans">The transform of the body the position and angle are relative to (used to calculate world values)</param>
    public void UpdateTarget(Vector3 bodyRelativePos, float bodyRelativeAngle, Transform bodyTrans)
    {
        if (transform.childCount == 1)
        {
            Transform target = transform.GetChild(0);
            Matrix4x4 relative = transform.worldToLocalMatrix * bodyTrans.localToWorldMatrix;
            Vector3 posOut = relative.MultiplyPoint(bodyRelativePos);
            Quaternion rotOut = Quaternion.LookRotation(relative.GetColumn(2), relative.GetColumn(1));
            target.localPosition = posOut;
            target.localRotation = rotOut * Quaternion.Euler(0, bodyRelativeAngle, 0);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////