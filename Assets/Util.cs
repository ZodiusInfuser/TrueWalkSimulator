using UnityEngine;
using UnityEditor;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Static Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public static class Util
{
    //http://csharphelper.com/blog/2014/08/determine-where-two-lines-intersect-in-c/
    public static bool FindRayLineIntersection(Vector3 p3, Vector3 p4, Vector3 p1, Vector3 p2, out Vector3 intersection)
    {
        // Get the segments' parameters.
        float dx12 = p2.x - p1.x;
        float dz12 = p2.z - p1.z;
        float dx34 = p4.x;
        float dz34 = p4.z;

        // Solve for t1 and t2
        float denominator = (dz12 * dx34 - dx12 * dz34);

        float t1 = ((p1.x - p3.x) * dz34 + (p3.z - p1.z) * dx34) / denominator;
        if (float.IsInfinity(t1))
        {
            // The lines are parallel (or close enough to it).
            intersection = new Vector3(float.NaN, 0.0f, float.NaN);
            return false;
        }

        // Find the point of intersection.
        intersection = new Vector3(p1.x + dx12 * t1, 0.0f, p1.z + dz12 * t1);

        // The segments intersect if t1 and t2 are between 0 and 1.
        return (t1 >= -0.00001) && (t1 <= 1.00001);// && (t2 >= 0) && (t2 <= 1));
    }

    //http://csharphelper.com/blog/2014/09/determine-where-a-line-intersects-a-circle-in-c/
    public static int FindLineCircleIntersections(float cx, float cz, float radius, Vector3 point1, Vector3 point2,
                                                     out Vector3 intersection1, out Vector3 intersection2, bool bInfinite = false)
    {
        float dx, dz, A, B, C, det, t;

        dx = point2.x - point1.x;
        dz = point2.z - point1.z;

        A = dx * dx + dz * dz;
        B = 2 * (dx * (point1.x - cx) + dz * (point1.z - cz));
        C = (point1.x - cx) * (point1.x - cx) + (point1.z - cz) * (point1.z - cz) - radius * radius;

        det = B * B - 4 * A * C;
        if ((A <= 0.0000001) || (det < 0))
        {
            // No real solutions.
            intersection1 = new Vector3(float.NaN, 0.0f, float.NaN);
            intersection2 = new Vector3(float.NaN, 0.0f, float.NaN);
            return 0;
        }
        else if (det == 0)
        {
            // One solution.
            t = -B / (2 * A);
            intersection1 = new Vector3(point1.x + t * dx, 0.0f, point1.z + t * dz);
            intersection2 = new Vector3(float.NaN, 0.0f, float.NaN);
            return 1;
        }
        else
        {
            // Two solutions.
            t = (float)((-B + Mathf.Sqrt(det)) / (2 * A));
            intersection1 = new Vector3(point1.x + t * dx, 0.0f, point1.z + t * dz);
            t = (float)((-B - Mathf.Sqrt(det)) / (2 * A));
            intersection2 = new Vector3(point1.x + t * dx, 0.0f, point1.z + t * dz);

            int results = 2;
            if (!bInfinite)
            {
                {
                    float dx1 = intersection2.x - point1.x;
                    float dz1 = intersection2.z - point1.z;
                    float dx2 = intersection2.x - point2.x;
                    float dz2 = intersection2.z - point2.z;
                    float A1 = dx1 * dx1 + dz1 * dz1;
                    float A2 = dx2 * dx2 + dz2 * dz2;
                    if (A1 > A || A2 > A)
                    {
                        intersection2 = new Vector3(float.NaN, 0.0f, float.NaN);
                        results--;
                    }
                }

                {
                    float dx1 = intersection1.x - point1.x;
                    float dz1 = intersection1.z - point1.z;
                    float dx2 = intersection1.x - point2.x;
                    float dz2 = intersection1.z - point2.z;
                    float A1 = dx1 * dx1 + dz1 * dz1;
                    float A2 = dx2 * dx2 + dz2 * dz2;
                    if (A1 > A || A2 > A)
                    {
                        intersection1 = intersection2;
                        intersection2 = new Vector3(float.NaN, 0.0f, float.NaN);
                        results--;
                    }
                }
            }

            return results;
        }
    }

    // Find the points where the two circles intersect.
    //http://csharphelper.com/blog/2014/09/determine-where-two-circles-intersect-in-c/
    public static int FindCircleCircleIntersections(float cx0, float cz0, float radius0, float cx1, float cz1, float radius1,
                                                       out Vector3 intersection1, out Vector3 intersection2)
    {
        // Find the distance between the centers.
        float dx = cx0 - cx1;
        float dz = cz0 - cz1;
        float dist = Mathf.Sqrt(dx * dx + dz * dz);

        // See how many solutions there are.
        if (dist > radius0 + radius1)
        {
            // No solutions, the circles are too far apart.
            intersection1 = new Vector3(float.NaN, 0.0f, float.NaN);
            intersection2 = new Vector3(float.NaN, 0.0f, float.NaN);
            return 0;
        }
        else if (dist < Mathf.Abs(radius0 - radius1))
        {
            // No solutions, one circle contains the other.
            intersection1 = new Vector3(float.NaN, 0.0f, float.NaN);
            intersection2 = new Vector3(float.NaN, 0.0f, float.NaN);
            return 0;
        }
        else if ((dist == 0) && (radius0 == radius1))
        {
            // No solutions, the circles coincide.
            intersection1 = new Vector3(float.NaN, 0.0f, float.NaN);
            intersection2 = new Vector3(float.NaN, 0.0f, float.NaN);
            return 0;
        }
        else
        {
            // Find a and h.
            float a = (radius0 * radius0 - radius1 * radius1 + dist * dist) / (2 * dist);
            float h = Mathf.Sqrt(radius0 * radius0 - a * a);

            // Find P2.
            float cx2 = cx0 + a * (cx1 - cx0) / dist;
            float cz2 = cz0 + a * (cz1 - cz0) / dist;

            // Get the points P3.
            intersection1 = new Vector3(cx2 + h * (cz1 - cz0) / dist, 0.0f, cz2 - h * (cx1 - cx0) / dist);
            intersection2 = new Vector3(cx2 - h * (cz1 - cz0) / dist, 0.0f, cz2 + h * (cx1 - cx0) / dist);

            // See if we have 1 or 2 solutions.
            if (dist == radius0 + radius1) return 1;
            return 2;
        }
    }

    public static Vector3 Rotate(Vector3 vector, float yAngle)
    {
        float angleRad = yAngle * Mathf.Deg2Rad;
        float cosAngle = Mathf.Cos(angleRad);
        float sinAngle = Mathf.Sin(angleRad);

        float xResult = (cosAngle * vector.x) + (sinAngle * vector.z);
        float zResult = (cosAngle * vector.z) - (sinAngle * vector.x);

        return new Vector3(xResult, vector.y, zResult);
    }

    public static Vector3 RotatePlus90(Vector3 vector)
    {
        return new Vector3(vector.z, vector.y, -vector.x);
    }

    public static Vector3 RotateMinus90(Vector3 vector)
    {
        return new Vector3(-vector.z, vector.y, vector.x);
    }

    public static float WrapAngle(float angle)
    {
        if (angle <= -180.0f)
            angle += 360.0f;

        if (angle > 180.0f)
            angle -= 360.0f;

        return angle;
    }

    public static void DrawEllipse(Vector3 pos, Vector3 forward, Vector3 up, float radiusX, float radiusY, int segments, Color color, float duration = 0)
    {
        float angle = 0f;
        Quaternion rot = Quaternion.LookRotation(forward, up);
        Vector3 lastPoint = Vector3.zero;
        Vector3 thisPoint = Vector3.zero;

        for (int i = 0; i < segments + 1; i++)
        {
            thisPoint.x = Mathf.Sin(Mathf.Deg2Rad * angle) * radiusX;
            thisPoint.z = Mathf.Cos(Mathf.Deg2Rad * angle) * radiusY;

            if (i > 0)
            {
                Debug.DrawLine(rot * lastPoint + pos, rot * thisPoint + pos, color, duration);
            }

            lastPoint = thisPoint;
            angle += 360f / segments;
        }
    }

    public static void DrawArc(Vector3 pos, Vector3 forward, Vector3 up, float radiusX, float radiusY, int segments, Color color, float angleStart, float angleDiff, float duration = 0)
    {
        float angle = angleStart;

        Quaternion rot = Quaternion.LookRotation(forward, up);
        Vector3 lastPoint = Vector3.zero;
        Vector3 thisPoint = Vector3.zero;

        for (int i = 0; i < segments + 1; i++)
        {
            thisPoint.x = Mathf.Sin(Mathf.Deg2Rad * angle) * radiusX;
            thisPoint.z = Mathf.Cos(Mathf.Deg2Rad * angle) * radiusY;

            if (i > 0)
            {
                Debug.DrawLine(rot * lastPoint + pos, rot * thisPoint + pos, color, duration);
            }

            lastPoint = thisPoint;
            angle += angleDiff / segments;
        }
    }

    //public float CalcHeight(float fStartH, float fEndH, float fPeakH, float fDuration, float fX)
    //{
    //    //https://www.desmos.com/calculator/pskenhejyw

    //    float fSP = Mathf.Sqrt(Mathf.Abs(fStartH - fPeakH));
    //    float fEP = Mathf.Sqrt(Mathf.Abs(fEndH - fPeakH));

    //    float fT = fSP / (fSP + fEP);
    //    float fH = fDuration * fT;

    //    float fA = (fPeakH - fStartH) / (fH * fH);
    //    return -fA * ((fX - fH) * (fX - fH)) + fPeakH;
    //}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////