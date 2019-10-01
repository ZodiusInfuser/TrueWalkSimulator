using UnityEngine;
using System.Collections.Generic;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Static Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public static class Util
{
    //http://csharphelper.com/blog/2014/08/determine-where-two-lines-intersect-in-c/
    public static bool FindRayRayIntersection(Vector3 p3, Vector3 p4, Vector3 p1, Vector3 p2, out Vector3 intersection)
    {
        // Get the segments' parameters.
        float dx12 = p2.x;
        float dz12 = p2.z;
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
        return true;// (t1 >= -0.00001) && (t1 <= 1.00001);// && (t2 >= 0) && (t2 <= 1));
    }

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

    public static float SignedYAngle(Vector3 from, Vector3 to)
    {
        return WrapAngle((Mathf.Atan2(from.z, from.x) - Mathf.Atan2(to.z, to.x)) * Mathf.Rad2Deg);
    }

    public static Vector3 FlattenY(Vector3 vector)
    {
        return new Vector3(vector.x, 0.0f, vector.z);
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

    public static float MapFloat(float val, float minIn, float maxIn, float minOut, float maxOut)
    {
        return (val - minIn) * (maxOut - minOut) / (maxIn - minIn) + minOut;
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

    public static void DrawEllipseDashed(Vector3 pos, Vector3 forward, Vector3 up, float radiusX, float radiusY, int segments, Color color, bool bStartDash = true, float duration = 0)
    {
        float angle = 0f;
        Quaternion rot = Quaternion.LookRotation(forward, up);
        Vector3 lastPoint = Vector3.zero;
        Vector3 thisPoint = Vector3.zero;

        bool bDraw = bStartDash;
        for (int i = 0; i < segments + 1; i++)
        {
            thisPoint.x = Mathf.Sin(Mathf.Deg2Rad * angle) * radiusX;
            thisPoint.z = Mathf.Cos(Mathf.Deg2Rad * angle) * radiusY;

            if (i > 0)
            {
                if (bDraw)
                    Debug.DrawLine(rot * lastPoint + pos, rot * thisPoint + pos, color, duration);
                bDraw = !bDraw;
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

    public static void FindChildrenWithAnimation(Transform parent, List<Animation> animList)
    {
        foreach (Transform child in parent)
        {
            if (child.gameObject.activeSelf)
            {
                Animation anim = child.GetComponent<Animation>();
                if (anim != null)
                    animList.Add(anim);
                else
                    FindChildrenWithAnimation(child, animList);
            }
        }
    }

    public static void CalcRelativeTo(Transform baseTrans, Transform trans, out Vector3 posOut, out Quaternion rotOut)
    {
        Matrix4x4 relative = baseTrans.worldToLocalMatrix * trans.localToWorldMatrix;
        posOut = relative.MultiplyPoint(Vector3.zero);
        rotOut = Quaternion.LookRotation(relative.GetColumn(2), relative.GetColumn(1));
    }

    // To find orientation of ordered triplet (p, q, r). 
    // The function returns following values 
    // 0 --> p, q and r are colinear 
    // 1 --> Clockwise 
    // 2 --> Counterclockwise 
    public static int Orientation(Vector3 p, Vector3 common, Vector3 r)
    {
        Vector3 ptq = common - p;
        Vector3 qtr = r - common;
        float val = (ptq.z * qtr.x) - (ptq.x * qtr.z);

        if (Mathf.Abs(val) < 0.000001f) return 0; // collinear 
        return (val > 0) ? 1 : 2; // clock or counterclock wise 
    }

    public static float SignedCrossProductXZ(Vector3 u, Vector3 v)
    {
        return (u.x * v.z) - (u.z * v.x);
    }

    //From: https://www.geeksforgeeks.org/convex-hull-set-1-jarviss-algorithm-or-wrapping/
    public static bool ConvexHull(List<Vector3> points, out List<Vector3> hull)
    {
        // There must be at least 3 points 
        if (points.Count < 3)
        {
            hull = null;
            return false;
        }

        // Initialize Result 
        hull = new List<Vector3>();

        // Find the leftmost point 
        int l = 0;
        for (int i = 1; i < points.Count; i++)
            if (points[i].x < points[l].x)
                l = i;

        // Start from leftmost point, keep moving  
        // counterclockwise until reach the start point 
        // again. This loop runs O(h) times where h is 
        // number of points in result or output. 
        int p = l, q;
        do
        {
            // Add current point to result 
            hull.Add(points[p]);

            if (hull.Count > points.Count)
            {
                Debug.LogError("Convex Hull Failed");
                return false;
            }

            // Search for a point 'q' such that  
            // orientation(p, x, q) is counterclockwise  
            // for all points 'x'. The idea is to keep  
            // track of last visited most counterclock- 
            // wise point in q. If any point 'i' is more  
            // counterclock-wise than q, then update q. 
            q = (p + 1) % points.Count;

            for (int i = 0; i < points.Count; i++)
            {
                // If i is more counterclockwise than  
                // current q, then update q 
                if (Orientation(points[p], points[i], points[q]) == 2)
                    q = i;
            }

            // Now q is the most counterclockwise with 
            // respect to p. Set p as q for next iteration,  
            // so that q is added to result 'hull' 
            p = q;

        } while (p != l); // While we don't come to first point 

        return true;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////