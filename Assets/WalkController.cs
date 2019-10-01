using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public class WalkController
{
    //--------------------------------------------------
    // Substructures/classes
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    public struct Point
    {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="point"></param>
        public Point(Point point)
        {
            position = point.position;
            angle = point.angle;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="pos"></param>
        /// <param name="ang"></param>
        public Point(Vector3 pos, float ang)
        {
            position = pos;
            angle = ang;
        }

        /// <summary>
        /// 
        /// </summary>
        public Vector3 position;

        /// <summary>
        /// 
        /// </summary>
        public float angle;
    }

    /// <summary>
    /// 
    /// </summary>
    public class LegData
    {
        /// <summary>
        /// 
        /// </summary>
        public FootMotionRegion region;

        /// <summary>
        /// 
        /// </summary>
        public MotionPoint point;

        /// <summary>
        /// 
        /// </summary>
        public Point gaitPoint;

        /// <summary>
        /// 
        /// </summary>
        public Point gaitError;

        /// <summary>
        /// 
        /// </summary>
        public Point finalPoint;

        /// <summary>
        /// 
        /// </summary>
        public Vector3 debugLastPosition;
    }


    //--------------------------------------------------
    // Variables
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    private Transform _bodyTransform;

    /// <summary>
    /// 
    /// </summary>
    private Transform _balanceTransform;

    /// <summary>
    /// 
    /// </summary>
    private Vector3 _balancePosition;

    /// <summary>
    /// 
    /// </summary>
    private MotionPoint _rootPoint;

    /// <summary>
    /// 
    /// </summary>
    private List<LegData> _legList;

    /// <summary>
    /// 
    /// </summary>
    private GaitGenerator _gaitGenerator;

    /// <summary>
    /// 
    /// </summary>
    private float _minCyclesPerSecond = 0.1f;

    /// <summary>
    /// 
    /// </summary>
    private float _maxCyclesPerSecond = 1.0f;

    /// <summary>
    /// 
    /// </summary>
    private float _heightCorrection = 0.01f;

    /// <summary>
    /// 
    /// </summary>
    private bool _isWalking = false;


    private List<Vector3> _convexStableArea;
    private bool _balanceTransformStable = false;


    //--------------------------------------------------
    // Constructors
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    /// <param name="bodyTransform"></param>
    /// <param name="balanceTransform"></param>
    /// <param name="rootPoint"></param>
    /// <param name="peakHeight"></param>
    /// <param name="minCyclesPerSecond"></param>
    /// <param name="maxCyclesPerSecond"></param>
    /// <param name="heightCorrection"></param>
    public WalkController(Transform bodyTransform, Transform balanceTransform, MotionPoint rootPoint, float minCyclesPerSecond, float maxCyclesPerSecond, float heightCorrection)
    {
        _bodyTransform = bodyTransform;
        _balanceTransform = balanceTransform;
        _balancePosition = Vector3.zero;
        _rootPoint = rootPoint;
        _legList = new List<LegData>();
        FindChildFeet(_bodyTransform, _legList);

        _minCyclesPerSecond = minCyclesPerSecond;
        _maxCyclesPerSecond = maxCyclesPerSecond;
        _heightCorrection = heightCorrection;

        float[] peakHeights = new float[_legList.Count];
        for (int i = 0; i < _legList.Count; i++)
        {
            peakHeights[i] = _legList[i].region.PeakHeight;
        }
        _gaitGenerator = new GaitGenerator(_legList.Count, peakHeights);
        QueueDefaultGait();
    }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    /// <returns></returns>
    public bool IsWalking()
    {
        return _gaitGenerator.IsAnyStriding();
    }

    /// <summary>
    /// 
    /// </summary>
    public void QueueDefaultGait()
    {
        Gait defaultGait = new Gait(_legList.Count);
        for (int i = 0; i < _legList.Count; i++)
        {
            foreach (GaitStrideData stride in _legList[i].region.strides)
            {
                defaultGait.AddStride(i, new GaitStride(stride.start, stride.end));
            }
        }
        _gaitGenerator.QueueGait(defaultGait);
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="stridingLegs"></param>
    public void QueueNUpGait(uint stridingLegs)
    {
        Gait oneUp = new Gait(_legList.Count);
        float start = 0.0f;
        float end = (float)stridingLegs / _legList.Count;
        for (int i = 0; i < _legList.Count; i++)
        {
            oneUp.AddStride(i, new GaitStride(start, end));
            start = end;
            end += (float)stridingLegs / _legList.Count;
            if (end >= 1.0f)
                end -= 1.0f;
        }
        _gaitGenerator.QueueGait(oneUp);
    }

    //https://stackoverflow.com/questions/3120357/get-closest-point-to-a-line
    //https://stackoverflow.com/questions/10983872/distance-from-a-point-to-a-polygon
    private static Vector3 GetClosestPoint(Vector3 a, Vector3 b, Vector3 p)
    {
        Vector3 atp = p - a;
        Vector3 atb = b - a;

        float atbSqr = atb.sqrMagnitude;

        float atp_dot_atb = Vector3.Dot(atp, atb);

        float t = atp_dot_atb / atbSqr;

        if (t < 0)
            return a;
        else if (t > 1)
            return b;
        else
            return new Vector3(a.x + (atb.x * t), 0.0f, a.z + (atb.z * t));
    }

    private class RayData
    {
        public Vector3 origin;
        public Vector3 dir;
        public float maxLengthSqr;
        public float minLengthSqr;
        public float sinTheta;
    }

    private class Edge
    {
        public RayData leftRay;
        public RayData rightRay;
        public float sqrOffset;
        public Edge leftEdge;
        public Edge rightEdge;
    }

    static int SortBySqrOffset(Edge t1, Edge t2)
    {
        return t1.sqrOffset.CompareTo(t2.sqrOffset);
    }

    //https://stackoverflow.com/questions/4279478/largest-circle-inside-a-non-convex-polygon/46867645#46867645
    private static bool FindConvexPolygonDistanceFromRayRayIntersection(List<Vector3> polygon, Vector3 point, Vector3 direction, RayData ray, out Vector3 intersectionPoint, out float maxDistSqr)
    {
        bool found = false;
        maxDistSqr = 0.0f;
        intersectionPoint = Vector3.zero;

        Vector3 intersection;
        if (Util.FindRayRayIntersection(point, direction, ray.origin, ray.dir, out intersection))
        {
            float sqrLength = (intersection - ray.origin).sqrMagnitude;
            if (sqrLength <= ray.maxLengthSqr)
            {
                bool pointInside = true;
                float minDistSqr = float.PositiveInfinity;
                Vector3 minPoint = Vector3.zero;

                for (int j = 0; j < polygon.Count; j++)
                {
                    //Check if the intersection is inside or on the edge of the polygon
                    if (Util.Orientation(polygon[j], polygon[(j + 1) % polygon.Count], intersection) != 1)
                    {
                        Vector3 closestPoint = GetClosestPoint(polygon[j], polygon[(j + 1) % polygon.Count], intersection);
                        if ((intersection - closestPoint).sqrMagnitude < minDistSqr)
                        {
                            minPoint = closestPoint;
                            minDistSqr = (intersection - closestPoint).sqrMagnitude;
                        }
                    }
                    else
                    {
                        pointInside = false;
                        break;
                    }
                }
                if (pointInside && !float.IsPositiveInfinity(minDistSqr))
                {
                    intersectionPoint = intersection;
                    maxDistSqr = minDistSqr;
                    found = true;
                }
            }
        }

        return found;
    }

    private Vector3 CalcLargestCircleInConvexPolygonAlongRay(List<Vector3> polygon, Vector3 point, Vector3 direction, float balanceInset, out float insetCircle)
    {
        insetCircle = 0.0f;
        // There must be at least 3 points 
        if (polygon.Count < 3) return Vector3.zero;

        //Convert the list of hull points into a doubly linked list of edges
        List<Edge> edges = new List<Edge>();
        List<Edge> edgesToExamine = new List<Edge>();
        List<RayData> raysToCheck = new List<RayData>();
        {
            Edge prevEdge = null;
            for (int j = 0; j < polygon.Count; j++)
            {
                RayData leftRay = new RayData();
                leftRay.origin = polygon[j];
                leftRay.maxLengthSqr = float.PositiveInfinity;
                leftRay.minLengthSqr = 0;
                leftRay.sinTheta = 0;
                raysToCheck.Add(leftRay);

                Edge newEdge = new Edge();
                newEdge.leftRay = leftRay;
                newEdge.leftEdge = prevEdge;
                newEdge.sqrOffset = float.PositiveInfinity;

                if (prevEdge != null)
                {
                    prevEdge.rightRay = newEdge.leftRay;
                    prevEdge.rightEdge = newEdge;
                    edges.Add(prevEdge);
                }

                prevEdge = newEdge;
            }

            if (prevEdge != null)
            {
                Edge firstEdge = edges[0];
                prevEdge.rightRay = firstEdge.leftRay;
                prevEdge.rightEdge = firstEdge;

                firstEdge.leftEdge = prevEdge;
                edges.Add(prevEdge);
            }
        }

        //Now that there is a chain of edges, calculate the direction of this edge's left Ray
        List<Edge> toRemove = new List<Edge>();
        foreach (Edge edge in edges)
        {
            Vector3 normalisedLeftEdge = (edge.leftEdge.leftRay.origin - edge.leftRay.origin).normalized;
            Vector3 normalisedEdge = (edge.rightRay.origin - edge.leftRay.origin).normalized;
            edge.leftRay.dir = normalisedLeftEdge + normalisedEdge;
            edge.leftRay.sinTheta = Mathf.Abs(Util.SignedCrossProductXZ(normalisedEdge, edge.leftRay.dir)) / edge.leftRay.dir.magnitude;

            if (edge.leftRay.dir.sqrMagnitude > 0.00000001f)
            {
                edgesToExamine.Add(edge);
            }
            else
            {
                Edge left = edge.leftEdge;
                Edge right = edge.rightEdge;

                left.rightRay = edge.rightRay;
                left.rightEdge = edge.rightEdge;
                right.leftEdge = left;

                toRemove.Add(edge);
            }
        }
        foreach (Edge edge in toRemove)
        {
            edge.leftRay = null;
            edge.leftEdge = null;

            edge.rightRay = null;
            edge.rightEdge = null;
            edges.Remove(edge);
        }

        while (edgesToExamine.Count > 0)
        {
            foreach (Edge edge in edgesToExamine)
            {
                //Find where the two corner ray's of this edge intersect
                Vector3 intersection;
                if (Util.FindRayRayIntersection(edge.leftRay.origin, edge.leftRay.dir, edge.rightRay.origin, edge.rightRay.dir, out intersection))
                {
                    //Calculate the square of how far the intersection point is away from the this edge
                    edge.sqrOffset = (intersection - GetClosestPoint(edge.leftRay.origin, edge.rightRay.origin, intersection)).sqrMagnitude;

                    //Record the square of how far the intersection point is from each ray's origin, but only if the value is smaller than what may have previously been calculated by another edge
                    edge.leftRay.maxLengthSqr = Mathf.Min((intersection - edge.leftRay.origin).sqrMagnitude, edge.leftRay.maxLengthSqr);
                    edge.rightRay.maxLengthSqr = Mathf.Min((intersection - edge.rightRay.origin).sqrMagnitude, edge.rightRay.maxLengthSqr);
                }
            }

            //All edges have been examined
            edgesToExamine.Clear();

            //If the polygon has more sides than a triangle
            if (edges.Count > 3)
            {
                //Sort the edges by their square offset, with the smallest offset being first
                edges.Sort(SortBySqrOffset);

                //Get a pointer to the first edge in the sorted list and its two neighbours
                Edge mid = edges[0];
                Edge left = mid.leftEdge;
                Edge right = mid.rightEdge;

                //Treating the left and right edges as rays, find where they intersect
                Vector3 intersection;
                if (!Util.FindRayRayIntersection(left.leftRay.origin, mid.leftRay.origin - left.leftRay.origin, right.rightRay.origin, mid.rightRay.origin - right.rightRay.origin, out intersection) || intersection.sqrMagnitude > 1000000000.0f)
                {
                    //The two edges are parallel so set the ray origin to be equidistant
                    intersection = (mid.leftRay.origin + mid.rightRay.origin) / 2.0f;
                }

                RayData betweenRay = new RayData();
                betweenRay.origin = intersection;

                Vector3 normalisedLeftEdge = (left.leftRay.origin - mid.leftRay.origin).normalized;
                Vector3 normalisedRightEdge = (right.rightRay.origin - mid.rightRay.origin).normalized;
                betweenRay.dir = normalisedLeftEdge + normalisedRightEdge;
                betweenRay.sinTheta = Mathf.Abs(Util.SignedCrossProductXZ(normalisedLeftEdge, betweenRay.dir)) / betweenRay.dir.magnitude;

                Vector3 intersection2;
                Util.FindRayRayIntersection(mid.leftRay.origin, mid.leftRay.dir, mid.rightRay.origin, mid.rightRay.dir, out intersection2);

                betweenRay.minLengthSqr = (intersection2 - intersection).sqrMagnitude;
                betweenRay.maxLengthSqr = float.PositiveInfinity;
                raysToCheck.Add(betweenRay);

                Vector3[] triPoints = new Vector3[3];
                triPoints[0] = mid.leftRay.origin + new Vector3(0, 0.01f, 0) * edges.Count;
                triPoints[1] = betweenRay.origin + new Vector3(0, 0.01f, 0) * edges.Count;
                triPoints[2] = mid.rightRay.origin + new Vector3(0, 0.01f, 0) * edges.Count;

                //Handles.color = new Color(1.0f, 1.0f, 0.0f, 0.25f);
                //Handles.DrawAAConvexPolygon(triPoints);

                left.rightRay = betweenRay;
                left.rightEdge = right;
                edgesToExamine.Add(left);

                right.leftRay = betweenRay;
                right.leftEdge = left;
                edgesToExamine.Add(right);

                mid.leftRay = null;
                mid.leftEdge = null;

                mid.rightRay = null;
                mid.rightEdge = null;
                edges.RemoveAt(0);
            }
            else
            {
                break;
            }
        }

        //This finds the best position
        float bestRadSqr = 0.0f;
        Vector3 bestIntersect = Vector3.zero;
        if (edges.Count == 3)
        {
            foreach (Edge edge in edges)
            {
                Vector3 intersection2 = Vector3.zero;
                if (Util.FindRayRayIntersection(edge.leftRay.origin, edge.leftRay.dir, edge.rightRay.origin, edge.rightRay.dir, out intersection2))
                {
                    float radSqr = float.PositiveInfinity;
                    for (int j = 0; j < polygon.Count; j++)
                    {
                        Vector3 pt = GetClosestPoint(polygon[j], polygon[(j + 1) % polygon.Count], intersection2);
                        radSqr = Mathf.Min((intersection2 - pt).sqrMagnitude, radSqr);
                    }

                    if (radSqr >= bestRadSqr)
                    {
                        bestIntersect = intersection2;
                        bestRadSqr = radSqr;
                    }
                }
            }

            //float bestRad = Mathf.Sqrt(bestRadSqr);
            //Debug.DrawLine(_bodyTransform.TransformPoint(bestIntersect), _bodyTransform.TransformPoint(point), Color.yellow);
            //Util.DrawEllipseDashed(_bodyTransform.TransformPoint(bestIntersect), Vector3.forward, Vector3.up, bestRad, bestRad, 64, Color.blue, true);
        }

        Vector3 maxIntersect = Vector3.zero;
        Vector3 nearestIntersect = Vector3.zero;
        {
            int i = 0;
            float maxDistToNearestEdgeSqr = float.NegativeInfinity;
            float minRayLengthSqr = float.PositiveInfinity;
            float minInset = Mathf.Sqrt(bestRadSqr) * balanceInset;
            foreach (RayData ray in raysToCheck)
            {
                Debug.DrawRay(_bodyTransform.TransformPoint(ray.origin /*+ new Vector3(0, 0.01f, 0) * i*/ + ray.dir.normalized * Mathf.Sqrt(ray.minLengthSqr)), _bodyTransform.TransformDirection(ray.dir.normalized * (Mathf.Sqrt(ray.maxLengthSqr) - Mathf.Sqrt(ray.minLengthSqr))), Color.white);

                Vector3 intersect;
                float distToNearestEdgeSqr;
                if (FindConvexPolygonDistanceFromRayRayIntersection(polygon, point, direction, ray, out intersect, out distToNearestEdgeSqr))
                {
                    float rayLengthSqr = (intersect - point).sqrMagnitude;

                    if (rayLengthSqr < minRayLengthSqr)
                    {
                        if (distToNearestEdgeSqr >= minInset * minInset)
                        {
                            maxIntersect = intersect;

                            if (Mathf.Sqrt(rayLengthSqr) < (Mathf.Sqrt(bestRadSqr) * (1.0f - balanceInset)) - (Mathf.Sqrt(bestRadSqr) - Mathf.Sqrt(distToNearestEdgeSqr)))
                                nearestIntersect = point;
                            else
                                nearestIntersect = intersect - (intersect - point).normalized * (Mathf.Sqrt(distToNearestEdgeSqr) - minInset);// intersect;

                            maxDistToNearestEdgeSqr = distToNearestEdgeSqr;
                            minRayLengthSqr = rayLengthSqr;
                        }
                        else if (ray.sinTheta > 0.0f)
                        {
                            float hypot = minInset / ray.sinTheta;
                            float hypotSqr = hypot * hypot;
                            if ((hypotSqr <= ray.maxLengthSqr) && (hypotSqr >= ray.minLengthSqr))
                            {
                                maxIntersect = nearestIntersect = ray.dir.normalized * hypot + ray.origin;
                                maxDistToNearestEdgeSqr = minInset * minInset;
                                minRayLengthSqr = (maxIntersect - point).sqrMagnitude;
                            }
                        }
                    }
                }
                i++;
            }

            if (float.IsNegativeInfinity(maxDistToNearestEdgeSqr))
            {
                maxDistToNearestEdgeSqr = bestRadSqr;
                nearestIntersect = bestIntersect;
            }

            float radius = Mathf.Sqrt(maxDistToNearestEdgeSqr);
            Debug.DrawLine(_bodyTransform.TransformPoint(maxIntersect), _bodyTransform.TransformPoint(nearestIntersect), Color.yellow);
            Debug.DrawLine(_bodyTransform.TransformPoint(nearestIntersect), _bodyTransform.TransformPoint(point), Color.cyan);
            Util.DrawEllipseDashed(_bodyTransform.TransformPoint(maxIntersect), Vector3.forward, Vector3.up, radius, radius, 64, Color.magenta, false);

            if (minInset < radius)
            {
                Util.DrawEllipseDashed(_bodyTransform.TransformPoint(maxIntersect), Vector3.forward, Vector3.up, radius - minInset, radius - minInset, 64, Color.yellow, true);
                insetCircle = radius - minInset;
            }
        }

        //Clean up cyclic links
        foreach (Edge edge in edges)
        {
            edge.leftEdge = null;
            edge.rightEdge = null;
            edge.leftRay = null;
            edge.rightRay = null;
        }

        return nearestIntersect;
    }

    /// <summary>
    /// 
    /// </summary>
    public void Render()
    {
        List<Vector3> stancedPointsIn = new List<Vector3>();
        int i = 0;
        foreach (LegData leg in _legList)
        {
            if (!_gaitGenerator.IsStriding(i))
            {
                stancedPointsIn.Add(Util.FlattenY(leg.finalPoint.position));
                foreach (Vector2 vertex in leg.region.Shape)
                {
                    stancedPointsIn.Add(Util.FlattenY(leg.finalPoint.position + Util.Rotate(new Vector3(vertex.x, 0, vertex.y), leg.finalPoint.angle)));
                }
            }
            i++;
        }

        List<Vector3> hullOut;
        if (Util.ConvexHull(stancedPointsIn, out hullOut))
        {
            ////http://demonstrations.wolfram.com/AnEfficientTestForAPointToBeInAConvexPolygon/
            bool balanceTransformStable = true;
            for (int j = 0; j < hullOut.Count; j++)
            {
                if (Util.Orientation(hullOut[j], hullOut[(j + 1) % hullOut.Count], _balancePosition) != 2)
                {
                    balanceTransformStable = false;
                    break;
                }
            }

            {
                Vector3[] pointsOut = new Vector3[hullOut.Count];
                i = 0;
                foreach (Vector3 temp in hullOut)
                {
                    pointsOut[i] = _bodyTransform.TransformPoint(temp);
                    i++;
                }

                if (balanceTransformStable)
                    Handles.color = new Color(0.0f, 1.0f, 0.0f, 0.25f);
                else
                    Handles.color = new Color(1.0f, 0.0f, 0.0f, 0.25f);
                Handles.DrawAAConvexPolygon(pointsOut);
            }


            {
                Vector3[] pointsOut = new Vector3[_convexStableArea.Count];
                i = 0;
                foreach (Vector3 temp in _convexStableArea)
                {
                    pointsOut[i] = _bodyTransform.TransformPoint(temp);
                    i++;
                }

                Handles.color = new Color(0.0f, 0.0f, 1.0f, 0.25f);
                Handles.DrawAAConvexPolygon(pointsOut);
            }

        }

        Vector3 worldFinalPosition = _bodyTransform.TransformPoint(_balancePosition);
        Debug.DrawRay(worldFinalPosition, _bodyTransform.TransformDirection(new Vector3(0.1f, 0, 0.1f)), Color.magenta);
        Debug.DrawRay(worldFinalPosition, _bodyTransform.TransformDirection(new Vector3(-0.1f, 0, 0.1f)), Color.magenta);
        Debug.DrawRay(worldFinalPosition, _bodyTransform.TransformDirection(new Vector3(0.1f, 0, -0.1f)), Color.magenta);
        Debug.DrawRay(worldFinalPosition, _bodyTransform.TransformDirection(new Vector3(-0.1f, 0, -0.1f)), Color.magenta);
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="futureRoot"></param>
    /// <param name="bodyMotion"></param>
    /// <param name="heightCorrectionCalc"></param>
    public void Update(MotionPoint futureRoot, Motion bodyMotion, out float heightCorrectionCalc, out float speedMultiplier, out float cyclesPerSecond)
    {
        heightCorrectionCalc = 0.0f;
        speedMultiplier = 1.0f;

        //Update the position of the leg points based on any animations that may have happened
        UpdateLegPoints();

        //Now check if
        if (ShouldStartWalking())
        {
            if (!_isWalking)
                _balanceTransformStable = false;

            _isWalking = true;
        }

        //TODO need to update below (as some point post conference) to account for chaining (centipede) causing rear segments to have completely different motions than the root
        //All the compute position stuff seems to be geared up for it, in that each leg is handled with its own motion, but we currently don't use chained motion points
        //May want to have a way in unity scene to define the hierachy, then MotionPoint baking "should" take care of the rest.

        //Calculate what the motion for each leg is as well as their speeds and other path factors, based on if they had been moved to the future root
        Motion[] motions; float[] pointSpeeds; float minPathTime, maxPointSpeed;
        AllLegPathFactors(futureRoot, out motions, out pointSpeeds, out minPathTime, out maxPointSpeed);

        //--------------------------------------------------

        cyclesPerSecond = 0.0f;
        Point[] updatedGaitPoints = new Point[_legList.Count];
        if (_isWalking)
        {
            if (_balanceTransformStable)
            {
                _gaitGenerator.CheckForNewStrides(_legList);
            }

            if (!float.IsPositiveInfinity(minPathTime))
            {
                cyclesPerSecond = _gaitGenerator.CalcCyclesPerSecond(minPathTime, _minCyclesPerSecond, _maxCyclesPerSecond, pointSpeeds, maxPointSpeed);

                //Clamp the cycles per second, adjusting other values accordingly
                if (cyclesPerSecond > _maxCyclesPerSecond)
                {
                    //Scale down the body speed to keep the maximum cycles per second
                    speedMultiplier *= _maxCyclesPerSecond / cyclesPerSecond;
                    cyclesPerSecond = _maxCyclesPerSecond;
                }
                else if (cyclesPerSecond < _minCyclesPerSecond)
                {
                    //Scale down the path to keep the minimum cycles per second
                    minPathTime *= cyclesPerSecond / _minCyclesPerSecond;
                    cyclesPerSecond = _minCyclesPerSecond;
                }
            }
            else
            {
                cyclesPerSecond = _minCyclesPerSecond;
            }
            Debug.Log("CPS = " + cyclesPerSecond.ToString() + ", Slowdown = " + speedMultiplier.ToString());

            _gaitGenerator.ComputePreUpdatePositions(_legList, motions, minPathTime, pointSpeeds, _bodyTransform, cyclesPerSecond * Time.deltaTime, updatedGaitPoints);
            _gaitGenerator.Update(cyclesPerSecond * Time.deltaTime);

            //--------------------------------------------------

            heightCorrectionCalc = _gaitGenerator.MeanStancedHeight() * -_heightCorrection;
            float displacementCorrection = 0.0f;
            if (minPathTime > 0.0f)
                displacementCorrection = (-Time.deltaTime * speedMultiplier) / minPathTime;

            _gaitGenerator.ApplyOffset(heightCorrectionCalc, displacementCorrection);

            for (int i = 0; i < _legList.Count; i++)
            {
                motions[i] = motions[i].Scaled(speedMultiplier);
            }
        }
        else
        {
            for (int i = 0; i < _legList.Count; i++)
            {
                LegData leg = _legList[i];
                updatedGaitPoints[i] = new Point(leg.gaitPoint);
            }
        }

        //Apply the inverse of the body motion to the legs in contact with the ground
        ApplyInverseMotionToStancedLegs(motions, heightCorrectionCalc);

        //Compute the final points of all the legs
        _gaitGenerator.ComputePositions(_legList, motions, minPathTime, pointSpeeds, _bodyTransform, cyclesPerSecond * Time.deltaTime, updatedGaitPoints);

        //Apply the final leg points to the targets so that IK can align to them
        UpdateLegTargets();
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="debugTrailDuration"></param>
    public void DebugTrace(float debugTrailDuration)
    {
        for (int i = 0; i < _legList.Count; i++)
        {
            Vector3 debugPosition = _bodyTransform.TransformPoint(_legList[i].finalPoint.position);
            Debug.DrawLine(_legList[i].debugLastPosition, debugPosition, _legList[i].region.DbgCol, debugTrailDuration);
            _legList[i].debugLastPosition = debugPosition;
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public void CheckForStridesComplete()
    {
        _gaitGenerator.CheckForStridesComplete(_legList, _bodyTransform);

        if (_isWalking && CanStopWalking())
        {
            _isWalking = false;
            _gaitGenerator.ResetDisplacements();
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="parent"></param>
    /// <param name="legList"></param>
    private void FindChildFeet(Transform parent, List<LegData> legList)
    {
        foreach (Transform child in parent)
        {
            FootMotionRegion region = child.GetComponent<FootMotionRegion>();
            if (region != null)
            {
                MotionPoint initialPoint = MotionPoint.FromTransform(_bodyTransform, region.transform, _rootPoint);
                LegData stuff = new LegData
                {
                    region = region,

                    point = initialPoint,
                    debugLastPosition = region.transform.position + new Vector3(0.0f, 0.0f, 0.0f),// stuff.channel.Height, stuff.channel.Offset);

                    gaitPoint = new Point(initialPoint.Position, initialPoint.Angle),
                    gaitError = new Point(Vector3.zero, 0.0f),
                    finalPoint = new Point(initialPoint.Position, initialPoint.Angle),
                };

                legList.Add(stuff);
            }
            else
            {
                //The child isn't anything so look at its children
                FindChildFeet(child, legList);
            }
        }
    }

    /// <summary>
    /// 
    /// </summary>
    private void UpdateLegPoints()
    {
        foreach (LegData leg in _legList)
        {
            //Create a copy of the current leg's point with root point's motion applied to it
            leg.point = MotionPoint.FromTransform(_bodyTransform, leg.region.transform, _rootPoint);
        }
    }

    /// <summary>
    /// 
    /// </summary>
    private void UpdateLegTargets()
    {
        foreach (LegData leg in _legList)
        {
            leg.region.UpdateTarget(leg.finalPoint.position, leg.finalPoint.angle, _bodyTransform);
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="futureRoot"></param>
    /// <param name="motions"></param>
    /// <param name="pointSpeeds"></param>
    /// <param name="minPathTime"></param>
    /// <param name="maxPointSpeed"></param>
    private void AllLegPathFactors(MotionPoint futureRoot, out Motion[] motions, out float[] pointSpeeds, out float minPathTime, out float maxPointSpeed)
    {
        motions = new Motion[_legList.Count];
        pointSpeeds = new float[_legList.Count];

        minPathTime = float.PositiveInfinity;
        maxPointSpeed = 0.0f;

        //Compute the individual foot point motions as a result of applying the previous motion to the root
        for (int i = 0; i < _legList.Count; i++)
        {
            LegData leg = _legList[i];

            //Compute the motion that would have resulted in the leg's point moving to the future point
            MotionPoint futurePoint = leg.point.RelativeTo(futureRoot);
            motions[i] = Motion.Between(leg.point, futurePoint);

            minPathTime = Mathf.Min(minPathTime, leg.region.BoundedPathFactors(leg.point, motions[i], _gaitGenerator.MaxStanceRatio(i), out pointSpeeds[i], _bodyTransform));
            maxPointSpeed = Mathf.Max(maxPointSpeed, pointSpeeds[i]);
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="legMotions"></param>
    /// <param name="heightDiff"></param>
    private void ApplyInverseMotionToStancedLegs(Motion[] legMotions, float heightDiff)
    {
        for (int i = 0; i < _legList.Count; i++)
        {
            if (legMotions[i].HasRotation())
            {
                float rotationAmount = legMotions[i].AngularVelocity;
                Vector3 localRotOrigin = legMotions[i].RotationOrigin;

                if (!_gaitGenerator.IsStriding(i))
                {
                    LegData leg = _legList[i];
                    leg.finalPoint.position = Util.Rotate(leg.finalPoint.position - localRotOrigin, -rotationAmount) + localRotOrigin;
                    leg.finalPoint.position.y += heightDiff;
                    leg.finalPoint.angle -= rotationAmount;
                }
            }
            else if (legMotions[i].HasTranslation())
            {
                if (!_gaitGenerator.IsStriding(i))
                {
                    LegData leg = _legList[i];
                    leg.finalPoint.position -= legMotions[i].LinearVelocity;
                    leg.finalPoint.position.y += heightDiff;
                }
            }
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <returns></returns>
    private bool ShouldStartWalking()
    {
        bool anyPointsOutOfRange = false;
        for (int i = 0; i < _legList.Count; i++)
        {
            if (_gaitGenerator.IsStriding(i))
            {
                anyPointsOutOfRange = true;
                break;
            }
            else
            {
                LegData leg = _legList[i];

                Vector3 flattened = Util.FlattenY(leg.finalPoint.position);
                float distanceSq = (flattened - leg.point.Position).sqrMagnitude;

                //Is this leg now outside of its start area?
                if (distanceSq > leg.region.StartRadius * leg.region.StartRadius)
                {
                    anyPointsOutOfRange = true;
                    break;
                }

                if (Mathf.Abs(leg.finalPoint.angle - leg.point.Angle) > 45.0f)
                {
                    anyPointsOutOfRange = true;
                    break;
                }
            }
        }
        return anyPointsOutOfRange;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <returns></returns>
    private bool CanStopWalking()
    {
        bool allPointsInRange = true;
        for (int i = 0; i < _legList.Count; i++)
        {
            if (_gaitGenerator.IsStriding(i))
            {
                allPointsInRange = false;
                break;
            }
            else
            {
                LegData leg = _legList[i];

                Vector3 flattened = Util.FlattenY(leg.finalPoint.position);
                float distanceSq = (flattened - leg.point.Position).sqrMagnitude;

                //Is this leg still outside its stop area?
                if (distanceSq >= leg.region.StopRadius * leg.region.StopRadius)
                {
                    allPointsInRange = false;
                    break;
                }

                if (Mathf.Abs(leg.finalPoint.angle - leg.point.Angle) > 0.1f)
                {
                    allPointsInRange = false;
                    break;
                }
            }
        }
        return allPointsInRange;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="cycleTimestep"></param>
    public void Balance(float cycleTimestep, float balanceInset)
    {
        List<bool> stableFutureLegs;
        _gaitGenerator.StableFutureLegs(out stableFutureLegs);


        List<Vector3> stancedPointsIn = new List<Vector3>();
        int i = 0;
        foreach (LegData leg in _legList)
        {
            leg.point.Render(_bodyTransform);
            if ((!_isWalking || stableFutureLegs[i]) && leg.region.gameObject.activeSelf)
            //if (leg.region.gameObject.activeSelf)
            {
                stancedPointsIn.Add(Util.FlattenY(leg.finalPoint.position));
                //stancedPointsIn.Add(Util.FlattenY(leg.point.Position));
                foreach (Vector2 vertex in leg.region.Shape)
                {
                    stancedPointsIn.Add(Util.FlattenY(leg.finalPoint.position + Util.Rotate(new Vector3(vertex.x, 0, vertex.y), leg.finalPoint.angle)));
                    //stancedPointsIn.Add(Util.FlattenY(leg.point.Position + Util.Rotate(new Vector3(vertex.x, 0, vertex.y), leg.point.Angle)));
                }
            }
            i++;
        }

        if (Util.ConvexHull(stancedPointsIn, out _convexStableArea))
        {
            //http://demonstrations.wolfram.com/AnEfficientTestForAPointToBeInAConvexPolygon/
            Vector3 origin = Vector3.zero;
            _balanceTransformStable = true;
            for (int j = 0; j < _convexStableArea.Count; j++)
            {
                if (Util.Orientation(_convexStableArea[j], _convexStableArea[(j + 1) % _convexStableArea.Count], _balancePosition) != 2)
                {
                    _balanceTransformStable = false;
                    break;
                }
            }


            Vector3 minPoint = Vector3.zero;
            Vector3 minEdgeNormal = Vector3.zero;
            float minDist = float.PositiveInfinity;
            for (int j = 0; j < _convexStableArea.Count; j++)
            {
                Vector3 point = GetClosestPoint(_convexStableArea[j], _convexStableArea[(j + 1) % _convexStableArea.Count], origin);
                if ((origin - point).sqrMagnitude < minDist)
                {
                    minPoint = point;
                    minEdgeNormal = Util.RotateMinus90(_convexStableArea[j] - _convexStableArea[(j + 1) % _convexStableArea.Count]);
                    minDist = (origin - point).sqrMagnitude;
                }
            }
            if (!float.IsPositiveInfinity(minDist))
            {
                Debug.DrawLine(_bodyTransform.TransformPoint(origin), _bodyTransform.TransformPoint(minPoint), Color.cyan);
                Vector3 direction = minPoint - origin;

                //If the origin is on the minPoint, use the inward facing normal of the edge the min point was on
                if (Mathf.Abs(direction.sqrMagnitude) < 0.000001f)
                    direction = minEdgeNormal;

                float insetCircle;
                minPoint = CalcLargestCircleInConvexPolygonAlongRay(_convexStableArea, origin, direction, balanceInset, out insetCircle);

                //if((_balancePosition - origin).sqrMagnitude < 0.01f * 0.01f)
                //{
                //    _balanceTransformStable = true;
                //}
            }


            List<int> legsStridingNext;
            float timeRemaining = _gaitGenerator.CycleTimeUntilNextStride(out legsStridingNext);

            Vector3 worldFinalPosition = _bodyTransform.TransformPoint(minPoint);
            Debug.DrawRay(worldFinalPosition, _bodyTransform.TransformDirection(new Vector3(0.1f, 0, 0.1f)), Color.white);
            Debug.DrawRay(worldFinalPosition, _bodyTransform.TransformDirection(new Vector3(-0.1f, 0, 0.1f)), Color.white);
            Debug.DrawRay(worldFinalPosition, _bodyTransform.TransformDirection(new Vector3(0.1f, 0, -0.1f)), Color.white);
            Debug.DrawRay(worldFinalPosition, _bodyTransform.TransformDirection(new Vector3(-0.1f, 0, -0.1f)), Color.white);


            if (timeRemaining > 0.0f)
            {
                Vector3 balanceError = Util.FlattenY(minPoint - _balancePosition);
                float spd = Mathf.Min(balanceError.magnitude * (cycleTimestep / timeRemaining), 0.05f);
                _balancePosition = Vector3.MoveTowards(_balancePosition, minPoint, spd);
            }
            else
            {
                _balancePosition = Vector3.MoveTowards(_balancePosition, minPoint, 0.005f);
            }



            Transform body = _bodyTransform.Find("Body");
            if (body != null)
            {
                body.localPosition = new Vector3(_balancePosition.x, body.localPosition.y, _balancePosition.z);
            }
        }
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////