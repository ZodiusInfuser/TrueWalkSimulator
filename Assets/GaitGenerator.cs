using System.Collections.Generic;
using UnityEngine;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// When provided with a gait, this class generates the offsets over time necessary for a set of legs to move achieve walking
/// </summary>
public class GaitGenerator
{
    //--------------------------------------------------
    // Variables
    //--------------------------------------------------

    /// <summary>
    /// An array of gait channels used to calculate the offsets used for walking
    /// </summary>
    private GaitChannel[] _channels;

    /// <summary>
    /// The current cycle time
    /// </summary>
    private float _cycleTime;

    /// <summary>
    /// The current gait definition that is loaded into the gait channels
    /// </summary>
    private Gait _currentGait;

    /// <summary>
    /// A queued gait definition that will replace what is in the gait channels at the next available opportunity
    /// </summary>
    private Gait _queuedGait;

    /// <summary>
    /// A boolean for indicating if any channels have active strides
    /// </summary>
    private bool _anyChannelsStriding;

    /// <summary>
    /// The longest stance of all stances
    /// </summary>
    private float _maxOfAllStances;

    /// <summary>
    /// The current cycle time that the generator is overrunning by, if at all
    /// </summary>
    private float _fOverrun;


    //--------------------------------------------------
    // Constructors
    //--------------------------------------------------

    /// <summary>
    /// Creates a new generator with support for the provided number of legs, and peak heights for each of those legs
    /// </summary>
    /// <param name="supportedLegs">The number of legs to support</param>
    /// <param name="peakHeight">An array of equal size to the number of supported legs, which the peak stride height of each</param>
    public GaitGenerator(int supportedLegs, float[] peakHeights)
    {
        _anyChannelsStriding = false;

        _channels = new GaitChannel[supportedLegs];
        for (int i = 0; i < _channels.Length; i++)
        {
            _channels[i] = new GaitChannel(peakHeights[i]);
        }

        _maxOfAllStances = 1.0f;
        _fOverrun = 0.0f;
        _cycleTime = 0.0f;
    }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// Sets the peak heights of all the supported legs to new values
    /// </summary>
    /// <param name="peakHeight">An array of equal size to the number of supported legs, which the peak stride height of each</param>
    public void SetPeakHeights(float[] peakHeights)
    {
        for (int i = 0; i < _channels.Length; i++)
        {
            _channels[i].PeakHeight = Mathf.Max(peakHeights[i], 0.0f);
        }
    }

    /// <summary>
    /// Checks if any legs are currently striding
    /// </summary>
    /// <returns>True if any are, false otherwise</returns>
    public bool IsAnyStriding()
    {
        return _anyChannelsStriding;
    }

    /// <summary>
    /// Queue up another gait for this generator to use.
    /// Only gets queued if it is for the number of supported legs, and if it is not the gait currently running
    /// </summary>
    /// <param name="newGait">The gait to use next</param>
    public void QueueGait(Gait newGait)
    {
        if (newGait.LegCount == _channels.Length)
        {
            if (newGait != _currentGait)
                _queuedGait = newGait;
        }

        if (!_anyChannelsStriding)
        {
            ApplyQueuedGait();
        }
    }

    /// <summary>
    /// Applies a queued gait if there is one
    /// </summary>
    private void ApplyQueuedGait()
    {
        if (_queuedGait != null)
        {
            for (int i = 0; i < _channels.Length; i++)
            {
                _channels[i].AssignStrides(_queuedGait.GetStrides(i), _cycleTime);
            }
            _currentGait = _queuedGait;
            _queuedGait = null;

            _maxOfAllStances = _currentGait.MaxStance();
        }
    }

    /// <summary>
    /// Returns the ratio of the selected channel's max stance length compared to the maximum of all channels
    /// </summary>
    /// <param name="index">The channel to get the max stance of</param>
    /// <returns>The percentage of that stance to the maximum of all stances</returns>
    public float MaxStanceRatio(int index)
    {
        return _channels[index].LongestStance / _maxOfAllStances;
    }

    /// <summary>
    /// Checks all of the legs to see which have completed their strides. Uses raycasts against collision objects to perform these checks
    /// </summary>
    /// <param name="legs">The final positions of all legs</param>
    /// <param name="transform">The transform to use to convert the local leg positions in to world positions</param>
    public void CheckForStridesComplete(List<WalkController.LegData> legs, Transform transform)
    {
        _anyChannelsStriding = false;

        _fOverrun = 0.0f;
        for (int i = 0; i < legs.Count; i++)
        {
            WalkController.LegData leg = legs[i];

            bool bEnded = false;
            _fOverrun = Mathf.Max(_channels[i].CheckForStrideComplete(transform.TransformPoint(leg.finalPoint.position), out bEnded), _fOverrun);
            if (bEnded)
            {
                leg.gaitError = new WalkController.Point(Vector3.zero, 0.0f);
            }
            else if (_channels[i].IsStriding)
            {
                _anyChannelsStriding = true;
            }
        }

        if (!_anyChannelsStriding)
        {
            ApplyQueuedGait();
        }
    }

    /// <summary>
    /// Checks all of the legs to see which should start their next strides
    /// </summary>
    /// <param name="legs">The final points and gait points of all legs (sets gaitError)</param>
    public void CheckForNewStrides(List<WalkController.LegData> legs)
    {
        //If there is a queued gait then we want the current gait to end completely first,
        //And we don't want new strides to start if any are overrunning
        if ((_queuedGait == null) && (_fOverrun == 0.0f))
        {
            for (int i = 0; i < _channels.Length; i++)
            {
                WalkController.LegData leg = legs[i];

                //Attempt to start the next stride
                if (_channels[i].AttemptNextStride(_cycleTime))
                {
                    leg.gaitError.position = Util.FlattenY(leg.finalPoint.position - leg.gaitPoint.position);
                    leg.gaitError.angle = leg.finalPoint.angle - leg.gaitPoint.angle;
                }
            }
        }
    }

    /// <summary>
    /// Given the intended cycle timestep for this generator, calculate the cycles per second, talking into account min and max values, and the speeds of the leg points
    /// </summary>
    /// <param name="cycleTimestep">The intended cycle timstep</param>
    /// <param name="minCyclesPerSecond">The minimum cycles per second</param>
    /// <param name="maxCyclesPerSecond">The maximum cycles per second</param>
    /// <param name="pointSpeeds">An array of the speed of each leg point</param>
    /// <param name="maxPointSpeed">The maximum speed of the leg points</param>
    /// <returns></returns>
    public float CalcCyclesPerSecond(float cycleTimestep, float minCyclesPerSecond, float maxCyclesPerSecond, float[] pointSpeeds, float maxPointSpeed)
    {
        //These values need to be calculated once on stride end and affect the timestep then until the next stride end
        float cyclesPerSecond = 0.0f;
        if (maxPointSpeed > 0.0f)
        {
            for (int i = 0; i < _channels.Length; i++)
            {
                float speedPercent = pointSpeeds[i] / maxPointSpeed;
                if (_channels[i].IsStriding)
                {
                    cyclesPerSecond = Mathf.Max(_channels[i].LongestStance * 0.5f * speedPercent / cycleTimestep, cyclesPerSecond);
                }
                else
                {
                    if (_channels[i].NextAvailableStancePercent > 0.0f)
                        cyclesPerSecond = Mathf.Max(_channels[i].NextAvailableStancePercent * speedPercent / cycleTimestep, cyclesPerSecond);
                    else
                        cyclesPerSecond = maxCyclesPerSecond;
                }
            }
        }
        return cyclesPerSecond;
    }

    /// <summary>
    /// Update this gait generator by the provided cycle timestep.
    /// </summary>
    /// <param name="cycleTimestep"></param>
    public void Update(float cycleTimestep)
    {
        bool cycleLooped = false;

        _cycleTime += cycleTimestep;
        if (_cycleTime >= 1.0f)
        {
            _cycleTime -= 1.0f;
            cycleLooped = true;
        }

        //Update the gait channels by the scaled timestep
        foreach (GaitChannel channel in _channels)
        {
            channel.Update(cycleTimestep);
            if (cycleLooped)
                channel.CycleLooped();
        }
    }

    /// <summary>
    /// Checks if the channel at the specified index is striding
    /// </summary>
    /// <param name="index">The index of the channel to check</param>
    /// <returns></returns>
    public bool IsStriding(int index)
    {
        return _channels[index].IsStriding;
    }

    /// <summary>
    /// Reset the displacement of each channel back to zero, but only if this channel is not striding
    /// </summary>
    public void ResetDisplacements()
    {
        foreach (GaitChannel channel in _channels)
        {
            channel.ResetDisplacement();
        }
    }

    /// <summary>
    /// Apply height and displacement offsets to all channels, if not striding
    /// </summary>
    /// <param name="heightOffset">The height offset to apply</param>
    /// <param name="displacementOffset">The displacement offset to apply</param>
    public void ApplyOffset(float heightOffset, float displacementOffset)
    {
        foreach (GaitChannel channel in _channels)
        {
            channel.ApplyOffsets(heightOffset, displacementOffset * _maxOfAllStances / channel.LongestStance);
        }
    }

    /// <summary>
    /// Calculate the mean height of all the legs that are in stance
    /// </summary>
    /// <returns>The mean height</returns>
    public float MeanStancedHeight()
    {
        int stanced = 0;
        float meanHeight = 0.0f;

        foreach (GaitChannel channel in _channels)
        {
            if (!channel.IsStriding)
            {
                meanHeight += channel.Height;
                stanced++;
            }
        }

        if (stanced > 0)
            meanHeight /= stanced;

        return meanHeight;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="channel"></param>
    /// <param name="point"></param>
    /// <param name="motion"></param>
    /// <param name="pathTime"></param>
    /// <param name="pointSpeed"></param>
    /// <param name="transform"></param>
    /// <returns></returns>
    private static WalkController.Point ComputePointPosition(GaitChannel channel, MotionPoint point, Motion motion, float pathTime, float pointSpeed, Transform transform)
    {
        Vector3 pointPosition = point.Position;
        float pointAngle = point.Angle;

        //TODO This is wrong as it returns a gait position at zero when in fact it should be elsewhere
        if (!float.IsPositiveInfinity(pathTime))
        {
            //Below finds the local ideal position of the stride point, as in where on the line/arc it is at
            if (motion.HasRotation())
            {
                Vector3 origin = motion.RotationOrigin;

                float pathRadius = (pointPosition - origin).magnitude;
                if (pathRadius > 0.0f)
                {
                    float angleRange = (pointSpeed * pathTime) / (pathRadius * Mathf.Deg2Rad);

                    Debug.DrawRay(transform.TransformPoint(origin + Vector3.up * 0.05f), transform.TransformDirection(Util.Rotate(pointPosition - origin, -angleRange)), Color.white);
                    Debug.DrawRay(transform.TransformPoint(origin + Vector3.up * 0.05f), transform.TransformDirection(Util.Rotate(pointPosition - origin, angleRange)), Color.white);

                    Util.DrawArc(transform.TransformPoint(origin + Vector3.up * 0.05f), transform.TransformDirection(pointPosition - origin), transform.TransformDirection(Vector3.up), pathRadius, pathRadius, 32, Color.white, -angleRange, angleRange * 2.0f);


                    //float fLength = pathRadius * (angleRange * Mathf.Deg2Rad);
                    //Debug.DrawRay(transform.TransformPoint(pointPosition + Vector3.up * 0.1f), transform.TransformDirection(Vector3.forward * fLength / 2), regions[i].DbgCol);
                    //Debug.DrawRay(transform.TransformPoint(pointPosition + Vector3.up * 0.1f), transform.TransformDirection(Vector3.back * fLength / 2), regions[i].DbgCol);

                    pointPosition = Util.Rotate(pointPosition - origin, angleRange * Mathf.Sign(motion.AngularVelocity) * channel.Displacement) + origin;
                    pointAngle += angleRange * Mathf.Sign(motion.AngularVelocity) * channel.Displacement;
                }
            }
            else
            {
                float pathLength = pointSpeed * pathTime;

                //TODO see if this is necessary any more now we do the infinity check
                if (!float.IsNaN(pathLength))
                {
                    Vector3 linVelocity = Util.Rotate(motion.LinearVelocity, point.Angle);

                    Vector3 endPos1 = pointPosition + (linVelocity.normalized * -pathLength) + new Vector3(linVelocity.normalized.z, 0.0f, -linVelocity.normalized.x) * pathLength * 0.5f;
                    Vector3 endPos2 = pointPosition + (linVelocity.normalized * -pathLength) - new Vector3(linVelocity.normalized.z, 0.0f, -linVelocity.normalized.x) * pathLength * 0.5f;
                    Debug.DrawLine(transform.TransformPoint(endPos1 + Vector3.up * 0.05f), transform.TransformPoint(endPos2 + Vector3.up * 0.05f), Color.white);

                    Vector3 endPos3 = pointPosition - (linVelocity.normalized * -pathLength) + new Vector3(linVelocity.normalized.z, 0.0f, -linVelocity.normalized.x) * pathLength * 0.5f;
                    Vector3 endPos4 = pointPosition - (linVelocity.normalized * -pathLength) - new Vector3(linVelocity.normalized.z, 0.0f, -linVelocity.normalized.x) * pathLength * 0.5f;
                    Debug.DrawLine(transform.TransformPoint(endPos3 + Vector3.up * 0.05f), transform.TransformPoint(endPos4 + Vector3.up * 0.05f), Color.white);

                    Debug.DrawRay(transform.TransformPoint(pointPosition + Vector3.up * 0.05f), transform.TransformDirection(linVelocity.normalized * pathLength), Color.white);
                    Debug.DrawRay(transform.TransformPoint(pointPosition + Vector3.up * 0.05f), transform.TransformDirection(linVelocity.normalized * -pathLength), Color.white);

                    pointPosition += linVelocity.normalized * pathLength * channel.Displacement;

                    //Debug.DrawRay(transform.TransformPoint(pointPosition + Vector3.up * 0.1f), transform.TransformDirection(Vector3.forward * pathLength / 2), regions[i].DbgCol);
                    //Debug.DrawRay(transform.TransformPoint(pointPosition + Vector3.up * 0.1f), transform.TransformDirection(Vector3.back * pathLength / 2), regions[i].DbgCol);
                }
            }
        }
        pointPosition.y += channel.Height;

        return new WalkController.Point(pointPosition, pointAngle);
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="legs"></param>
    /// <param name="motions"></param>
    /// <param name="minPathTime"></param>
    /// <param name="pointSpeeds"></param>
    /// <param name="transform"></param>
    /// <param name="cycleTimestep"></param>
    /// <param name="gaitPointsOut"></param>
    public void ComputePreUpdatePositions(List<WalkController.LegData> legs, Motion[] motions, float minPathTime, float[] pointSpeeds, Transform transform, float cycleTimestep, WalkController.Point[] gaitPointsOut)
    {
        for (int i = 0; i < legs.Count; i++)
        {
            gaitPointsOut[i] = ComputePointPosition(_channels[i], legs[i].point, motions[i], minPathTime, pointSpeeds[i], transform);
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="legs"></param>
    /// <param name="motions"></param>
    /// <param name="minPathTime"></param>
    /// <param name="pointSpeeds"></param>
    /// <param name="transform"></param>
    /// <param name="cycleTimestep"></param>
    /// <param name="gaitPointsIn"></param>
    public void ComputePositions(List<WalkController.LegData> legs, Motion[] motions, float minPathTime, float[] pointSpeeds, Transform transform, float cycleTimestep, WalkController.Point[] gaitPointsIn)
    {
        for (int i = 0; i < legs.Count; i++)
        {
            WalkController.LegData leg = legs[i];

            //Difference between the previous gait position and where it would be on the new motion but at the current timestep
            leg.gaitError.position += Util.FlattenY(leg.gaitPoint.position - gaitPointsIn[i].position);
            leg.gaitError.angle += leg.gaitPoint.angle - gaitPointsIn[i].angle;

            //The gait position on the new motion path updated by the current timestep
            leg.gaitPoint = ComputePointPosition(_channels[i], leg.point, motions[i], minPathTime, pointSpeeds[i], transform);


            if (_channels[i].IsStriding)
            {
                leg.finalPoint.position = leg.gaitPoint.position + leg.gaitError.position;
                leg.finalPoint.angle = leg.gaitPoint.angle + leg.gaitError.angle;

                Debug.DrawRay(transform.TransformPoint(leg.gaitPoint.position), transform.TransformDirection(leg.gaitError.position), new Color(0.75f, 0.75f, 0.75f));

                float timeRemaining = _channels[i].StrideDurationRemaining();
                if (timeRemaining > 0.0f)
                {
                    float spd = Mathf.Min(leg.gaitError.position.magnitude * (cycleTimestep / timeRemaining), 0.005f);
                    float spdA = Mathf.Min(Mathf.Abs(leg.gaitError.angle) * (cycleTimestep / timeRemaining), 0.5f);

                    leg.gaitError.position = Vector3.MoveTowards(leg.gaitError.position, Vector3.zero, spd);
                    leg.gaitError.angle = Mathf.MoveTowards(leg.gaitError.angle, 0.0f, spdA);
                }
            }
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="legsStridingNext"></param>
    /// <returns></returns>
    public float CycleTimeUntilNextStride(out List<int> legsStridingNext)
    {
        legsStridingNext = new List<int>();
        float timeUntil = float.PositiveInfinity;
        for (int i = 0; i < _channels.Length; i++)
        {
            float time = _channels[i].TimeUntilNextStride(_cycleTime);
            if (time <= timeUntil)
            {
                if (time < timeUntil)
                {
                    timeUntil = time;
                    legsStridingNext.Clear();
                }
                legsStridingNext.Add(i);
            }
        }
        return timeUntil;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="stableLegs"></param>
    public void StableFutureLegs(out List<bool> stableLegs)
    {
        stableLegs = new List<bool>(_channels.Length);

        float timeUntilNextStance = float.PositiveInfinity;
        for (int i = 0; i < _channels.Length; i++)
        {
            timeUntilNextStance = Mathf.Min(_channels[i].TimeUntilStance(_cycleTime), timeUntilNextStance);
        }

        for (int i = 0; i < _channels.Length; i++)
        {
            stableLegs.Add(!_channels[i].IsStriding && (_channels[i].TimeUntilNextStride(_cycleTime) >= timeUntilNextStance));
        }
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////