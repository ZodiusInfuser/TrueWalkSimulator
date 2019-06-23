using System.Collections.Generic;
using UnityEngine;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public class GaitController
{
    //--------------------------------------------------
    // Variables
    //--------------------------------------------------

    private readonly int _supportedLegs;

    private GaitChannel[] _channels;

    private float _cycleTime;
    private float _lastTime;

    private bool _isWalking;

    private Gait _currentGait;
    private Gait _queuedGait;

    private bool _anyChannelsStriding;

    private float _maxOfAllStances;

    private float _fOverrun;

    private bool _bAllPointsIdle;

    public bool IsIdle() { return _bAllPointsIdle; }


    //--------------------------------------------------
    // Constructors
    //--------------------------------------------------

    /// <summary>
    ///// 
    /// </summary>
    public GaitController(int supportedLegs)
    {
        _isWalking = false;
        _anyChannelsStriding = false;

        _supportedLegs = supportedLegs;
        _channels = new GaitChannel[supportedLegs];
        for(int i = 0; i < _channels.Length; i++)
        {
            _channels[i] = new GaitChannel();
        }

        _maxOfAllStances = 1.0f;

        _fOverrun = 0.0f;

        _cycleTime = 0.0f;
        _lastTime = 0.0f;


        _bAllPointsIdle = true;
    }

    public void StartWalking()
    {

    }

    public void StopWalking()
    {

    }

    public bool IsAnyStriding() { return _anyChannelsStriding; }

    public void QueueGait(Gait newGait)
    {
        if (newGait.LegCount <= _supportedLegs)
        {
            if(newGait != _currentGait)
                _queuedGait = newGait;
        }

        if (!_anyChannelsStriding)
        {
            ApplyQueuedGait();
        }
    }

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
    /// Returns the ratio of the selected channel's max stance length compared to the maximum of all channels.
    /// </summary>
    /// <param name="index"></param>
    /// <returns></returns>
    public float MaxStanceRatio(int index)
    {
        return _channels[index].LongestStance / _maxOfAllStances;
    }

    public void CheckForStridesComplete(List<LegStuff> legs, Transform transform)
    {
        _anyChannelsStriding = false;
        _bAllPointsIdle = true;

        _fOverrun = 0.0f;
        for (int i = 0; i < legs.Count; i++)
        {
            LegStuff leg = legs[i];

            bool bEnded = false;
            _fOverrun = Mathf.Max(_channels[i].CheckForStrideComplete(transform.TransformPoint(leg.finalPosition), out bEnded), _fOverrun);
            if (bEnded)
            {
                leg.groundPosition = leg.finalPosition;
                leg.positionDiff = Vector3.zero;
            }
            else if (_channels[i].IsStriding)
            {
                _anyChannelsStriding = true;
            }

            if (!_channels[i].IsStriding)
            {
                Vector3 flattened = leg.groundPosition;
                flattened.y = 0.0f;
                if ((flattened - leg.point.Position).sqrMagnitude > leg.region.IdleRadius * leg.region.IdleRadius)
                    _bAllPointsIdle = false;
            }
        }

        if (!_anyChannelsStriding)
        {
            ApplyQueuedGait();
        }
    }

    public void CheckForNewStrides(List<LegStuff> legs)
    {
        if (_queuedGait == null)
        {
            //Now immediately check to see if any new strides should be started
            for (int i = 0; i < _channels.Length; i++)
            {
                LegStuff leg = legs[i];

                if (_channels[i].AttemptNextStride(_fOverrun, _cycleTime))
                {
                    //Record the difference between where the ground position is and where it is expected to have left the ground
                    leg.positionDiff = leg.groundPosition - leg.gaitPosition;
                }
            }
        }
    }

    public float CalcScaledTimestep(float cycleTimestep, float maxTimestep, float[] pointSpeeds, float maxPointSpeed)
    {
        //These values need to be calculated once on stride end and affect the timestep then until the next stride end
        float scaledTimestep = 0.0f;
        for (int i = 0; i < _channels.Length; i++)
        {
            if (!_channels[i].IsStriding)
            {
                float speedPercent = pointSpeeds[i] / maxPointSpeed;
                if (_channels[i].NextAvailableStancePercent > 0.0f)
                    scaledTimestep = Mathf.Max(Time.deltaTime * _channels[i].NextAvailableStancePercent * speedPercent / cycleTimestep, scaledTimestep);
                else
                    scaledTimestep = maxTimestep;
            }
        }
        return scaledTimestep;
    }

    public void Update(float cycleTimestep)
    {
        bool cycleLooped = false;

        _lastTime = _cycleTime;
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
            if(cycleLooped)
                channel.CycleLooped();
        }
    }

    public bool IsStriding(int index)
    {
        return _channels[index].IsStriding;
    }

    public void ApplyOffset(float heightOffset, float displacementOffset)
    {
        foreach (GaitChannel channel in _channels)
        {
            channel.ApplyOffset(heightOffset, displacementOffset * _maxOfAllStances / channel.LongestStance);
        }
    }

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

    private static Vector3 ComputePointPosition(GaitChannel channel, MotionPoint point, Motion motion, float pathTime, float pointSpeed, Transform transform)
    {
        Vector3 pointPosition = point.Position;

        Vector3 linVelocity = motion.LinearVelocity;
        float angVelocity = motion.AngularVelocity;
        Vector3 origin = motion.RotationOrigin;


        //Below finds the local ideal position of the stride point, as in where on the line/arc it is at
        if (motion.HasRotation())
        {
            float pathRadius = (pointPosition - origin).magnitude;
            if (pathRadius > 0.0f)
            {
                float angleRange = (pointSpeed * pathTime) / (pathRadius * Mathf.Deg2Rad);

                Debug.DrawRay(transform.TransformPoint(origin + Vector3.up * 0.05f), transform.TransformDirection(Util.Rotate(pointPosition - origin, -angleRange)));
                Debug.DrawRay(transform.TransformPoint(origin + Vector3.up * 0.05f), transform.TransformDirection(Util.Rotate(pointPosition - origin, angleRange)));

                Util.DrawArc(transform.TransformPoint(origin + Vector3.up * 0.05f), transform.TransformDirection(pointPosition - origin), transform.TransformDirection(Vector3.up), pathRadius, pathRadius, 32, Color.white, -angleRange, angleRange * 2.0f);


                //float fLength = pathRadius * (angleRange * Mathf.Deg2Rad);
                //Debug.DrawRay(transform.TransformPoint(pointPosition + Vector3.up * 0.1f), transform.TransformDirection(Vector3.forward * fLength / 2), regions[i].DbgCol);
                //Debug.DrawRay(transform.TransformPoint(pointPosition + Vector3.up * 0.1f), transform.TransformDirection(Vector3.back * fLength / 2), regions[i].DbgCol);

                pointPosition = Util.Rotate(pointPosition - origin, angleRange * Mathf.Sign(angVelocity) * channel.Displacement) + origin;
            }
        }
        else
        {
            float pathLength = pointSpeed * pathTime;
            if (!float.IsNaN(pathLength))
            {
                Vector3 endPos1 = pointPosition + (linVelocity.normalized * -pathLength) + new Vector3(linVelocity.normalized.z, 0.0f, -linVelocity.normalized.x) * pathLength * 0.5f;
                Vector3 endPos2 = pointPosition + (linVelocity.normalized * -pathLength) - new Vector3(linVelocity.normalized.z, 0.0f, -linVelocity.normalized.x) * pathLength * 0.5f;
                Debug.DrawLine(transform.TransformPoint(endPos1 + Vector3.up * 0.05f), transform.TransformPoint(endPos2 + Vector3.up * 0.05f));

                Vector3 endPos3 = pointPosition - (linVelocity.normalized * -pathLength) + new Vector3(linVelocity.normalized.z, 0.0f, -linVelocity.normalized.x) * pathLength * 0.5f;
                Vector3 endPos4 = pointPosition - (linVelocity.normalized * -pathLength) - new Vector3(linVelocity.normalized.z, 0.0f, -linVelocity.normalized.x) * pathLength * 0.5f;
                Debug.DrawLine(transform.TransformPoint(endPos3 + Vector3.up * 0.05f), transform.TransformPoint(endPos4 + Vector3.up * 0.05f));

                Debug.DrawRay(transform.TransformPoint(pointPosition + Vector3.up * 0.05f), transform.TransformDirection(linVelocity.normalized * pathLength));
                Debug.DrawRay(transform.TransformPoint(pointPosition + Vector3.up * 0.05f), transform.TransformDirection(linVelocity.normalized * -pathLength));

                pointPosition += linVelocity.normalized * pathLength * channel.Displacement;

                //Debug.DrawRay(transform.TransformPoint(pointPosition + Vector3.up * 0.1f), transform.TransformDirection(Vector3.forward * pathLength / 2), regions[i].DbgCol);
                //Debug.DrawRay(transform.TransformPoint(pointPosition + Vector3.up * 0.1f), transform.TransformDirection(Vector3.back * pathLength / 2), regions[i].DbgCol);
            }
        }
        pointPosition.y += channel.Height;

        return pointPosition;
    }

    public void ComputePositions(List<LegStuff> legs, Motion[] motions, float minPathTime, float[] pointSpeeds, Transform transform, float debugTrailDuration)
    {
        for (int i = 0; i < legs.Count; i++)
        {
            LegStuff leg = legs[i];

            leg.gaitPosition = ComputePointPosition(_channels[i], leg.point, motions[i], minPathTime, pointSpeeds[i], transform);

            Vector3 debugPosition;
            if (_channels[i].IsStriding)
            {
                leg.finalPosition = leg.gaitPosition + leg.positionDiff;
                debugPosition = transform.TransformPoint(leg.finalPosition);

                Debug.DrawRay(transform.TransformPoint(leg.gaitPosition), transform.TransformDirection(leg.positionDiff), new Color(0.75f, 0.75f, 0.75f));

                leg.positionDiff *= 0.95f;
            }
            else
            {
                leg.finalPosition = leg.groundPosition;
                debugPosition = transform.TransformPoint(leg.finalPosition);
            }

            Debug.DrawLine(leg.debugLastPosition, debugPosition, leg.region.DbgCol, debugTrailDuration);
            leg.debugLastPosition = debugPosition;
        }
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////