using System.Collections.Generic;
using UnityEngine;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// The running instance of a set of gait strides for a single leg
/// </summary>
public class GaitChannel
{
    //--------------------------------------------------
    // Constants
    //--------------------------------------------------

    /// <summary>
    /// The maximum displacement from zero
    /// </summary>
    private const float MAX_DISPLACEMENT = 1.0f;

    /// <summary>
    /// The minimum percentage of a stride that is allowed to be performed (avoids unintentionally short strides from occuring)
    /// </summary>
    private const float MIN_STRIDE_PERCENT = 0.5f;

    /// <summary>
    /// The percentage of a stride that needs to have passed before raycasts for collision contact can start (avoids strides ending as soon as they have started)
    /// </summary>
    private const float RAYCAST_CHECK_PERCENT = 0.5f;

    /// <summary>
    /// The length of the collision contact raycast
    /// </summary>
    private const float RAYCAST_LENGTH = 1000.0f;

    /// <summary>
    /// The height above the raycast point from which to cast from
    /// </summary>
    private const float RAYCAST_HEIGHT_OFFSET = 10.0f;


    //--------------------------------------------------
    // Variables
    //--------------------------------------------------

    /// <summary>
    /// The list of GaitStrides that are currently being performed by this channel
    /// </summary>
    private List<GaitStride> _strides;

    /// <summary>
    /// The index of the next stride in the list
    /// </summary>
    private int _nextStrideIndex;

    /// <summary>
    /// The duration of the stance between the end of the current stride and the start of the next
    /// </summary>
    private float _nextStanceDuration;

    //--------------------------------------------------

    /// <summary>
    /// How far along
    /// </summary>
    private float _progressAlong;

    /// <summary>
    /// Stride parabola K value
    /// </summary>
    private float _parabolaK;

    /// <summary>
    /// Stride parabola H value
    /// </summary>
    private float _parabolaH;

    /// <summary>
    /// Stride parabola A value
    /// </summary>
    private float _parabolaA;

    /// <summary>
    /// Stride parabola D value
    /// </summary>
    private float _parabolaD;

    //--------------------------------------------------

    /// <summary>
    /// Stride path M value
    /// </summary>
    private float _pathM;

    /// <summary>
    /// Stride path B value
    /// </summary>
    private float _pathB;


    //--------------------------------------------------
    // Constructors
    //--------------------------------------------------

    /// <summary>
    /// Creates a new GaitChannel with a given peak stride height
    /// </summary>
    /// <param name="peakHeight">The peak height each stride will go to</param>
    public GaitChannel(float peakHeight)
    {
        _strides = new List<GaitStride>();
        _nextStrideIndex = 0;
        _nextStanceDuration = 0.0f;

        IsStriding = false;
        Height = 0.0f;
        PeakHeight = Mathf.Max(peakHeight, 0.0f);
        Displacement = 0.0f;
        LongestStance = 1.0f;
    }


    //--------------------------------------------------
    // Properties
    //--------------------------------------------------

    /// <summary>
    /// Whether or not this channel is striding
    /// </summary>
    public bool IsStriding { get; private set; }

    /// <summary>
    /// The current height calculated by this channel
    /// </summary>
    public float Height { get; private set; }

    /// <summary>
    /// Get or set the peak height strides of this channel will go to
    /// </summary>
    public float PeakHeight { get; set; }

    /// <summary>
    /// The current displacement calculated by this channel
    /// </summary>
    public float Displacement { get; private set; }

    /// <summary>
    /// The duration of the longest stance between the currently loaded list of GaitStrides
    /// </summary>
    public float LongestStance { get; private set; }

    /// <summary>
    /// How long a stance should be performed next, based on the current displacement of the channel
    /// </summary>
    public float NextAvailableStancePercent { get; private set; }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// Assign a new list of strides to this channel
    /// </summary>
    /// <param name="newStrides">The strides to assign, replacing the current list of strides</param>
    /// <param name="cycleTime">The current cycle time</param>
    public void AssignStrides(List<GaitStride> newStrides, float cycleTime)
    {
        _strides = new List<GaitStride>(newStrides);
        _nextStrideIndex = 0;

        //Depending on the cycle time, the first stride may not be the best one to start with
        do
        {
            //Check if the stride start has already been passed
            GaitStride nextStride = _strides[_nextStrideIndex % _strides.Count];
            _nextStanceDuration = nextStride.Start - cycleTime;
            if (_nextStanceDuration < 0.0f)
            {
                //If the stride isn't one cycle ahead
                if (_nextStrideIndex < _strides.Count)
                {
                    //See if there's enough time to do a worthwhile stride
                    float availableStrideTime = nextStride.End - cycleTime;
                    if (availableStrideTime >= nextStride.StrideLength * MIN_STRIDE_PERCENT)
                        _nextStanceDuration = 0.0f; //There is so zero the stance duration
                    else
                        _nextStrideIndex++; //There isn't so move on to the next stride
                }
                else
                {
                    //We're back to the first stride so add a whole cycle to the stance preceding it
                    _nextStanceDuration += 1.0f;
                }
            }
        } while (_nextStanceDuration < 0.0f);

        LongestStance = 0.0f;
        for (int i = 0; i < _strides.Count; i++)
        {
            int next = (i + 1) % _strides.Count;
            LongestStance = Mathf.Max(GaitStride.StanceLength(_strides[i], _strides[next]), LongestStance);
        }

        NextAvailableStancePercent = _nextStanceDuration / (Displacement + MAX_DISPLACEMENT);
    }

    /// <summary>
    /// Apply offsets to the height and displacement based on external factors, but only if this channel is not striding
    /// </summary>
    /// <param name="heightOffset">The height offset to apply</param>
    /// <param name="displacementOffset">The displacement offset to apply</param>
    public void ApplyOffsets(float heightOffset, float displacementOffset)
    {
        if (!IsStriding)
        {
            Height += heightOffset;
            Displacement += displacementOffset;
        }
    }

    /// <summary>
    /// Reset the displacement back to zero, but only if this channel is not striding.
    /// Also recalculates the NextAvailableStancePercent
    /// </summary>
    public void ResetDisplacement()
    {
        if (!IsStriding)
        {
            Displacement = 0.0f;
            NextAvailableStancePercent = _nextStanceDuration / (Displacement + MAX_DISPLACEMENT);
        }
    }

    /// <summary>
    /// Attempt to perform the next stride in the list
    /// </summary>
    /// <param name="cycleTime">The current cycle time</param>
    /// <returns>True if the next stride has started, false otherwise</returns>
    public bool AttemptNextStride(float cycleTime)
    {
        bool strideStarted = false;
        if (!IsStriding && _nextStrideIndex < _strides.Count)
        {
            if (_strides[_nextStrideIndex].Within(cycleTime))
            {
                float startTime = 0.0f; //May do something with this in the future

                float availableStrideTime = _strides[_nextStrideIndex].End - cycleTime;
                if (availableStrideTime >= _strides[_nextStrideIndex].StrideLength * MIN_STRIDE_PERCENT)
                {
                    int next = (_nextStrideIndex + 1) % _strides.Count;
                    _nextStanceDuration = GaitStride.StanceLength(_strides[_nextStrideIndex], _strides[next]);

                    float stanceOffsetScale = _nextStanceDuration / LongestStance;

                    //Trigger Stride
                    BeginStride(0.0f, Height + PeakHeight, MAX_DISPLACEMENT * stanceOffsetScale, availableStrideTime + startTime, startTime);
                    _nextStrideIndex += 1;
                    strideStarted = true;
                }
                else
                {
                    _nextStrideIndex += 1;
                    Debug.Log("Stride Skipped");
                }
            }
        }
        return strideStarted;
    }

    /// <summary>
    /// Calculate the time remaining until the next stride should start
    /// </summary>
    /// <param name="cycleTime">The current cycle time</param>
    /// <returns>The time until the next stride should start. If this is zero then the next stride should start now (or have already started)</returns>
    public float TimeUntilNextStride(float cycleTime)
    {
        float timeToNextStride = float.PositiveInfinity;
        if (!IsStriding)
        {
            //If the stride is one cycle ahead
            if (_nextStrideIndex == _strides.Count)
                timeToNextStride = (_strides[_nextStrideIndex % _strides.Count].Start + 1.0f) - cycleTime;

            //If the stride start has yet to be passed
            else if (cycleTime < _strides[_nextStrideIndex].Start)
                timeToNextStride = _strides[_nextStrideIndex].Start - cycleTime;
            else
                timeToNextStride = 0.0f;
        }
        return timeToNextStride;
    }

    /// <summary>
    /// Calculate the time remaining until the next stance should start
    /// </summary>
    /// <param name="cycleTime">The current cycle time</param>
    /// <returns>The time until the next stride should start. If this is zero then the next stance should start now (or have already started)</returns>
    public float TimeUntilStance(float cycleTime)
    {
        float timeToStance = float.PositiveInfinity;
        if (IsStriding)
        {
            int currentStrideIndex = (_nextStrideIndex + (_strides.Count - 1)) % _strides.Count;
            if (_strides[currentStrideIndex].Within(cycleTime))
                timeToStance = _strides[currentStrideIndex].End - cycleTime;
            else
                timeToStance = 0.0f;
        }
        else
        {
            if (_nextStrideIndex == _strides.Count)
                timeToStance = (_strides[_nextStrideIndex % _strides.Count].End + 1.0f) - cycleTime;
            else
                timeToStance = _strides[_nextStrideIndex].End - cycleTime;
        }
        return timeToStance;
    }

    /// <summary>
    /// Update this gait channel by the provided cycle timestep.
    /// This advances any active stride forward in time, resulting in new Height and Displacement values
    /// </summary>
    /// <param name="cycleTimestep"></param>
    public void Update(float cycleTimestep)
    {
        if (IsStriding)
        {
            _progressAlong += cycleTimestep;
            Height = -_parabolaA * ((_progressAlong - _parabolaH) * (_progressAlong - _parabolaH)) + _parabolaK;
            Displacement = (_pathM * _progressAlong) + _pathB;
        }
    }

    /// <summary>
    /// Informs this channel that the cycle has looped
    /// </summary>
    public void CycleLooped()
    {
        _nextStrideIndex = _nextStrideIndex % _strides.Count;
    }

    /// <summary>
    /// Checks if the active stride (if there is one) has completed.
    /// Currently this is done by performing a raycast RAYCAST_HEIGHT_OFFSET above the provided position down in to any collision geometry.
    /// The raycast is only performed once the active stride has passed the RAYCAST_CHECK_PERCENT
    /// </summary>
    /// <param name="pos">The position to raycast from (plus height offset)</param>
    /// <param name="bYesNo">Whether the raycast hit anything</param>
    /// <returns>If the raycast did hit, the amount of cycle time the stride overran by, or zero</returns>
    public float CheckForStrideComplete(Vector3 pos, out bool bYesNo)
    {
        bYesNo = false;
        float fOverrun = 0.0f;
        if (IsStriding)
        {
            RaycastHit hit;
            // Does the ray intersect any objects excluding the player layer
            bool hasHit = Physics.Raycast(pos + (Vector3.up * 10.0f), Vector3.down, out hit, Mathf.Infinity);
            if (hasHit)
                Debug.DrawRay(pos + (Vector3.up * RAYCAST_HEIGHT_OFFSET), Vector3.down * hit.distance, Color.yellow);
            else
                Debug.DrawRay(pos + (Vector3.up * RAYCAST_HEIGHT_OFFSET), Vector3.down * RAYCAST_LENGTH, Color.white);

            if (hasHit && hit.distance < 10.0f && (_progressAlong >= _parabolaD * RAYCAST_CHECK_PERCENT))
            {
                IsStriding = false;
                NextAvailableStancePercent = _nextStanceDuration / (Displacement + MAX_DISPLACEMENT);
                bYesNo = true;
            }
            else if (_progressAlong >= _parabolaD)
            {
                fOverrun = _progressAlong - _parabolaD;
            }
        }
        return fOverrun;
    }

    /// <summary>
    /// Begin a stride with the provided parameters
    /// </summary>
    /// <param name="endHeight">The height the stride should end at</param>
    /// <param name="peakHeight">The height the stride should go to at its peak</param>
    /// <param name="endDisplacement">The displacement the stride should end at</param>
    /// <param name="duration">The duration (in cycle time) that the stride should run for</param>
    /// <param name="startTime">The time into this stride it should start at</param>
    private void BeginStride(float endHeight, float peakHeight, float endDisplacement, float duration, float startTime)
    {
        //Is the duration a non-zero length?
        if (duration > 0.0f)
        {
            float peak = Mathf.Max(Mathf.Max(Height, endHeight), peakHeight);
            float trough = Mathf.Min(Mathf.Min(Height, endHeight), peakHeight);

            //Does the stride have a height difference?
            if ((peak - trough) > 0.0f)
            {
                _parabolaK = peak;
                _parabolaD = duration;

                if (Height < peak)
                {
                    //https://www.desmos.com/calculator/pskenhejyw
                    float lp = Mathf.Sqrt(peak - Height);
                    float ep = Mathf.Sqrt(peak - endHeight);

                    _parabolaH = duration * (lp / (lp + ep));
                    _parabolaA = (peak - Height) / (_parabolaH * _parabolaH);
                }
                else
                {
                    //https://www.desmos.com/calculator/udc9vgycb8
                    _parabolaH = 0;
                    _parabolaA = (peak - endHeight) / (duration * duration);
                }

                //https://www.desmos.com/calculator/kbqjcqzg5i
                _pathM = (endDisplacement - Displacement) / duration;
                _pathB = Displacement;

                _progressAlong = startTime;
                IsStriding = true;
            }
        }
    }

    /// <summary>
    /// How much cycle time of the current stride is remaining.
    /// Note, this can be negative if the stride is overrunning
    /// </summary>
    /// <returns>The stride duration remaining</returns>
    public float StrideDurationRemaining()
    {
        return _parabolaD - _progressAlong;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////