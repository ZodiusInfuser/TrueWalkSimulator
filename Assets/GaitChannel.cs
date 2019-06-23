using System.Collections.Generic;
using UnityEngine;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public class GaitChannel
{
    //--------------------------------------------------
    // Constants
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    private const float MAX_DISPLACEMENT = 1.0f;

    /// <summary>
    /// 
    /// </summary>
    private const float PEAK_HEIGHT = 1.0f;

    /// <summary>
    /// 
    /// </summary>
    private const float MIN_STRIDE_PERCENT = 0.5f;


    //--------------------------------------------------
    // Variables
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    private List<GaitStride> _strides;

    /// <summary>
    /// 
    /// </summary>
    private int _nextStrideIndex;

    /// <summary>
    /// 
    /// </summary>
    private float _nextStanceDuration;
    
    /// <summary>
    /// 
    /// </summary>
    private float _lastReceivedOverrun;    

    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    private float _progressAlong;

    /// <summary>
    /// 
    /// </summary>
    private float _parabolaK;

    /// <summary>
    /// 
    /// </summary>
    private float _parabolaH;

    /// <summary>
    /// 
    /// </summary>
    private float _parabolaA;
    
    /// <summary>
    /// 
    /// </summary>
    private float _parabolaD;

    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    private float _pathM;

    /// <summary>
    /// 
    /// </summary>
    private float _pathB;


    //--------------------------------------------------
    // Constructors
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    public GaitChannel()
    {
        _strides = new List<GaitStride>();
        _nextStrideIndex = 0;
        _lastReceivedOverrun = 0.0f;
        _nextStanceDuration = 0.0f;

        IsStriding = false;
        Height = 0.0f;
        Displacement = 0.0f;
        LongestStance = 1.0f;
    }


    //--------------------------------------------------
    // Properties
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    public bool IsStriding { get; private set; }

    /// <summary>
    /// 
    /// </summary>
    public float Height { get; private set; }

    /// <summary>
    /// 
    /// </summary>
    public float Displacement { get; private set; }

    /// <summary>
    /// 
    /// </summary>
    public float LongestStance { get; private set; }

    /// <summary>
    /// 
    /// </summary>
    public float NextAvailableStancePercent { get; private set; }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    /// <param name="newStrides"></param>
    /// <param name="cycleTime"></param>
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
    /// 
    /// </summary>
    /// <param name="heightOffset"></param>
    /// <param name="displacementOffset"></param>
    public void ApplyOffset(float heightOffset, float displacementOffset)
    {
        if (!IsStriding)
        {
            Height += heightOffset;
            Displacement += displacementOffset;
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="overrun"></param>
    /// <param name="cycleTime"></param>
    /// <returns></returns>
    public bool AttemptNextStride(float overrun, float cycleTime)
    {
        bool strideStarted = false;
        if (!IsStriding && _nextStrideIndex < _strides.Count)
        {
            if (_strides[_nextStrideIndex].Within(cycleTime))
            {
                if (overrun == 0.0f)
                {
                    float startTime = 0.0f; //May do something with this in the future

                    float availableStrideTime = _strides[_nextStrideIndex].End - cycleTime;
                    if (availableStrideTime >= _strides[_nextStrideIndex].StrideLength * MIN_STRIDE_PERCENT)
                    {
                        int next = (_nextStrideIndex + 1) % _strides.Count;
                        _nextStanceDuration = GaitStride.StanceLength(_strides[_nextStrideIndex], _strides[next]);

                        float stanceOffsetScale = _nextStanceDuration / LongestStance;

                        //Trigger Stride
                        BeginStride(0.0f, Height + PEAK_HEIGHT, MAX_DISPLACEMENT * stanceOffsetScale, availableStrideTime + startTime, startTime);
                        _nextStrideIndex += 1;
                        _lastReceivedOverrun = 0.0f;
                        strideStarted = true;
                    }
                    else
                    {
                        _nextStrideIndex += 1;
                        Debug.Log("Stride Skipped");
                    }
                }
                else
                {
                    _lastReceivedOverrun = overrun;
                }
            }
        }
        return strideStarted;
    }

    /// <summary>
    /// 
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
    /// 
    /// </summary>
    public void CycleLooped()
    {
        _nextStrideIndex = _nextStrideIndex % _strides.Count;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="pos"></param>
    /// <param name="bYesNo"></param>
    /// <returns></returns>
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
                Debug.DrawRay(pos + (Vector3.up * 10.0f), Vector3.down * hit.distance, Color.yellow);
            else
                Debug.DrawRay(pos + (Vector3.up * 10.0f), Vector3.down * 1000, Color.white);

            if (hasHit && hit.distance < 10.0f && (_progressAlong >= _parabolaD * 0.5f))
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
    /// 
    /// </summary>
    /// <param name="endHeight"></param>
    /// <param name="peakHeight"></param>
    /// <param name="endDisplacement"></param>
    /// <param name="duration"></param>
    /// <param name="startTime"></param>
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
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////