using UnityEngine;
using UnityEditor;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public class IterativeIKTranslationalJoint : IterativeIKJoint
{
    //--------------------------------------------------
    // Variables
    //--------------------------------------------------

    [Header("Translation")]

    /// <summary>
    ///  
    /// </summary>
    [SerializeField]
    protected float _minDisplacement = -1.0f;

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    protected float _maxDisplacement = 1.0f;

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    private float _poseDisplacement = 0.0f;

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    protected float _translationRate = 5.0f;

    /// <summary>
    /// 
    /// </summary>
    private Vector3 _zeroPosition = Vector3.zero;

    /// <summary>
    /// 
    /// </summary>
    private float _currentDisplacement = 0.0f;

    /// <summary>
    /// 
    /// </summary>
    private float _outputDisplacement = 0.0f;

    /// <summary>
    /// 
    /// </summary>
    private bool _firstRun = true;


    //--------------------------------------------------
    // Properties
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    public float MinDisplacement
    {
        get { return _minDisplacement; }
    }

    /// <summary>
    /// 
    /// </summary>
    public float MaxDisplacement
    {
        get { return _maxDisplacement; }
    }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    void Start()
    {
        _zeroPosition = transform.localPosition;
    }

    /// <summary>
    /// 
    /// </summary>
    void OnDrawGizmos()
    {
        Vector3 minPos = transform.TransformPoint(Vector3.forward * _minDisplacement);
        Vector3 minPosSide = minPos - transform.TransformDirection(Vector3.right * _debugScale);
        Debug.DrawLine(minPos, minPosSide, _debugColor);
        Handles.Label(minPosSide, _minDisplacement.ToString());

        Vector3 maxPos = transform.TransformPoint(Vector3.forward * _maxDisplacement);
        Vector3 maxPosSide = maxPos - transform.TransformDirection(Vector3.right * _debugScale);
        Debug.DrawLine(maxPos, maxPosSide, _debugColor);
        Handles.Label(maxPosSide, _maxDisplacement.ToString());

        Debug.DrawLine(minPos, maxPos, _debugColor);

        Vector3 idlePos = transform.TransformPoint(Vector3.forward * _poseDisplacement);
        Vector3 idlePosSide = idlePos + transform.TransformDirection(Vector3.right * _debugScale);
        Debug.DrawLine(idlePos, idlePosSide, _debugColor);
        Handles.Label(idlePosSide, _poseDisplacement.ToString());
    }

    /// <summary>
    /// 
    /// </summary>
    protected override void OnValidate()
    {
        base.OnValidate();
        _maxDisplacement = Mathf.Max(_maxDisplacement, _minDisplacement);
        _poseDisplacement = Mathf.Clamp(_poseDisplacement, _minDisplacement, _maxDisplacement);
    }

    /// <summary>
    /// 
    /// </summary>
    public override void InitialisePose()
    {
        _currentDisplacement = _poseDisplacement;
        transform.localPosition = _zeroPosition + (transform.localRotation * Vector3.forward * _currentDisplacement);
    }

    /// <summary>
    /// 
    /// </summary>
    public override void ApplyResult()
    {
        if (_instantMotion || _firstRun)
            _outputDisplacement = _currentDisplacement;
        else
            _outputDisplacement = Mathf.MoveTowards(_outputDisplacement, _currentDisplacement, _translationRate * Time.deltaTime);
        transform.localPosition = _zeroPosition + (transform.localRotation * Vector3.forward * _outputDisplacement);
        _firstRun = false;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <returns></returns>
    public override float GetValue()
    {
        return _outputDisplacement;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="ikEndPos"></param>
    /// <param name="ikTargetPos"></param>
    /// <returns></returns>
    public override bool AlignEndToTarget(Transform ikEndPos, Transform ikTargetPos)
    {
        bool bHasChanged = false;

        if (_controlled)
        {
            float jointToEnd = Vector3.Dot(ikEndPos.position - transform.position, transform.forward);
            float jointToTarget = Vector3.Dot(ikTargetPos.position - transform.position, transform.forward);

            float dispDiff = jointToTarget - jointToEnd;
            float newDisp = Mathf.Clamp(_currentDisplacement + dispDiff, _minDisplacement, _maxDisplacement);
            if (!Mathf.Approximately(newDisp - _currentDisplacement, 0.0f))
            {
                _currentDisplacement = newDisp;
                transform.localPosition = _zeroPosition + (transform.localRotation * Vector3.forward * _currentDisplacement);
                bHasChanged = true;
            }
            bHasChanged = true;
        }
        return bHasChanged;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////