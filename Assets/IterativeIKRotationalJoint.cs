using UnityEngine;
using UnityEditor;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public class IterativeIKRotationalJoint : IterativeIKJoint
{
    //--------------------------------------------------
    // Enums
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    public enum AlignmentMode
    {
        EndToTarget,
        EndZToTargetZ,
        EndXToTargetX,
        EndYToTargetY
    };


    //--------------------------------------------------
    // Variables
    //--------------------------------------------------

    //[Space(10, order = 0)]
    [Header("Rotation")]

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    [Range(-180, 180)]
    [Tooltip("Angle between -180 and 180 degrees")]
    protected float _minAngle = -90.0f;

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    [Range(-180, 180)]
    [Tooltip("Angle between -180 and 180 degrees. Cannot be smaller than minAngle")]
    protected float _maxAngle = 90.0f;

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    [Range(-180, 180)]
    [Tooltip("Angle between min and max angle to use when joint is not controlled")]
    private float _poseAngle = 0.0f;

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    [Tooltip("In degrees per second")]
    protected float _rotationRate = 300.0f;

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    protected AlignmentMode _align = AlignmentMode.EndToTarget;

    /// <summary>
    /// 
    /// </summary>
    private Quaternion _zeroRotation = Quaternion.identity;

    /// <summary>
    /// 
    /// </summary>
    private float _currentAngle = 0.0f;

    /// <summary>
    /// 
    /// </summary>
    private float _outputAngle = 0.0f;

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
    public float MinAngle
    {
        get { return _minAngle; }
    }

    /// <summary>
    /// 
    /// </summary>
    public float MaxAngle
    {
        get { return _maxAngle; }
    }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    void Start()
    {
        _zeroRotation = transform.localRotation;
    }

    /// <summary>
    /// 
    /// </summary>
    void OnDrawGizmos()
    {
        Util.DrawArc(transform.position, transform.TransformDirection(Vector3.forward), transform.TransformDirection(Vector3.up), _debugScale, _debugScale, 32, _debugColor, _minAngle - _outputAngle, _maxAngle - _minAngle);
        Debug.DrawRay(transform.position, transform.TransformDirection(Vector3.forward * _debugScale * 1.5f), _debugColor);

        Vector3 maxEnd = transform.position + transform.TransformDirection(Util.Rotate(Vector3.forward * _debugScale, _maxAngle - _outputAngle));
        Debug.DrawLine(transform.position, maxEnd, _debugColor);
        Handles.Label(maxEnd, _maxAngle.ToString());

        Vector3 minEnd = transform.position + transform.TransformDirection(Util.Rotate(Vector3.forward * _debugScale, _minAngle - _outputAngle));
        Debug.DrawLine(transform.position, minEnd, _debugColor);
        Handles.Label(minEnd, _minAngle.ToString());

        Vector3 idleEnd = transform.position + transform.TransformDirection(Util.Rotate(Vector3.forward * _debugScale, _poseAngle - _outputAngle));
        Debug.DrawLine(transform.position, idleEnd, _debugColor);
        Handles.Label(idleEnd, _poseAngle.ToString());
    }

    /// <summary>
    /// 
    /// </summary>
    protected override void OnValidate()
    {
        base.OnValidate();
        _maxAngle = Mathf.Max(_maxAngle, _minAngle);
        _poseAngle = Mathf.Clamp(_poseAngle, _minAngle, _maxAngle);
    }

    /// <summary>
    /// 
    /// </summary>
    public override void InitialisePose()
    {
        _currentAngle = _poseAngle;
        transform.localRotation = _zeroRotation * Quaternion.Euler(0, _currentAngle, 0);
    }

    /// <summary>
    /// 
    /// </summary>
    public override void ApplyResult()
    {
        if (_instantMotion || _firstRun)
            _outputAngle = _currentAngle;
        else
            _outputAngle = Mathf.MoveTowards(_outputAngle, _currentAngle, _rotationRate * Time.deltaTime);
        transform.localRotation = _zeroRotation * Quaternion.Euler(0, _outputAngle, 0);
        _firstRun = false;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <returns></returns>
    public override float GetValue()
    {
        return _outputAngle;
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
            switch (_align)
            {
                case AlignmentMode.EndToTarget:
                    {
                        Vector3 jointToEnd = ikEndPos.position - transform.position;
                        Vector3 jointToTarget = ikTargetPos.position - transform.position;
                        Vector3 projectedJointToEnd = new Vector3(Vector3.Dot(jointToEnd, transform.right), 0.0f, Vector3.Dot(jointToEnd, transform.forward));
                        Vector3 projectedJointToTarget = new Vector3(Vector3.Dot(jointToTarget, transform.right), 0.0f, Vector3.Dot(jointToTarget, transform.forward));

                        float angleDiff = Util.SignedYAngle(projectedJointToEnd, projectedJointToTarget);
                        float newAng = Mathf.Clamp(Util.WrapAngle(_currentAngle + angleDiff), _minAngle, _maxAngle);
                        if (!Mathf.Approximately(newAng - _currentAngle, 0.0f))
                        {
                            _currentAngle = newAng;
                            transform.localRotation = _zeroRotation * Quaternion.Euler(0, _currentAngle, 0);
                            bHasChanged = true;
                        }
                    }
                    break;

                case AlignmentMode.EndZToTargetZ:
                    {
                        Vector3 projectedEndAxis = new Vector3(Vector3.Dot(ikEndPos.forward, transform.right), 0.0f, Vector3.Dot(ikEndPos.forward, transform.forward));
                        Vector3 projectedTargetAxis = new Vector3(Vector3.Dot(ikTargetPos.forward, transform.right), 0.0f, Vector3.Dot(ikTargetPos.forward, transform.forward));

                        float angleDiff = Util.SignedYAngle(projectedEndAxis, projectedTargetAxis);
                        float newAng = Mathf.Clamp(Util.WrapAngle(_currentAngle + angleDiff), _minAngle, _maxAngle);
                        if (!Mathf.Approximately(newAng - _currentAngle, 0.0f))
                        {
                            _currentAngle = newAng;
                            transform.localRotation = _zeroRotation * Quaternion.Euler(0, _currentAngle, 0);
                            bHasChanged = true;
                        }
                    }
                    break;

                case AlignmentMode.EndXToTargetX:
                    {
                        Vector3 projectedEndAxis = new Vector3(Vector3.Dot(ikEndPos.right, transform.right), 0.0f, Vector3.Dot(ikEndPos.right, transform.forward));
                        Vector3 projectedTargetAxis = new Vector3(Vector3.Dot(ikTargetPos.right, transform.right), 0.0f, Vector3.Dot(ikTargetPos.right, transform.forward));

                        float angleDiff = Util.SignedYAngle(projectedEndAxis, projectedTargetAxis);
                        float newAng = Mathf.Clamp(Util.WrapAngle(_currentAngle + angleDiff), _minAngle, _maxAngle);
                        if (!Mathf.Approximately(newAng - _currentAngle, 0.0f))
                        {
                            _currentAngle = newAng;
                            transform.localRotation = _zeroRotation * Quaternion.Euler(0, _currentAngle, 0);
                            bHasChanged = true;
                        }
                    }
                    break;

                case AlignmentMode.EndYToTargetY:
                    {
                        Vector3 projectedEndAxis = new Vector3(Vector3.Dot(ikEndPos.up, transform.right), 0.0f, Vector3.Dot(ikEndPos.up, transform.forward));
                        Vector3 projectedTargetAxis = new Vector3(Vector3.Dot(ikTargetPos.up, transform.right), 0.0f, Vector3.Dot(ikTargetPos.up, transform.forward));

                        float angleDiff = Util.SignedYAngle(projectedEndAxis, projectedTargetAxis);
                        float newAng = Mathf.Clamp(Util.WrapAngle(_currentAngle + angleDiff), _minAngle, _maxAngle);
                        if (!Mathf.Approximately(newAng - _currentAngle, 0.0f))
                        {
                            _currentAngle = newAng;
                            transform.localRotation = _zeroRotation * Quaternion.Euler(0, _currentAngle, 0);
                            bHasChanged = true;
                        }
                    }
                    break;
            }
        }
        return bHasChanged;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////