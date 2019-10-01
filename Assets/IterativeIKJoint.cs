using UnityEngine;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public abstract class IterativeIKJoint : MonoBehaviour
{
    public enum ServoID
    {
        None = -1,
        Servo0 = 0,
        Servo1,
        Servo2,
        Servo3,
        Servo4,
        Servo5,
        Servo6,
        Servo7,
        Servo8,
        Servo9,
        Servo10,
        Servo11,
        Servo12,
        Servo13,
        Servo14,
        Servo15,
        Servo16,
        Servo17,
        Servo18,
        Servo19,
        Servo20,
        Servo21,
        Servo22,
        Servo23,
        Servo24,
        Servo25,
        Servo26,
        Servo27,
        Servo28,
        Servo29,
        Servo30,
        Servo31
    }

    //--------------------------------------------------
    // Variables
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    [Tooltip("Whether the joint can be controlled by IK")]
    protected bool _controlled = true;

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    protected bool _instantMotion = false;

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    protected float _debugScale = 1.0f;

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    protected Color _debugColor = new Color(1, 1, 1, 1);

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    private ServoID _servoID = ServoID.None;

    [SerializeField]
    private bool _suppressServoOutput = false;


    //--------------------------------------------------
    // Properties
    //--------------------------------------------------

    public ServoID ID
    {
        get { return _servoID; }
    }

    public bool ServoOutputSuppressed
    {
        get { return _suppressServoOutput; }
    }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    protected virtual void OnValidate()
    {
        _debugScale = Mathf.Max(_debugScale, 0.0f);
    }

    /// <summary>
    /// 
    /// </summary>
    public abstract void InitialisePose();

    /// <summary>
    /// 
    /// </summary>
    public abstract void ApplyResult();

    /// <summary>
    /// 
    /// </summary>
    /// <returns></returns>
    public abstract float GetValue();

    /// <summary>
    /// 
    /// </summary>
    /// <param name="ikEndPos"></param>
    /// <param name="ikTargetPos"></param>
    /// <returns></returns>
    public abstract bool AlignEndToTarget(Transform ikEndPos, Transform ikTargetPos);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////