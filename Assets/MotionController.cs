using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using System.IO.Ports;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public class MotionController : MonoBehaviour
{
    private MotionPoint _rootPoint;
    private List<Animation> _animationList;
    private List<IterativeIK> _ikList;

    [SerializeField]
    private bool velocityInput = false;

    [SerializeField]
    private float maxXZSpeed = 1.0f; //m/s

    [SerializeField]
    private float maxRSpeed = 90.0f; //deg/s

    [SerializeField]
    private float debugTrailDuration = 5.0f;

    [SerializeField]
    private float debugErrorDuration = 5.0f;

    [SerializeField]
    [Range(0, 5)]
    private float _minCyclesPerSecond = 0.1f;

    [SerializeField]
    [Range(0, 5)]
    private float _maxCyclesPerSecond = 1.0f;

    [SerializeField]
    private float _heightCorrection = 0.01f;

    [SerializeField]
    private bool _moveBody = true;

    [SerializeField]
    [Range(0, 1)]
    private float _balanceInset = 0.1f;

    private float _currentXSpeed = 0.0f;
    private float _currentZSpeed = 0.0f;
    private float _currentRSpeed = 0.0f;

#if _SHOWSPEED
    private Vector3 _oldPos = Vector3.zero;
    private float _oldAngle = 0.0f;
    float _newVelocity = 0.0f;
    float _newAngVel = 0.0f;
    float _ftime = 0.0f;
#endif

    private WalkController _walkController;
    private SerialPort _port;

    protected void OnValidate()
    {
        _maxCyclesPerSecond = Mathf.Max(_maxCyclesPerSecond, _minCyclesPerSecond);
    }

    //https://www.alanzucconi.com/2015/10/07/how-to-integrate-arduino-with-unity/
    public void CreateAndOpenPort(string portName)
    {
        ClosePort();

        try
        {
            _port = new SerialPort("\\\\.\\" + portName, 115200);
            _port.ReadTimeout = 50;
            _port.Open();

            foreach (IterativeIK ik in _ikList)
            {
                ik.SendShutdown(_port);
            }
        }
        catch (System.IO.IOException e)
        {
            Debug.Log("Cannot open Comm Port. " + e.Message);
        }
    }

    public void ClosePort()
    {
        if (_port != null)
        {
            if (_port.IsOpen)
            {
                foreach (IterativeIK ik in _ikList)
                {
                    ik.SendShutdown(_port);
                }
                _port.Close();
            }
            _port = null;
        }
    }

    public bool IsConnected()
    {
        bool isConnected = false;
        if (_port != null)
        {
            isConnected = _port.IsOpen;
        }
        return isConnected;
    }

    private static MotionPoint PointFromTransform(Transform baseTrans, Transform trans, float yAngle, out Quaternion angOut, MotionPoint pointParent)
    {
        Vector3 localPos; Quaternion localRot;
        Util.CalcRelativeTo(baseTrans, trans, out localPos, out localRot);
        localRot *= Quaternion.Euler(0.0f, yAngle, 0.0f);
        angOut = localRot;
        return MotionPoint.Local(localPos.x, localPos.z, localRot.eulerAngles.y, pointParent);
    }

    public static void FindChildrenIKs(Transform parent, List<IterativeIK> IKs)
    {
        foreach (Transform child in parent)
        {
            if (child.gameObject.activeSelf)
            {
                IterativeIK ik = child.GetComponent<IterativeIK>();
                if (ik != null)
                    IKs.Add(ik);
                else
                    FindChildrenIKs(child, IKs);
            }
        }
    }

    // Use this for initialization
    void Start()
    {
        _rootPoint = new MotionPoint(0, 0, 0); //Can be anything

        _animationList = new List<Animation>();
        Util.FindChildrenWithAnimation(transform, _animationList);

        _ikList = new List<IterativeIK>();
        FindChildrenIKs(transform, _ikList);

        _walkController = new WalkController(transform, null, _rootPoint, _minCyclesPerSecond, _maxCyclesPerSecond, _heightCorrection);
    }

    private void OnApplicationQuit()
    {
        ClosePort();
    }

    [ContextMenu("Queue Default Gait")]
    void QueueDefaultGait()
    {
        if (_walkController != null)
        {
            _walkController.QueueDefaultGait();
        }
    }

    [ContextMenu("Queue OneUp Gait")]
    void QueueOneUpGait()
    {
        if (_walkController != null)
        {
            _walkController.QueueNUpGait(1);
        }
    }

    [ContextMenu("Queue TwoUp Gait")]
    void QueueTwoUpGait()
    {
        if (_walkController != null)
        {
            _walkController.QueueNUpGait(2);
        }
    }

    [ContextMenu("Queue ThreeUp Gait")]
    void QueueThreeUpGait()
    {
        if (_walkController != null)
        {
            _walkController.QueueNUpGait(3);
        }
    }

    void OnDrawGizmos()
    {
        Handles.Label(transform.position + Vector3.up * 0.2f, "Vel = " + _newVelocity.ToString());
        Handles.Label(transform.position + Vector3.up * 0.6f, "AngVel = " + _newAngVel.ToString());
        Handles.Label(transform.position + Vector3.up * 1.0f, "XSpeed = " + _currentXSpeed.ToString());
        Handles.Label(transform.position + Vector3.up * 1.4f, "ZSpeed = " + _currentZSpeed.ToString());
        Handles.Label(transform.position + Vector3.up * 1.8f, "RSpeed = " + _currentRSpeed.ToString());
        Handles.Label(transform.position + Vector3.up * 2.2f, "Time = " + _ftime.ToString());

        Debug.DrawRay(transform.position, transform.TransformDirection(new Vector3(0, 0, 2)), Color.blue);
        Debug.DrawRay(transform.position, transform.TransformDirection(new Vector3(2, 0, 0)), Color.red);
        try
        {
            _rootPoint.Render(transform);
            _walkController.Render();
        }
        catch (NullReferenceException e)
        {
            //TODO handle this properly
        }
    }
#if _SHOWSPEED
    private void Debug_SpeedChecking()
    {
        Vector3 flatPos = Util.FlattenY(transform.position);

        _newVelocity = (flatPos - _oldPos).magnitude / Time.deltaTime;
        _oldPos = flatPos;

        float newAngle = transform.rotation.eulerAngles.y;
        _newAngVel = (newAngle - _oldAngle) / Time.deltaTime;
        _oldAngle = newAngle;
    }
#endif

    private void GetInput()
    {
        if (velocityInput)
        {
            if (Input.GetKey(KeyCode.W))
            {
                if (_currentZSpeed <= maxXZSpeed)
                    _currentZSpeed = Mathf.Min(_currentZSpeed + 0.01f, maxXZSpeed);
                else
                    _currentZSpeed = Mathf.Clamp(_currentZSpeed - 0.01f, 0.0f, maxXZSpeed);
            }
            if (Input.GetKey(KeyCode.S))
            {
                if (_currentZSpeed >= -maxXZSpeed)
                    _currentZSpeed = Mathf.Max(_currentZSpeed - 0.01f, -maxXZSpeed);
                else
                    _currentZSpeed = Mathf.Clamp(_currentZSpeed + 0.01f, -maxXZSpeed, 0.0f);
            }

            if (Input.GetKey(KeyCode.D))
            {
                if (_currentXSpeed <= maxXZSpeed)
                    _currentXSpeed = Mathf.Min(_currentXSpeed + 0.01f, maxXZSpeed);
                else
                    _currentXSpeed = Mathf.Clamp(_currentXSpeed - 0.01f, 0.0f, maxXZSpeed);
            }
            if (Input.GetKey(KeyCode.A))
            {
                if (_currentXSpeed >= -maxXZSpeed)
                    _currentXSpeed = Mathf.Max(_currentXSpeed - 0.01f, -maxXZSpeed);
                else
                    _currentXSpeed = Mathf.Clamp(_currentXSpeed + 0.01f, -maxXZSpeed, 0.0f);
            }

            if (Input.GetKey(KeyCode.E))
            {
                if (_currentRSpeed <= maxRSpeed)
                    _currentRSpeed = Mathf.Min(_currentRSpeed + 0.1f, maxRSpeed);
                else
                    _currentRSpeed = Mathf.Clamp(_currentRSpeed - 0.1f, 0.0f, maxRSpeed);
            }
            if (Input.GetKey(KeyCode.Q))
            {
                if (_currentRSpeed >= -maxRSpeed)
                    _currentRSpeed = Mathf.Max(_currentRSpeed - 0.1f, -maxRSpeed);
                else
                    _currentRSpeed = Mathf.Clamp(_currentRSpeed + 0.1f, -maxRSpeed, 0.0f);
            }
        }
        else
        {
            float sensitivityXZ = maxXZSpeed * 2.0f;
            float sensitivityR = maxRSpeed * 2.0f;
            //float dead = 0.000001f;
            float dead = 0.2f;

            float zTarget = 0.0f;
            if (Input.GetKey(KeyCode.W))
                zTarget += 1;
            if (Input.GetKey(KeyCode.S))
                zTarget -= 1;
            zTarget = Mathf.Clamp(zTarget + Input.GetAxis("LeftJoystickVertical"), -1.0f, 1.0f);

            float xTarget = 0.0f;
            if (Input.GetKey(KeyCode.D))
                xTarget += 1;
            if (Input.GetKey(KeyCode.A))
                xTarget -= 1;
            xTarget = Mathf.Clamp(xTarget + Input.GetAxis("LeftJoystickHorisontal"), -1.0f, 1.0f);

            Vector2 input = new Vector2(xTarget, zTarget);
            if (input.SqrMagnitude() > dead * dead)
            {
                input = (input - (input.normalized * dead)) / (1.0f - dead);
                input *= maxXZSpeed;
            }
            else
            {
                input = Vector2.zero;
            }
            _currentZSpeed = Mathf.MoveTowards(_currentZSpeed, input.y, sensitivityXZ * Time.deltaTime);
            _currentXSpeed = Mathf.MoveTowards(_currentXSpeed, input.x, sensitivityXZ * Time.deltaTime);
            //currentZSpeed = input.y;
            //currentXSpeed = input.x;

            float rTarget = 0.0f;
            if (Input.GetKey(KeyCode.E))
                rTarget += 1;
            if (Input.GetKey(KeyCode.Q))
                rTarget -= 1;
            rTarget = Mathf.Clamp(rTarget + Input.GetAxis("RightJoystickHorisontal"), -1.0f, 1.0f);

            if (rTarget * rTarget > dead * dead)
            {
                if (rTarget > 0.0f)
                    rTarget = (rTarget - dead) / (1.0f - dead);
                else
                    rTarget = (rTarget + dead) / (1.0f - dead);
                rTarget *= maxRSpeed;
            }
            else
            {
                rTarget = 0.0f;
            }
            _currentRSpeed = Mathf.MoveTowards(_currentRSpeed, rTarget, sensitivityR * Time.deltaTime);
            //currentRSpeed = rTarget;
        }
    }

    void FixedUpdate()
    {
#if _SHOWSPEED
        _ftime += Time.deltaTime;
        Debug_SpeedChecking();
#endif

        //--------------------------------------------------

        //Queue next gait

        //Get User Input
        GetInput();

        //Update Animations
        foreach (Animation anim in _animationList)
        {
            if (!anim.isPlaying && Input.GetKey(KeyCode.Space))
                anim.Play();
        }

        //--------------------------------------------------

        //Compute where the root motion point would be with the user speed inputs applied
        Motion rootMotion = new Motion(_currentXSpeed * Time.deltaTime, _currentZSpeed * Time.deltaTime, _currentRSpeed * Time.deltaTime);
        Motion bodyMotion = rootMotion.Convert(_rootPoint, MotionPoint.Identity);
        MotionPoint futureRoot = _rootPoint.RelativeMove(rootMotion);

        //--------------------------------------------------

        //Leg stuff
        float heightCorrectionCalc;
        float speedMultiplier;
        float cyclesPerSecond;
        _walkController.Update(futureRoot, bodyMotion, out heightCorrectionCalc, out speedMultiplier, out cyclesPerSecond);
        bodyMotion = bodyMotion.Scaled(speedMultiplier);

        ///// Would perform IK here if we were doing this outside of Unity /////

        //--------------------------------------------------

        if (_moveBody)
        {
            //For unity only, apply the motion to the body to move it in space
            ApplyMotionToBody(bodyMotion, heightCorrectionCalc);
        }

        //--------------------------------------------------

        _walkController.DebugTrace(debugTrailDuration);

        //--------------------------------------------------

        //On the real robot this would occur at the beginning of the update, as we would be checking contact button presses.
        //Check which legs have finished striding
        _walkController.CheckForStridesComplete();

        if (cyclesPerSecond == 0.0f)
            cyclesPerSecond = _minCyclesPerSecond;

        _walkController.Balance(cyclesPerSecond * Time.deltaTime, _balanceInset);
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="bodyMotion"></param>
    /// <param name="heightDiff"></param>
    public void ApplyMotionToBody(Motion bodyMotion, float heightDiff)
    {
        //Vector3 adjust = transform.position;
        //adjust.y -= fHeightDiff;
        //transform.position = adjust;

        //Apply the root motion to the actual game object so that it appears to be walking around
        Vector3 bodyPosition = transform.position;
        if (bodyMotion.HasRotation())
        {
            float rotationAmount = bodyMotion.AngularVelocity;

            Vector3 worldRotOrigin = transform.TransformPoint(bodyMotion.RotationOrigin);
            bodyPosition = Util.Rotate(bodyPosition - worldRotOrigin, rotationAmount) + worldRotOrigin;
            bodyPosition.y -= heightDiff;
            transform.Rotate(0.0f, rotationAmount, 0.0f);

            Debug.DrawLine(worldRotOrigin, bodyPosition, Color.red);
        }
        else if (bodyMotion.HasTranslation())
        {
            bodyPosition += transform.TransformDirection(bodyMotion.LinearVelocity);
            bodyPosition.y -= heightDiff;
        }

        //bodyPosition.y = transform.position.y;  //Reset the Y to whatever it was before
        transform.position = bodyPosition;
    }

    private void LateUpdate()
    {
        if (IsConnected())
        {
            if (_walkController.IsWalking())
                _port.Write("1");
            else
                _port.Write("0");

            foreach (IterativeIK ik in _ikList)
            {
                ik.SendAngles(_port);
            }

            _port.BaseStream.Flush();
        }
    }
}