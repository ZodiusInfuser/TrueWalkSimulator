using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class PositionControllable : MonoBehaviour
{
    private Quaternion _zeroRotation = Quaternion.identity;
    private Vector3 _zeroPosition = Vector3.zero;

    [SerializeField]
    protected float _debugScale = 1.0f;

    [SerializeField]
    protected Color _debugColor = new Color(1, 1, 1, 1);

    [SerializeField]
    [Range(0, 180)]
    protected float xAxisRange = 30.0f;

    [SerializeField]
    [Range(0, 180)]
    protected float yAxisRange = 30.0f;

    [SerializeField]
    [Range(0, 180)]
    protected float zAxisRange = 30.0f;

    private float _currentX = 0.0f;
    private float _currentY = 0.0f;
    private float _currentZ = 0.0f;

    [SerializeField]
    [Tooltip("In degrees per second")]
    protected float _rotationRate = 300.0f;


    [SerializeField]
    [Tooltip("In metres per second")]
    protected float _translationRate = 1.0f;

    private float _currentXT = 0.0f;
    private float _currentYT = 0.0f;
    private float _currentZT = 0.0f;

    void OnDrawGizmos()
    {
        float height = Mathf.Sin(xAxisRange * Mathf.Deg2Rad) * _debugScale;
        float radius = Mathf.Cos(xAxisRange * Mathf.Deg2Rad) * _debugScale;
        Quaternion fix = Quaternion.Euler(0, 0, -_currentZ) * Quaternion.Euler(-_currentX, 0, 0) * Quaternion.Euler(0, -_currentY, 0);

        Util.DrawArc(transform.position, transform.TransformDirection(fix * Vector3.forward), transform.TransformDirection(fix * Vector3.up), _debugScale, _debugScale, 32, _debugColor, -yAxisRange, yAxisRange * 2.0f);
        Util.DrawArc(transform.position + transform.TransformDirection(fix * new Vector3(0, height, 0)), transform.TransformDirection(fix * Vector3.forward), transform.TransformDirection(fix * Vector3.up), radius, radius, 32, _debugColor, -yAxisRange, yAxisRange * 2.0f);
        Util.DrawArc(transform.position + transform.TransformDirection(fix * new Vector3(0, -height, 0)), transform.TransformDirection(fix * Vector3.forward), transform.TransformDirection(fix * Vector3.up), radius, radius, 32, _debugColor, -yAxisRange, yAxisRange * 2.0f);

        Util.DrawArc(transform.position, transform.TransformDirection(fix * Vector3.forward), transform.TransformDirection(fix * Vector3.right), _debugScale, _debugScale, 32, _debugColor, -xAxisRange, xAxisRange * 2.0f);
        Util.DrawArc(transform.position, transform.TransformDirection(fix * Quaternion.Euler(0, yAxisRange, 0) * Vector3.forward), transform.TransformDirection(fix * Quaternion.Euler(0, yAxisRange, 0) * Vector3.right), _debugScale, _debugScale, 32, _debugColor, -xAxisRange, xAxisRange * 2.0f);
        Util.DrawArc(transform.position, transform.TransformDirection(fix * Quaternion.Euler(0, -yAxisRange, 0) * Vector3.forward), transform.TransformDirection(fix * Quaternion.Euler(0, -yAxisRange, 0) * Vector3.right), _debugScale, _debugScale, 32, _debugColor, -xAxisRange, xAxisRange * 2.0f);

        Vector3 plusXEnd = transform.TransformDirection(fix * Quaternion.Euler(xAxisRange, 0, 0) * Vector3.forward * _debugScale);
        Debug.DrawRay(transform.position, plusXEnd, _debugColor);
        Handles.Label(transform.position + plusXEnd, xAxisRange.ToString());

        Vector3 minusXEnd = transform.TransformDirection(fix * Quaternion.Euler(-xAxisRange, 0, 0) * Vector3.forward * _debugScale);
        Debug.DrawRay(transform.position, minusXEnd, _debugColor);
        Handles.Label(transform.position + minusXEnd, (-xAxisRange).ToString());

        Vector3 plusYEnd = transform.TransformDirection(fix * Quaternion.Euler(0, yAxisRange, 0) * Vector3.forward * _debugScale);
        Debug.DrawRay(transform.position, plusYEnd, _debugColor);
        Handles.Label(transform.position + plusYEnd, yAxisRange.ToString());

        Vector3 minusYEnd = transform.TransformDirection(fix * Quaternion.Euler(0, -yAxisRange, 0) * Vector3.forward * _debugScale);
        Debug.DrawRay(transform.position, minusYEnd, _debugColor);
        Handles.Label(transform.position + minusYEnd, (-yAxisRange).ToString());

        Debug.DrawRay(transform.position, transform.TransformDirection(fix * Quaternion.Euler(0, -yAxisRange, 0) * Quaternion.Euler(-xAxisRange, 0, 0) * Vector3.forward * _debugScale), _debugColor);
        Debug.DrawRay(transform.position, transform.TransformDirection(fix * Quaternion.Euler(0, -yAxisRange, 0) * Quaternion.Euler(xAxisRange, 0, 0) * Vector3.forward * _debugScale), _debugColor);
        Debug.DrawRay(transform.position, transform.TransformDirection(fix * Quaternion.Euler(0, yAxisRange, 0) * Quaternion.Euler(-xAxisRange, 0, 0) * Vector3.forward * _debugScale), _debugColor);
        Debug.DrawRay(transform.position, transform.TransformDirection(fix * Quaternion.Euler(0, yAxisRange, 0) * Quaternion.Euler(xAxisRange, 0, 0) * Vector3.forward * _debugScale), _debugColor);

        Debug.DrawRay(transform.position, transform.TransformDirection(Vector3.forward * _debugScale * 1.5f), _debugColor);

        Quaternion fix2 = Quaternion.Euler(0, 0, -_currentZ);
        Util.DrawArc(transform.position + transform.TransformDirection(Vector3.forward * _debugScale), transform.TransformDirection(fix2 * Vector3.up), transform.TransformDirection(Vector3.forward), _debugScale * 0.5f, _debugScale * 0.5f, 32, _debugColor, -zAxisRange, zAxisRange * 2.0f);

        Vector3 plusZEnd = transform.TransformDirection(fix2 * Quaternion.Euler(0, 0, zAxisRange) * Vector3.up * _debugScale * 0.5f);
        Debug.DrawRay(transform.position + transform.TransformDirection(Vector3.forward * _debugScale), plusZEnd, _debugColor);
        Handles.Label(transform.position + plusZEnd, zAxisRange.ToString());

        Vector3 minusZEnd = transform.TransformDirection(fix2 * Quaternion.Euler(0, 0, -zAxisRange) * Vector3.up * _debugScale * 0.5f);
        Debug.DrawRay(transform.position + transform.TransformDirection(Vector3.forward * _debugScale), minusZEnd, _debugColor);
        Handles.Label(transform.position + minusZEnd, zAxisRange.ToString());

        Debug.DrawRay(transform.position + transform.TransformDirection(Vector3.forward * _debugScale), transform.TransformDirection(Vector3.up * _debugScale * 0.75f), _debugColor);
    }

    // Use this for initialization
    void Start()
    {
        _zeroRotation = transform.localRotation;
        _zeroPosition = transform.localPosition;
    }

    // Update is called once per frame
    void Update()
    {

    }

    void FixedUpdate()
    {
        Vector3 dir = Vector3.zero;
        //dir.x = Input.GetAxis("LeftJoystickHorisontal");
        //dir.z = Input.GetAxis("LeftJoystickVertical");
        //if (dir.sqrMagnitude < 0.2f * 0.2f)
        //{
        //    dir = Vector3.zero;
        //}
        dir.y = -Input.GetAxis("Triggers");

        //_currentXT = Mathf.MoveTowards(_currentXT, dir.x * 0.4f, _translationRate * Time.deltaTime);
        _currentYT = Mathf.MoveTowards(_currentYT, dir.y * 0.5f, _translationRate * Time.deltaTime);
        //_currentZT = Mathf.MoveTowards(_currentZT, dir.z * 0.4f, _translationRate * Time.deltaTime);

        //float rotation = Input.GetAxis("RightJoystickVertical") * Time.deltaTime * 180.0f;

        float dead = 0.2f;

        Vector3 right = Vector3.zero;
        right.x = -Input.GetAxis("RightJoystickVertical");
        right.y = Input.GetAxis("RightJoystickHorisontal");
        if (right.sqrMagnitude > dead * dead)
        {
            right = (right - (right.normalized * dead)) / (1.0f - dead);
            right.x *= xAxisRange;
            right.y *= yAxisRange;
        }
        else
        {
            right.x = 0.0f;
            right.y = 0.0f;
        }
        _currentX = Mathf.MoveTowards(_currentX, right.x, _rotationRate * Time.deltaTime);
        _currentY = Mathf.MoveTowards(_currentY, right.y, _rotationRate * Time.deltaTime);

        right.z -= Input.GetButton("RightBumper") ? zAxisRange : 0;
        right.z += Input.GetButton("LeftBumper") ? zAxisRange : 0;
        _currentZ = Mathf.MoveTowards(_currentZ, right.z, _rotationRate * Time.deltaTime);

        transform.localRotation = _zeroRotation * Quaternion.Euler(_currentX, _currentY, _currentZ);
        transform.localPosition = _zeroPosition + new Vector3(_currentXT, _currentYT, _currentZT);
    }
}