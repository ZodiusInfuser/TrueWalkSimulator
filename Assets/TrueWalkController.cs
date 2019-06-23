using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public class LegStuff
{
    public FootMotionRegion region;
    public Transform childTransform;
    public MotionPoint point;    
    public Vector3 debugLastPosition;
    public Vector3 groundPosition;
    public Vector3 positionDiff;
    public Vector3 gaitPosition;
    public Vector3 finalPosition;
}

/// <summary>
/// 
/// </summary>
public class TrueWalkController : MonoBehaviour
{
    private MotionPoint rootPoint;
    private List<LegStuff> legs;

    [SerializeField]
    private float maxXZSpeed = 1.0f; //m/s

    [SerializeField]
    private float maxRSpeed = 90.0f; //deg/s

    [SerializeField]
    private float minTimestep = 0.001f;

    [SerializeField]
    private float maxTimestep = 0.1f;

    [SerializeField]
    private float debugTrailDuration = 5.0f;

    [SerializeField]
    private float heightCorrection = 0.01f;

    [SerializeField]
    private int legsSupported = 10;


    private float currentXSpeed = 0.0f;
    private float currentZSpeed = 0.0f;
    private float currentRSpeed = 0.0f;

#if _SHOWSPEED
    private Vector3 oldPos = Vector3.zero;
    private float oldAngle = 0.0f;
    float newVelocity = 0.0f;
    float newAngVel = 0.0f;
    float ftime = 0.0f;
#endif
    Gait twoUp;
    Gait fiveUp;
    Gait oneUpStaggered;
    Gait threeUp;

    private GaitController gaitController;

    private Transform[] _legHips;
    private Quaternion[] _CoxaIdleAngles;
    private Quaternion[] _FemurIdleAngles;
    private Quaternion[] _TibiaIdleAngles;

    // Use this for initialization
    void Start()
    {
        rootPoint = new MotionPoint(0, 0, 0);

        twoUp = new Gait(legsSupported);

        if (legsSupported == 10)
        {
            fiveUp = new Gait(10);
            fiveUp.AddStride(0, new GaitStride(0.0f, 0.5f));
            fiveUp.AddStride(2, new GaitStride(0.0f, 0.5f));
            fiveUp.AddStride(4, new GaitStride(0.0f, 0.5f));
            fiveUp.AddStride(6, new GaitStride(0.0f, 0.5f));
            fiveUp.AddStride(8, new GaitStride(0.0f, 0.5f));

            fiveUp.AddStride(1, new GaitStride(0.5f, 1.0f));
            fiveUp.AddStride(3, new GaitStride(0.5f, 1.0f));
            fiveUp.AddStride(5, new GaitStride(0.5f, 1.0f));
            fiveUp.AddStride(7, new GaitStride(0.5f, 1.0f));
            fiveUp.AddStride(9, new GaitStride(0.5f, 1.0f));

            oneUpStaggered = new Gait(10);
            oneUpStaggered.AddStride(0, new GaitStride(0.0f, 0.2f));
            oneUpStaggered.AddStride(0, new GaitStride(0.3f, 0.5f));
            oneUpStaggered.AddStride(1, new GaitStride(0.1f, 0.3f));
            oneUpStaggered.AddStride(2, new GaitStride(0.2f, 0.4f));
            oneUpStaggered.AddStride(3, new GaitStride(0.3f, 0.5f));
            oneUpStaggered.AddStride(4, new GaitStride(0.4f, 0.2f));

            oneUpStaggered.AddStride(5, new GaitStride(0.5f, 0.7f));
            oneUpStaggered.AddStride(5, new GaitStride(0.8f, 0.0f));
            oneUpStaggered.AddStride(6, new GaitStride(0.6f, 0.8f));
            oneUpStaggered.AddStride(7, new GaitStride(0.7f, 0.9f));
            oneUpStaggered.AddStride(8, new GaitStride(0.8f, 0.0f));
            oneUpStaggered.AddStride(9, new GaitStride(0.9f, 0.7f));

            Transform skeleton = transform.Find("Skeleton");
            _legHips = new Transform[10];
            _legHips[0] = skeleton.Find("LF0");
            _legHips[1] = skeleton.Find("LMF0");
            _legHips[2] = skeleton.Find("LMM0");
            _legHips[3] = skeleton.Find("LMB0");
            _legHips[4] = skeleton.Find("LB0");
            _legHips[5] = skeleton.Find("RF0");
            _legHips[6] = skeleton.Find("RMF0");
            _legHips[7] = skeleton.Find("RMM0");
            _legHips[8] = skeleton.Find("RMB0");
            _legHips[9] = skeleton.Find("RB0");

            _CoxaIdleAngles = new Quaternion[10];
            _FemurIdleAngles = new Quaternion[10];
            _TibiaIdleAngles = new Quaternion[10];
            for (int i = 0; i < 10; i++)
            {
                _CoxaIdleAngles[i] = _legHips[i].localRotation;
                _FemurIdleAngles[i] = _legHips[i].GetChild(0).localRotation;
                _TibiaIdleAngles[i] = _legHips[i].GetChild(0).GetChild(0).localRotation;
            }
        }
        else if(legsSupported == 6)
        {
            Transform skeleton = transform.Find("Skeleton");
            _legHips = new Transform[6];
            _legHips[0] = skeleton.Find("LF0");
            _legHips[1] = skeleton.Find("LM0");
            _legHips[2] = skeleton.Find("LB0");
            _legHips[3] = skeleton.Find("RF0");
            _legHips[4] = skeleton.Find("RM0");
            _legHips[5] = skeleton.Find("RB0");

            threeUp = new Gait(6);
            threeUp.AddStride(0, new GaitStride(0.0f, 0.5f));
            threeUp.AddStride(2, new GaitStride(0.0f, 0.5f));
            threeUp.AddStride(4, new GaitStride(0.0f, 0.5f));
            threeUp.AddStride(1, new GaitStride(0.5f, 1.0f));
            threeUp.AddStride(3, new GaitStride(0.5f, 1.0f));
            threeUp.AddStride(5, new GaitStride(0.5f, 1.0f));

            _CoxaIdleAngles = new Quaternion[6];
            _FemurIdleAngles = new Quaternion[6];
            _TibiaIdleAngles = new Quaternion[6];
            for (int i = 0; i < 6; i++)
            {
                _CoxaIdleAngles[i] = _legHips[i].localRotation;
                _FemurIdleAngles[i] = _legHips[i].GetChild(0).localRotation;
                _TibiaIdleAngles[i] = _legHips[i].GetChild(0).GetChild(0).localRotation;
            }
        }
        
        int index = 0;

        legs = new List<LegStuff>();
        foreach (Transform child in transform)
        {
            FootMotionRegion region = child.GetComponent<FootMotionRegion>();
            if (region != null)
            {
                LegStuff stuff = new LegStuff();

                stuff.region = region;
                stuff.childTransform = child;

                foreach (Vector2 stride in region.strides)
                {
                    twoUp.AddStride(index, new GaitStride(stride.x, stride.y));
                }

                stuff.point = new MotionPoint(child.localPosition.x, child.localPosition.z, 0.0f, rootPoint, false);
                stuff.debugLastPosition = child.position + new Vector3(0.0f, 0.0f, 0.0f);// stuff.channel.Height, stuff.channel.Offset);

                stuff.groundPosition = new Vector3(child.localPosition.x, 0.0f, child.localPosition.z);
                stuff.gaitPosition = new Vector3(child.localPosition.x, 0.0f, child.localPosition.z);
                stuff.finalPosition = stuff.gaitPosition;

                legs.Add(stuff);
                index++;
            }
        }

        gaitController = new GaitController(legs.Count);
        gaitController.QueueGait(twoUp);
    }

#if _SHOWSPEED
    void OnDrawGizmos()
    {
        Handles.Label(transform.position + Vector3.up * 0.2f, "Vel = " + newVelocity.ToString());
        Handles.Label(transform.position + Vector3.up * 0.6f, "AngVel = " + newAngVel.ToString());
        Handles.Label(transform.position + Vector3.up * 1.0f, "XSpeed = " + currentXSpeed.ToString());
        Handles.Label(transform.position + Vector3.up * 1.4f, "ZSpeed = " + currentZSpeed.ToString());
        Handles.Label(transform.position + Vector3.up * 1.8f, "RSpeed = " + currentRSpeed.ToString());
        Handles.Label(transform.position + Vector3.up * 2.2f, "Time = " + ftime.ToString());
    }

    private void Debug_SpeedChecking()
    {
        Vector3 flatPos = transform.position;
        flatPos.y = 0;
        newVelocity = (flatPos - oldPos).magnitude / Time.deltaTime;
        oldPos = flatPos;

        float newAngle = transform.rotation.eulerAngles.y;
        newAngVel = (newAngle - oldAngle) / Time.deltaTime;
        oldAngle = newAngle;
    }
#endif
    void Update()
    {
        Debug.DrawRay(transform.position, transform.TransformDirection(new Vector3(0, 0, 2)), Color.blue);
        Debug.DrawRay(transform.position, transform.TransformDirection(new Vector3(2, 0, 0)), Color.red);
        rootPoint.Render(transform);
        foreach (LegStuff leg in legs)
        {
            leg.point.Render(transform);
        }

        for (int i = 0; i < legsSupported; i++)
        {
            Vector3 worldFinalPos = transform.TransformPoint(legs[i].finalPosition);

            Transform joint1 = _legHips[i].GetChild(0);
            Transform joint2 = joint1.GetChild(0);
            Transform end = joint2.GetChild(0);

            _legHips[i].localRotation = _CoxaIdleAngles[i];
            joint1.localRotation = _FemurIdleAngles[i];
            joint2.localRotation = _TibiaIdleAngles[i];

            Vector3 coxaWorld = _legHips[i].position;
            Vector3 coxaRotAxis = _legHips[i].up;
            Vector3 coxaToEnd = Vector3.ProjectOnPlane(end.position - coxaWorld, coxaRotAxis);
            Vector3 coxaToTarget = Vector3.ProjectOnPlane(worldFinalPos - coxaWorld, coxaRotAxis);

            float coxaAngleDiff = Vector3.SignedAngle(coxaToEnd, coxaToTarget, coxaRotAxis);
            _legHips[i].Rotate(0, coxaAngleDiff, 0);

            for (int pass = 0; pass < 10; pass++)
            {
                Vector3 femurWorld = joint1.position;
                Vector3 femurRotAxis = joint1.forward;
                Vector3 femurToEnd = Vector3.ProjectOnPlane(end.position - femurWorld, femurRotAxis);
                Vector3 femurToTarget = Vector3.ProjectOnPlane(worldFinalPos - femurWorld, femurRotAxis);

                float femurAngleDiff = Vector3.SignedAngle(femurToEnd, femurToTarget, femurRotAxis);
                joint1.Rotate(0, 0, femurAngleDiff);


                Vector3 tibiaWorld = joint2.position;
                Vector3 tibiaRotAxis = joint2.forward;
                Vector3 tibiaToEnd = Vector3.ProjectOnPlane(end.position - tibiaWorld, tibiaRotAxis);
                Vector3 tibiaToTarget = Vector3.ProjectOnPlane(worldFinalPos - tibiaWorld, tibiaRotAxis);

                float tibiaAngleDiff = Vector3.SignedAngle(tibiaToEnd, tibiaToTarget, tibiaRotAxis);
                joint2.Rotate(0, 0, tibiaAngleDiff);
            }

            //float COXA_LENGTH = Vector3.ProjectOnPlane(joint1.position - _legHips[i].position, _legHips[i].up).magnitude;
            //float FEMUR_LENGTH = Vector3.ProjectOnPlane(joint2.position - joint1.position, joint1.forward).magnitude;
            //float TIBIA_LENGTH = Vector3.ProjectOnPlane(end.position - joint2.position, joint2.forward).magnitude;




            //float ikSW;                       // Length between Femur and Tibia 
            //float ikRadiansFemurTibiaGround;  // Angle of the line Femur and Tibia with respect to the ground in radians 
            //float ikRadiansFemurTibia;        // Angle of the line Femur and Tibia with respect to the femur in radians 
            //float ikFeetPosXZ;                // Distance between the Coxa and Ground Contact 

            //// Distance between the Coxa and Ground Contact 
            //ikFeetPosXZ = Mathf.Sqrt(worldFinalPos.x * worldFinalPos.x + worldFinalPos.z * worldFinalPos.z);

            //// ikSW - Length between Femur axis and Tibia 
            //ikSW = Mathf.Sqrt(Mathf.Pow((ikFeetPosXZ - COXA_LENGTH), 2) + Mathf.Pow(worldFinalPos.y, 2));

            //// ikRadiansFemurTibiaGround - Angle between Femur and Tibia line and the ground in radians 
            //ikRadiansFemurTibiaGround = Mathf.Atan2(ikFeetPosXZ - COXA_LENGTH, worldFinalPos.y);

            //// ikRadiansFemurTibia - Angle of the line Femur and Tibia with respect to the Femur in radians 
            //ikRadiansFemurTibia = Mathf.Acos(((Mathf.Pow(FEMUR_LENGTH, 2) - Mathf.Pow(TIBIA_LENGTH, 2)) + Mathf.Pow(ikSW, 2)) / (2 * FEMUR_LENGTH * ikSW));

            //// ikCoxaAngle in degrees 
            //Vector3 diff = worldFinalPos - _legHips[i].position;
            //float coxaAngle = -Mathf.Atan2(diff.z, diff.x) * Mathf.Rad2Deg;

            //// ikFemurAngle in degrees 
            //float femurAngle = -(ikRadiansFemurTibiaGround + ikRadiansFemurTibia) * 180 / Mathf.PI + 90;

            //// ikTibiaAngle in degrees 
            //float tibiaAngle = -(90 - (((Mathf.Acos((Mathf.Pow(FEMUR_LENGTH, 2) + Mathf.Pow(TIBIA_LENGTH, 2) - Mathf.Pow(ikSW, 2)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH))) * 180) / Mathf.PI));


            //Vector3 coxaEnd = Util.Rotate((Vector3.right * COXA_LENGTH), coxaAngle) + _legHips[i].position;
            //Vector3 femurEnd = Util.Rotate((Quaternion.Euler(0, 0, femurAngle) * Vector3.right * FEMUR_LENGTH * 2), coxaAngle) + coxaEnd;
            //Debug.DrawLine(_legHips[i].position, coxaEnd, legs[i].region.DbgCol);
            //Debug.DrawLine(coxaEnd, femurEnd, legs[i].region.DbgCol);

            //Vector3 diff = transform.TransformPoint(legs[i].finalPosition) - _legHips[i].position;
            //float angle = Mathf.Atan2(diff.x, diff.z) * Mathf.Rad2Deg;
            //float currentAngle = _legHips[i].rotation.eulerAngles.y + 90.0f;
            //if (Mathf.Abs(coxaAngle) > 0.0f)
            //_legHips[i].rotation = Quaternion.Euler(0, coxaAngle, 0);

            //Vector3 diff1 = transform.TransformPoint(legs[i].finalPosition) - joint1.position;
            //Vector3 length1 = joint2.position - joint1.position;
            //float angle1 = Vector3.Angle(length1, diff1);

            //joint1.localRotation = Quaternion.identity;
            //joint1.Rotate(0, 0, femurAngle);

        }
    }

    private void GetInput()
    {
    #if _VELOCITYINPUT
        if (Input.GetKey(KeyCode.W))
        {
            if (currentZSpeed <= maxXZSpeed)
                currentZSpeed = Mathf.Min(currentZSpeed + 0.01f, maxXZSpeed);
            else
                currentZSpeed = Mathf.Clamp(currentZSpeed - 0.01f, 0.0f, maxXZSpeed);
        }
        if (Input.GetKey(KeyCode.S))
        {
            if (currentZSpeed >= -maxXZSpeed)
                currentZSpeed = Mathf.Max(currentZSpeed - 0.01f, -maxXZSpeed);
            else
                currentZSpeed = Mathf.Clamp(currentZSpeed + 0.01f, -maxXZSpeed, 0.0f);
        }

        if (Input.GetKey(KeyCode.D))
        {
            if (currentXSpeed <= maxXZSpeed)
                currentXSpeed = Mathf.Min(currentXSpeed + 0.01f, maxXZSpeed);
            else
                currentXSpeed = Mathf.Clamp(currentXSpeed - 0.01f, 0.0f, maxXZSpeed);
        }
        if (Input.GetKey(KeyCode.A))
        {
            if (currentXSpeed >= -maxXZSpeed)
                currentXSpeed = Mathf.Max(currentXSpeed - 0.01f, -maxXZSpeed);
            else
                currentXSpeed = Mathf.Clamp(currentXSpeed + 0.01f, -maxXZSpeed, 0.0f);
        }

        if (Input.GetKey(KeyCode.E))
        {
            if (currentRSpeed <= maxRSpeed)
                currentRSpeed = Mathf.Min(currentRSpeed + 0.1f, maxRSpeed);
            else
                currentRSpeed = Mathf.Clamp(currentRSpeed - 0.1f, 0.0f, maxRSpeed);
        }
        if (Input.GetKey(KeyCode.Q))
        {
            if (currentRSpeed >= -maxRSpeed)
                currentRSpeed = Mathf.Max(currentRSpeed - 0.1f, -maxRSpeed);
            else
                currentRSpeed = Mathf.Clamp(currentRSpeed + 0.1f, -maxRSpeed, 0.0f);
        }
#else
        float sensitivityXZ = maxXZSpeed * 2.0f;
        float sensitivityR = maxRSpeed * 2.0f;
        float dead = 0.001f;
        
        float zTarget = 0.0f;
        if (Input.GetKey(KeyCode.W))
            zTarget += 1;
        if (Input.GetKey(KeyCode.S))
            zTarget -= 1;

        float xTarget = 0.0f;
        if (Input.GetKey(KeyCode.D))
            xTarget += 1;
        if (Input.GetKey(KeyCode.A))
            xTarget -= 1;

        Vector2 input = new Vector2(xTarget, zTarget);
        if (input.SqrMagnitude() != 0.0f)
        {
            input.Normalize();
            input *= maxXZSpeed;
            currentZSpeed = Mathf.MoveTowards(currentZSpeed, input.y, sensitivityXZ * Time.deltaTime);
            currentZSpeed = (Mathf.Abs(currentZSpeed) < dead) ? 0f : currentZSpeed;

            currentXSpeed = Mathf.MoveTowards(currentXSpeed, input.x, sensitivityXZ * Time.deltaTime);
            currentXSpeed = (Mathf.Abs(currentXSpeed) < dead) ? 0f : currentXSpeed;
        }
        else
        {
            currentZSpeed = Mathf.MoveTowards(currentZSpeed, 0.0f, sensitivityXZ * Time.deltaTime);
            currentXSpeed = Mathf.MoveTowards(currentXSpeed, 0.0f, sensitivityXZ * Time.deltaTime);
        }

        float rTarget = 0.0f;
        if (Input.GetKey(KeyCode.E))
            rTarget += maxRSpeed;
        if (Input.GetKey(KeyCode.Q))
            rTarget -= maxRSpeed;

        
        
        currentRSpeed = Mathf.MoveTowards(currentRSpeed, rTarget, sensitivityR * Time.deltaTime);
        currentRSpeed = (Mathf.Abs(currentRSpeed) < dead) ? 0f : currentRSpeed;
#endif
    }

    void FixedUpdate()
    {
        /////
        /// NEW FLOW
        /// - Get user input
        /// if all strides have stopped,there are no new motion inputs, and legs are near their resting points
        ///   - Compute new motions and motion points
        ///   - Compute the speed ratio
        ///   - Start any new strides accordingly
        ///   - Compute the scaled timestep
        ///   - Advance the gait generator in time
        ///   - Compensate for height difference and offsetoffset
        ///   - Move the body forward (or in the case of the robot, send commands to servo)
        /// endif
        /// - Check for feet contacts since last update
        ///

#if _SHOWSPEED
        ftime += Time.deltaTime;
        Debug_SpeedChecking();
#endif
        if (legsSupported == 10)
        {
            if (Input.GetKeyDown(KeyCode.Alpha1))
            {
                gaitController.QueueGait(fiveUp);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha2))
            {
                gaitController.QueueGait(twoUp);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha3))
            {
                gaitController.QueueGait(oneUpStaggered);
            }
        }
        else if (legsSupported == 6)
        {
            if (Input.GetKeyDown(KeyCode.Alpha1))
            {
                gaitController.QueueGait(threeUp);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha2))
            {
                gaitController.QueueGait(twoUp);
            }
        }

        //--------------------------------------------------

        //Get User Input
        GetInput();

        //Get all other factors (such as animation that may change motion points)

        //--------------------------------------------------

        //Compute where the root motion point would be with the user speed inputs applied
        Motion motion = new Motion(currentXSpeed, currentZSpeed, currentRSpeed);
        MotionPoint futureRoot = rootPoint.RelativeMove(motion);

        //Compute the individual foot point motions as a result of applying the previous motion to the root
        MotionPoint[] futurePoints = new MotionPoint[legs.Count];
        for (int i = 0; i < legs.Count; i++)
        {
            //Create a copy of the current leg's point with root point's motion applied to it
            MotionPoint point = legs[i].point;
            point.SetLocal(legs[i].childTransform.localPosition.x, legs[i].childTransform.localPosition.z, 0.0f, rootPoint);
            legs[i].point = point;

            futurePoints[i] = point.RelativeTo(futureRoot);
        }

        //--------------------------------------------------

        float minPathTime = float.PositiveInfinity;
        float maxPointSpeed = 0.0f;

        Motion[] motions = new Motion[legs.Count];
        float[] pointSpeeds = new float[legs.Count];

        //Compute the individual foot point motions as a result of applying the previous motion to the root
        for (int i = 0; i < legs.Count; i++)
        {
            LegStuff leg = legs[i];

            //Compute the motion that would have resulted in the leg's point moving to the future point
            motions[i] = Motion.Between(leg.point, futurePoints[i]);

            minPathTime = Mathf.Min(minPathTime, leg.region.BoundedPathFactors(leg.point, motions[i], gaitController.MaxStanceRatio(i), out pointSpeeds[i], transform));
            maxPointSpeed = Mathf.Max(maxPointSpeed, pointSpeeds[i]);
        }

        //--------------------------------------------------

        if (gaitController.IsAnyStriding() || !gaitController.IsIdle() || Mathf.Abs(maxPointSpeed) > 0.05f)
        {
            gaitController.CheckForNewStrides(legs);

            float scaledTimestep = gaitController.CalcScaledTimestep(minPathTime, maxTimestep, pointSpeeds, maxPointSpeed);
            //Scale the timestep further based on the speed ratio
            if (scaledTimestep > maxTimestep)
            {
                //minPathTime *= scaledTimestep / maxTimestep;
                scaledTimestep = maxTimestep;
            }
            else if (scaledTimestep < minTimestep)
            {
                minPathTime *= scaledTimestep / minTimestep;
                scaledTimestep = minTimestep;
            }

            gaitController.Update(scaledTimestep);

            //--------------------------------------------------

            float heightCorrection = gaitController.MeanStancedHeight() * -this.heightCorrection;
            float displacementCorrection = minPathTime > 0.0f ? -Time.deltaTime / minPathTime : 0.0f;
            gaitController.ApplyOffset(heightCorrection, displacementCorrection);            

            //--------------------------------------------------

            Motion bodyMotion = motion.Convert(rootPoint, MotionPoint.Identity);
            ApplyInverseMotion(bodyMotion, MotionPoint.Identity, transform, legs, heightCorrection);

            //--------------------------------------------------

            gaitController.ComputePositions(legs, motions, minPathTime, pointSpeeds, transform, debugTrailDuration);
        }

        foreach (LegStuff leg in legs)
        {
            Vector3 worldFinalPosition = transform.TransformPoint(leg.finalPosition);
            Debug.DrawRay(worldFinalPosition, transform.TransformDirection(new Vector3(0.1f, 0, 0.1f)), Color.white);
            Debug.DrawRay(worldFinalPosition, transform.TransformDirection(new Vector3(-0.1f, 0, 0.1f)), Color.white);
            Debug.DrawRay(worldFinalPosition, transform.TransformDirection(new Vector3(0.1f, 0, -0.1f)), Color.white);
            Debug.DrawRay(worldFinalPosition, transform.TransformDirection(new Vector3(-0.1f, 0, -0.1f)), Color.white);
            Debug.DrawLine(transform.position, worldFinalPosition, Color.gray);
        }

        //On the real robot this would occur at the beginning of the update, as we would be checking button presses.
        //Check which legs have finished striding
        gaitController.CheckForStridesComplete(legs, transform);
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="minTimestep"></param>
    /// <param name="maxTimestep"></param>
    /// <param name="pathTime"></param>
    /// <param name="legs"></param>
    //public void UpdateGaitChannels(float minTimestep, float maxTimestep, ref float pathTime, List<LegStuff> legs)
    //{
    //    //These values need to be calculated once on stride end and affect the timestep then until the next stride end
    //    float scaledTimestep = 0.0f;
    //    for (int i = 0; i < legs.Count; i++)
    //    {
    //        LegStuff leg = legs[i];
    //        if (!gaitController.Channel(i).IsStriding)
    //        {
    //            float lastLength = gaitController.Channel(i).LastLength;
    //            if (lastLength > 0.0f)
    //                scaledTimestep = Mathf.Max((Time.deltaTime * gaitController.Channel(i).MaxStance) / (lastLength * pathTime), scaledTimestep);
    //            else
    //                scaledTimestep = maxTimestep;
    //        }
    //    }

    //    //Scale the timestep further based on the speed ratio
    //    if (scaledTimestep > maxTimestep)
    //    {
    //        pathTime *= scaledTimestep / maxTimestep;
    //        scaledTimestep = maxTimestep;
    //    }
    //    else if (scaledTimestep < minTimestep)
    //    {
    //        pathTime *= scaledTimestep / minTimestep;
    //        scaledTimestep = minTimestep;
    //    }

    //    //Update the gait channels by the scaled timestep
    //    for (int i = 0; i < legs.Count; i++)
    //    {
    //        LegStuff leg = legs[i];
    //        gaitController.Channel(i).Update(scaledTimestep);
    //    }
    //}

    /// <summary>
    /// Apply the provided motion to the body transform.
    /// Motion is relative to the transform, so will first need to be converted if another position is to be the reference
    /// </summary>
    /// <param name="motion">The motion to apply</param>
    /// <param name="point">The point in local space that the motion is relative to</param>
    /// <param name="bodyTrans"></param>
    /// <param name="legs"></param>
    public void ApplyInverseMotion(Motion motion, MotionPoint point, Transform bodyTrans, List<LegStuff> legs, float heightDiff)
    {
        //Vector3 adjust = transform.position;
        //adjust.y -= fHeightDiff;
        //transform.position = adjust;

        float pointAngle = point.Angle;

        //Apply the root motion to the actual game object so that it appears to be walking around
        Vector3 bodyPosition = bodyTrans.position;
        if (motion.HasRotation())
        {
            float rotationAmount = motion.AngularVelocity * Time.deltaTime;

            Vector3 localRotOrigin = Util.Rotate(motion.RotationOrigin, pointAngle) + point.Position;
            Vector3 worldRotOrigin = bodyTrans.TransformPoint(localRotOrigin);
            bodyPosition = Util.Rotate(bodyPosition - worldRotOrigin, rotationAmount) + worldRotOrigin;
            bodyPosition.y -= heightDiff;
            bodyTrans.Rotate(0.0f, rotationAmount, 0.0f);

            Debug.DrawLine(worldRotOrigin, bodyPosition, Color.red);

            for (int i = 0; i < legs.Count; i++)
            {
                LegStuff leg = legs[i];
                if (!gaitController.IsStriding(i))
                {
                    leg.groundPosition = Util.Rotate(leg.groundPosition - localRotOrigin, -rotationAmount) + localRotOrigin;
                    leg.groundPosition.y += heightDiff;
                }
            }            
        }
        else
        {
            Vector3 translation = Util.Rotate(motion.LinearVelocity, pointAngle) * Time.deltaTime;            
            bodyPosition += bodyTrans.TransformDirection(translation);
            bodyPosition.y -= heightDiff;

            for (int i = 0; i < legs.Count; i++)
            {
                LegStuff leg = legs[i];
                if (!gaitController.IsStriding(i))
                {
                    leg.groundPosition -= translation;
                    leg.groundPosition.y += heightDiff;
                }
            }
        }

        //bodyPosition.y = bodyTrans.position.y;  //Reset the Y to whatever it was before
        bodyTrans.position = bodyPosition;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////