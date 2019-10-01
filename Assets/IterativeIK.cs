using UnityEngine;
using System.Collections.Generic;
using System.IO.Ports;
using System.Text;
using System;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public class IterativeIK : MonoBehaviour
{
    //--------------------------------------------------
    // Variables
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    protected string endName = "end";

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    protected string targetName = "target";

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    protected float maxError = 0.001f;

    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    protected uint maxPasses = 100;

    /// <summary>
    /// 
    /// </summary>
    private List<IterativeIKJoint> ikChain;

    /// <summary>
    /// 
    /// </summary>
    private Transform ikEnd;

    /// <summary>
    /// 
    /// </summary>
    private Transform ikTarget;


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    void Start()
    {
        FindChainToEnd();
        FindTargetRecursive(transform.root, ref ikTarget, targetName);
    }

    public void SendAngles(SerialPort port)
    {
        foreach (IterativeIKJoint joint in ikChain)
        {
            if ((joint.ID != IterativeIKJoint.ServoID.None) && !joint.ServoOutputSuppressed)
            {
                //string valueStr = string.Format("{0:0.00}", joint.GetMappedValue());
                //string toSend = '#' + ((int)joint.ID).ToString() + 'a' + valueStr + '\n';
                //port.Write(toSend);

                //Only works for up to 32 servos
                int pulse = (int)Mathf.Clamp((joint.GetValue() + 180.0f) * 10.0f, 0.0f, 3600.0f);
                int pulseHigh = (pulse & 0xFC0) >> 6 | 0x80;
                int pulseLow = (pulse & 0x3F) | 0xC0;

                byte[] toSend = new byte[3];
                toSend[0] = (byte)((int)joint.ID | 0x40);
                toSend[1] = (byte)pulseHigh;
                toSend[2] = (byte)pulseLow;
                port.Write(toSend, 0, 3);
            }
        }
    }

    public void SendShutdown(SerialPort port)
    {
        foreach (IterativeIKJoint joint in ikChain)
        {
            if (joint.ID != IterativeIKJoint.ServoID.None)
            {
                byte[] toSend = new byte[1];
                toSend[0] = (byte)((int)joint.ID | 0x60);
                port.Write(toSend, 0, 1);
            }
        }
    }


    /// <summary>
    /// 
    /// </summary>
    void FixedUpdate()
    {
        if (ikEnd != null && ikTarget != null)
        {
            //Restore all the joints in the chain to their initial state
            foreach (IterativeIKJoint joint in ikChain)
            {
                joint.InitialisePose();
            }

            uint pass;
            float beforeDiff = (ikEnd.position - ikTarget.position).sqrMagnitude;
            for (pass = 0; pass < maxPasses; pass++)
            {
                bool bChange = false;
                foreach (IterativeIKJoint joint in ikChain)
                {
                    bChange |= joint.AlignEndToTarget(ikEnd, ikTarget);
                }

                float afterDiff = (ikEnd.position - ikTarget.position).sqrMagnitude;
                if (!bChange || afterDiff <= (maxError * maxError) || Mathf.Abs(beforeDiff - afterDiff) <= (maxError * maxError))
                {
                    break;
                }
                beforeDiff = afterDiff;
            }

            for (int i = 1; i < ikChain.Count; i++)
                Debug.DrawLine(ikChain[i - 1].transform.position, ikChain[i].transform.position, Color.white);

            if (ikChain.Count > 0 && ikEnd != null)
                Debug.DrawLine(ikChain[ikChain.Count - 1].transform.position, ikEnd.position, Color.white);

            Debug.DrawLine(ikEnd.position, ikTarget.position, Color.red);
            //Debug.Log("Passes = " + pass.ToString());

            foreach (IterativeIKJoint joint in ikChain)
            {
                joint.ApplyResult();
            }
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="parent"></param>
    /// <param name="transformChain"></param>
    /// <param name="endPoint"></param>
    /// <param name="endTransName"></param>
    /// <returns></returns>
    private static bool FindEndRecursive(Transform parent, List<IterativeIKJoint> transformChain, ref Transform endPoint, string endTransName)
    {
        bool bEndFound = false;
        foreach (Transform child in parent)
        {
            if (child.gameObject.activeSelf)
            {
                if (child.name == endTransName)
                {
                    endPoint = child;
                    bEndFound = true;
                    break;
                }
                else if (bEndFound = FindEndRecursive(child, transformChain, ref endPoint, endTransName))
                {
                    IterativeIKJoint joint = child.GetComponent<IterativeIKJoint>();
                    if (joint != null)
                        transformChain.Insert(0, joint);
                    break;
                }
            }
        }
        return bEndFound;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="parent"></param>
    /// <param name="targetPoint"></param>
    /// <param name="targetTransName"></param>
    /// <returns></returns>
    private static bool FindTargetRecursive(Transform parent, ref Transform targetPoint, string targetTransName)
    {
        bool bTargetFound = false;
        foreach (Transform child in parent)
        {
            if (child.gameObject.activeSelf)
            {
                if (child.name == targetTransName)
                {
                    targetPoint = child;
                    bTargetFound = true;
                    break;
                }
                else if (bTargetFound = FindTargetRecursive(child, ref targetPoint, targetTransName))
                {
                    break;
                }
            }
        }
        return bTargetFound;
    }

    /// <summary>
    /// 
    /// </summary>
    private void FindChainToEnd()
    {
        ikChain = new List<IterativeIKJoint>();
        if (FindEndRecursive(transform, ikChain, ref ikEnd, endName))
        {
            IterativeIKJoint joint = transform.GetComponent<IterativeIKJoint>();
            if (joint != null)
                ikChain.Insert(0, joint);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////