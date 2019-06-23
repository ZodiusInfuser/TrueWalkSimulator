using System.Collections.Generic;
using UnityEngine;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public class Gait
{
    private List<GaitStride>[] _strideSets;


    //--------------------------------------------------
    // Constructors
    //--------------------------------------------------

    public Gait(int legs)
    {
        LegCount = legs;
        _strideSets = new List<GaitStride>[legs];
        for (int i = 0; i < _strideSets.Length; i++)
        {
            _strideSets[i] = new List<GaitStride>();
        }
    }

    //--------------------------------------------------
    // Properties
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    public int LegCount { get; private set; }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    /// <param name="stride"></param>
    public void AddStride(int legIndex, GaitStride stride)
    {
        _strideSets[legIndex].Add(stride);
    }

    public List<GaitStride> GetStrides(int legIndex)
    {
        return _strideSets[legIndex];
    }

    public float MaxStance()
    {
        float maxStance = 0.0f;
        foreach (List<GaitStride> strides in _strideSets)
        {
            for (int i = 0; i < strides.Count; i++)
            {
                int next = (i + 1) % strides.Count;
                maxStance = Mathf.Max(GaitStride.StanceLength(strides[i], strides[next]), maxStance);
            }
        }
        return maxStance;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////