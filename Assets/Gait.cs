using System.Collections.Generic;
using UnityEngine;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// A definition of a walking gait, containing lists of strides for a specified set of legs
/// </summary>
public class Gait
{
    //--------------------------------------------------
    // Variables
    //--------------------------------------------------

    /// <summary>
    /// An array of lists of GaitStrides
    /// </summary>
    private List<GaitStride>[] _strideSets;


    //--------------------------------------------------
    // Constructors
    //--------------------------------------------------

    /// <summary>
    /// Creates a new gait for the provided number of legs
    /// </summary>
    /// <param name="legs">The number of legs this gait is for</param>
    public Gait(int legs)
    {
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
    /// The number of legs this gait is intended to operate on
    /// </summary>
    public int LegCount
    {
        get { return _strideSets.Length; }
    }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// Adds a GaitStride for the specified leg
    /// </summary>
    /// <param name="legIndex">The leg to add the stride to</param>
    /// <param name="stride">The stride definition to add</param>
    public void AddStride(int legIndex, GaitStride stride)
    {
        _strideSets[legIndex].Add(stride);
    }

    /// <summary>
    /// Gets the list of strides for the specified leg
    /// </summary>
    /// <param name="legIndex">The leg to get the strides of</param>
    /// <returns>A list of GaitStrides</returns>
    public List<GaitStride> GetStrides(int legIndex)
    {
        return _strideSets[legIndex];
    }

    /// <summary>
    /// Calculates the duration of the longest stance any leg in this gait will experience
    /// </summary>
    /// <returns>The maximum stance</returns>
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