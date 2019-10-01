////////////////////////////////////////////////////////////////////////////////////////////////////
// Structures
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// A definition of a single stride of a leg in a walking gait. Times for strides are within a looping 0 to 1 range, called the cycle time
/// </summary>
public struct GaitStride
{
    //--------------------------------------------------
    // Constructors
    //--------------------------------------------------

    /// <summary>
    /// Creates a new gait stride
    /// </summary>
    /// <param name="start">The start cycle time of the stride (between 0 and 1)</param>
    /// <param name="end">The end cycle time of the stride (between 0 and 1). If less than the start time the value will be wrapped around</param>
    public GaitStride(float start, float end)
    {
        //Limit the start and end to be within 0 to 1
        Start = ((start >= 0.0f) && (start < 1.0f)) ? start : 0.0f;
        TrueEnd = ((end >= 0.0f) && (end < 1.0f)) ? end : 0.0f;

        End = (TrueEnd >= Start) ? TrueEnd : (TrueEnd + 1.0f);
    }


    //--------------------------------------------------
    // Properties
    //--------------------------------------------------

    /// <summary>
    /// The start cycle time of the stride
    /// </summary>
    public float Start { get; private set; }

    /// <summary>
    /// The end cycle time of the stride, wrapped around to be after the start time
    /// </summary>
    public float End { get; private set; }

    /// <summary>
    /// The true end cycle time of the stride
    /// </summary>
    private float TrueEnd { get; set; }

    /// <summary>
    /// The cycle time between the start and end of the stride
    /// </summary>
    public float StrideLength
    {
        get { return End - Start; }
    }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// Checks whether the provided cycle time is within the start and end of this stride
    /// </summary>
    /// <param name="cycleTime">The cycle time to check</param>
    /// <returns>True if within the stride, false otherwise</returns>
    public bool Within(float cycleTime)
    {
        return (cycleTime >= Start) && (cycleTime < End);
    }

    /// <summary>
    /// Calculates the time between the end of the first provided stride and the start of the second provided stride.
    /// These two strides should be neighbours
    /// </summary>
    /// <param name="first">The first stride</param>
    /// <param name="second">The second stride</param>
    /// <returns>The stance length</returns>
    public static float StanceLength(GaitStride first, GaitStride second)
    {
        return ((second.Start >= first.TrueEnd) ? second.Start : second.Start + 1.0f) - first.TrueEnd;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////