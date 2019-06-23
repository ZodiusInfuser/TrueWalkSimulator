////////////////////////////////////////////////////////////////////////////////////////////////////
// Structures
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// 
/// </summary>
public struct GaitStride
{
    //--------------------------------------------------
    // Constructors
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    /// <param name="start"></param>
    /// <param name="end"></param>
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
    /// 
    /// </summary>
    public float Start { get; private set; }

    /// <summary>
    /// 
    /// </summary>
    public float End { get; private set; }
    
    /// <summary>
    /// 
    /// </summary>
    private float TrueEnd { get; set; }

    /// <summary>
    /// 
    /// </summary>
    public float StrideLength
    {
        get { return End - Start; }
    }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    /// <param name="value"></param>
    /// <returns></returns>
    public bool Within(float value)
    {
        return (value >= Start) && (value < End);
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="first"></param>
    /// <param name="second"></param>
    /// <returns></returns>
    public static float StanceLength(GaitStride first, GaitStride second)
    {
        return ((second.Start >= first.TrueEnd) ? second.Start : second.Start + 1.0f) - first.TrueEnd;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////