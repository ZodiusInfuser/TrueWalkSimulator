using UnityEngine;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// A representation of a 2D motion (translation + angle).
/// Includes a number of conversion functions for motion points.
/// </summary>
public class Motion
{
    //--------------------------------------------------
    // Constructors
    //--------------------------------------------------

    /// <summary>
    /// Private constructor to create a motion with pre-computed values.
    /// </summary>
    /// <param name="linearVelocity"></param>
    /// <param name="angularVelocity"></param>
    /// <param name="rotationOrigin"></param>
    private Motion(Vector3 linearVelocity, float angularVelocity, Vector3 rotationOrigin)
    {
        LinearVelocity = linearVelocity;
        AngularVelocity = angularVelocity;
        RotationOrigin = rotationOrigin;
    }    

    /// <summary>
    /// Create a new motion with the given linear and angular velocity components.
    /// </summary>
    /// <param name="xVelocity">Linear velocity in the X direction</param>
    /// <param name="zVelocity">Linear velocity in the Z direction</param>
    /// <param name="angVelocity">Angular velocity around the Y axis</param>
    public Motion(float xVelocity, float zVelocity, float angVelocity)
    {
        //Assign the linear and angular velocities
        LinearVelocity = new Vector3(xVelocity, 0.0f, zVelocity);
        AngularVelocity = angVelocity;

        if (LinearVelocity.magnitude > 0.001f)
        {
            if (Mathf.Abs(AngularVelocity) > 0.001f)
            {
                //Calculate the rotation origin based on the direction of the angular velocity
                float pathFactor = Mathf.Rad2Deg / Mathf.Abs(AngularVelocity);
                if (AngularVelocity > 0.0f)
                    RotationOrigin = Util.RotatePlus90(LinearVelocity) * pathFactor;
                else
                    RotationOrigin = Util.RotateMinus90(LinearVelocity) * pathFactor;
            }
            else
            {
                RotationOrigin = new Vector3(float.NaN, 0.0f, float.NaN);
            }
        }
        else
        {
            //Rotation is either around the origin or NaN
            if (Mathf.Abs(AngularVelocity) > 0.001f)
                RotationOrigin = Vector3.zero;
            else
                RotationOrigin = new Vector3(float.NaN, 0.0f, float.NaN);
        }
    }


    //--------------------------------------------------
    // Properties
    //--------------------------------------------------

    /// <summary>
    /// The linear velocity of this motion as a Vector3 (y axis is zero).
    /// </summary>
    public Vector3 LinearVelocity { get; private set; }

    /// <summary>
    /// The angular velocity of this motion.
    /// </summary>
    public float AngularVelocity { get; private set; }

    /// <summary>
    /// The origin around which the angular velocity is applied (y axis is zero).
    /// </summary>
    public Vector3 RotationOrigin { get; private set; }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// Checks if this motion has a linear velocity component.
    /// </summary>
    /// <returns>True if there is a translation</returns>
    public bool HasTranslation()
    {
        return Mathf.Abs(LinearVelocity.magnitude) > 0.001f;
    }

    /// <summary>
    /// Checks if this motion has angular velocity and rotation origin components.
    /// </summary>
    /// <returns>True if there is a rotation</returns>
    public bool HasRotation()
    {
        return Mathf.Abs(AngularVelocity) > 0.0f && !float.IsNaN(RotationOrigin.x) && !float.IsNaN(RotationOrigin.z);
    }

    /// <summary>
    /// Converts this motion from being relative to one motion point to being relative to another
    /// </summary>
    /// <param name="currentPoint">The point this motion is relative to</param>
    /// <param name="newPoint">The point the returned motion is to be relative to</param>
    /// <returns>A new motion that is relative to the newPoint</returns>
    public Motion Convert(MotionPoint currentPoint, MotionPoint newPoint)
    {
        Vector3 newLinVelocity, newRotOrigin;

        if (HasRotation())
        {
            //Compute the new rotation origin
            Vector3 localRotOrigin = Util.Rotate(RotationOrigin, currentPoint.Angle) + currentPoint.Position;
            newRotOrigin = Util.Rotate(localRotOrigin - newPoint.Position, -newPoint.Angle);

            //Calculate the new linear velocity
            float newPathRadius = newRotOrigin.magnitude;
            if (newPathRadius > 0.0f)
            {
                float arcLength = newPathRadius * AngularVelocity * Mathf.Deg2Rad;
                newLinVelocity = Util.RotateMinus90(newRotOrigin).normalized * arcLength;
            }
            else
            {
                newLinVelocity = Vector3.zero;
            }
        }
        else
        {
            //Calculate the new linear velocity and set the rotation origin to NaN
            newLinVelocity = Util.Rotate(LinearVelocity, currentPoint.Angle - newPoint.Angle);
            newRotOrigin = new Vector3(float.NaN, 0.0f, float.NaN);
        }

        return new Motion(newLinVelocity, AngularVelocity, newRotOrigin);
    }

    /// <summary>
    /// Computes the motion that would move the start point to the end point, relative to the start point.
    /// </summary>
    /// <param name="start">The point the motion is from</param>
    /// <param name="end">The point the motion is to</param>
    /// <returns>A new motion that would move the start point to the end point</returns>
    public static Motion Between(MotionPoint start, MotionPoint end)
    {
        Vector3 newLinVelocity, newRotOrigin;

        //Get the difference between the two points
        Vector3 distDiff = end.Position - start.Position;
        float angleDiff = Util.WrapAngle(end.Angle - start.Angle);

        //Is there a linear separation between the two points?
        float chordLength = distDiff.magnitude;
        if (chordLength > 0.001f)
        {
            //Is there an angular separation between the two points?
            if (Mathf.Abs(angleDiff) > 0.001f)
            {
                //Use the distance between the two points as the chord length for calculating the new rotation origin
                float halfChord = chordLength / 2.0f;
                float pathRadius = halfChord / Mathf.Sin(0.5f * Mathf.Abs(angleDiff) * Mathf.Deg2Rad);
                float hypotenuseFactor = Mathf.Sqrt(Mathf.Max(pathRadius * pathRadius - halfChord * halfChord, 0.0f)) / chordLength;

                newRotOrigin = (end.Position + start.Position) / 2.0f;
                if (angleDiff > 0.0f)
                    newRotOrigin += Util.RotatePlus90(distDiff) * hypotenuseFactor;
                else
                    newRotOrigin += Util.RotateMinus90(distDiff) * hypotenuseFactor;

                //Calculate the new linear velocity 
                float arcLength = pathRadius * angleDiff * Mathf.Deg2Rad;
                Vector3 velocityPoint = Util.RotateMinus90(newRotOrigin - start.Position).normalized * arcLength;
                newLinVelocity = new Vector3(Vector3.Dot(velocityPoint, start.Side), 0.0f, Vector3.Dot(velocityPoint, start.Forward));
            }
            else
            {
                //Calculate the new linear velocity and set the rotation origin to NaN
                newLinVelocity = new Vector3(Vector3.Dot(distDiff, start.Side), 0.0f, Vector3.Dot(distDiff, start.Forward));
                newRotOrigin = new Vector3(float.NaN, 0.0f, float.NaN);
            }
        }
        else
        {
            //There's no linear velocity
            newLinVelocity = Vector3.zero;

            //Rotation origin is either around the start/end point (they're the same), or NaN
            if (Mathf.Abs(angleDiff) > 0.001f)
                newRotOrigin = start.Position;
            else
                newRotOrigin = new Vector3(float.NaN, 0.0f, float.NaN);
        }

        return new Motion(newLinVelocity, angleDiff, newRotOrigin);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////