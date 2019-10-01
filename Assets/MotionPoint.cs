using UnityEngine;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// Represents a position and rotation in 2D space relative to a frame of reference.
/// By default a new point will be in local space, but if a parent point is provided to it,
/// it will become relative to that. This allows for any motions applied to a parent point
/// to be easily propogated to child points.
/// 
/// For convenience it is possible to set a point's position and rotation in local space whilst
/// also giving it a parent point, and internally the correct relative values will be assigned.
/// 
/// Note that a motion point does not keep a reference to any parent point provided.
/// </summary>
public class MotionPoint
{
    //--------------------------------------------------
    // Constants
    //--------------------------------------------------

    /// <summary>
    /// 
    /// </summary>
    private const float AXIS_LENGTH = 0.5f;


    //--------------------------------------------------
    // Constructors
    //--------------------------------------------------

    /// <summary>
    /// Creates a new motion point with the provided position and rotation in local space.
    /// </summary>
    /// <param name="x">The local x position</param>
    /// <param name="z">The local z position</param>
    /// <param name="angle">The local angle</param>
    public MotionPoint(float x, float z, float angle)
    {
        Set(x, z, angle);
    }

    /// <summary>
    /// Creates a new motion point with the provided position and rotation,
    /// either in local space, or relative to the provided parent.
    /// </summary>
    /// <param name="x">The local/relative x position</param>
    /// <param name="z">The local/relative z position</param>
    /// <param name="angle">The local/relative angle</param>
    /// <param name="parent">The motion point this one will internally be relative to</param>
    /// <param name="isRelative">Whether or not the position and rotation are relative (defaults to true)</param>
    //public MotionPoint(float x, float z, float angle, MotionPoint parent, bool isRelative = true)
    //{
    //    if (isRelative)
    //        SetRelative(x, z, angle, parent);
    //    else
    //        SetLocal(x, z, angle, parent);
    //}


    //--------------------------------------------------
    // Properties
    //--------------------------------------------------

    /// <summary>
    /// The relative x position of this point
    /// </summary>
    public float RelativeX { get; private set; }

    /// <summary>
    /// The relative z position of this point
    /// </summary>
    public float RelativeZ { get; private set; }

    /// <summary>
    /// The relative angle of this point
    /// </summary>
    public float RelativeAngle { get; private set; }

    /// <summary>
    /// The position of this point
    /// </summary>
    public Vector3 Position { get; private set; }

    /// <summary>
    /// The local forward axis of this point
    /// </summary>
    public Vector3 Forward { get; private set; }

    /// <summary>
    /// The local side axis of this point
    /// </summary>
    public Vector3 Side { get; private set; }

    /// <summary>
    /// The local angle of this point
    /// </summary>
    public float Angle
    {
        get { return Mathf.Atan2(Forward.x, Forward.z) * Mathf.Rad2Deg; }
    }

    /// <summary>
    /// The baked matrix calculated when new relative coordinates are set
    /// </summary>
    private Matrix4x4 BakedMatrix { get; set; }


    //--------------------------------------------------
    // Static Properties
    //--------------------------------------------------

    /// <summary>
    /// A motion point void of any motion
    /// </summary>
    public static MotionPoint Identity
    {
        get { return new MotionPoint(0, 0, 0); }
    }


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------

    /// <summary>
    /// Sets the position and rotation of this point in local space.
    /// </summary>
    /// <param name="x">The local x position</param>
    /// <param name="z">The local z position</param>
    /// <param name="angle">The local angle</param>
    public void Set(float x, float z, float angle)
    {
        RelativeX = x;
        RelativeZ = z;
        RelativeAngle = angle;
        Bake(Matrix4x4.identity);
    }

    /// <summary>
    /// Sets the position and rotation of this point in local space,
    /// but internally relative to the provided parent.
    /// </summary>
    /// <param name="x">The local x position</param>
    /// <param name="z">The local z position</param>
    /// <param name="angle">The local angle</param>
    /// <param name="parent">The motion point this one will internally be relative to</param>
    public void SetLocal(float x, float z, float angle, MotionPoint parent)
    {
        float parentAngle = parent.Angle;
        Vector3 inverse = Util.Rotate(new Vector3(x, 0.0f, z) - parent.Position, -parentAngle);

        RelativeX = inverse.x;
        RelativeZ = inverse.z;
        RelativeAngle = angle - parentAngle;
        Bake(parent.BakedMatrix);
    }

    /// <summary>
    /// Sets the position and rotation of this point, relative to the provided parent.
    /// </summary>
    /// <param name="x">The relative x position</param>
    /// <param name="z">The relative z position</param>
    /// <param name="angle">The relative angle</param>
    /// <param name="parent">The motion point this one will be relative to</param>
    public void SetRelative(float x, float z, float angle, MotionPoint parent)
    {
        RelativeX = x;
        RelativeZ = z;
        RelativeAngle = angle;
        Bake(parent.BakedMatrix);
    }

    /// <summary>
    /// Creates a new motion point with the provided position and rotation in local space,
    /// but internally relative to the provided parent.
    /// </summary>
    /// <param name="x">The local x position</param>
    /// <param name="z">The local z position</param>
    /// <param name="angle">The local angle</param>
    /// <param name="parent">The motion point the created one will internally be relative to</param>
    /// <returns>A new motion point</returns>
    public static MotionPoint Local(float x, float z, float angle, MotionPoint parent)
    {
        MotionPoint point = Identity;
        point.SetLocal(x, z, angle, parent);
        return point;
    }

    /// <summary>
    /// Creates a new motion point with the provided position and rotation, relative to the provided parent.
    /// </summary>
    /// <param name="x">The relative x position</param>
    /// <param name="z">The relative z position</param>
    /// <param name="angle">The relative angle</param>
    /// <param name="parent">The motion point the created one will be relative to</param>
    /// <returns>A new motion point</returns>
    public static MotionPoint Relative(float x, float z, float angle, MotionPoint parent)
    {
        MotionPoint point = Identity;
        point.SetRelative(x, z, angle, parent);
        return point;
    }

    /// <summary>
    /// Sets this point to the same position and rotation in local space,
    /// but internally relative to the provided parent.
    /// </summary>
    /// <param name="newParent">The motion point that will be the new parent</param>
    public void SetLocalTo(MotionPoint newParent)
    {
        SetLocal(Position.x, Position.z, Angle, newParent);
    }

    /// <summary>
    /// Sets this point to be relative to the provided parent point.
    /// </summary>
    /// <param name="newParent">The motion point that will be the new parent</param>
    public void SetRelativeTo(MotionPoint newParent)
    {
        SetRelative(RelativeX, RelativeZ, RelativeAngle, newParent);
    }

    /// <summary>
    /// Creates a copy of this point with the same position and rotation in local space,
    /// but internally relative to the provided parent.
    /// </summary>
    /// <param name="newParent">The motion point that will be the new parent</param>
    /// <returns>A new motion point</returns>
    public MotionPoint LocalTo(MotionPoint newParent)
    {
        //Workaround for situation where zeros get passed in rather than the relative values
        float x = Position.x;
        float z = Position.z;
        float a = Angle;
        return Local(x, z, a, newParent);
    }

    /// <summary>
    /// Creates a copy of this point that is relative to the provided parent point.
    /// </summary>
    /// <param name="newParent">The motion point that will be the new parent</param>
    /// <returns>A new motion point</returns>
    public MotionPoint RelativeTo(MotionPoint newParent)
    {
        //Workaround for situation where zeros get passed in rather than the relative values
        float x = RelativeX;
        float z = RelativeZ;
        float a = RelativeAngle;
        return Relative(x, z, a, newParent);
        //return Relative(RelativeX, RelativeZ, RelativeAngle, newParent);
    }

    //public MotionPoint RelativeTo(MotionPoint newParent)
    //{
    //    return Relative(RelativeX, RelativeZ, RelativeAngle, newParent);
    //}

    /// <summary>
    /// Applies the provided local motion to this point.
    /// This point will loose any relation it had to a parent point.
    /// </summary>
    /// <param name="motion">The local motion to apply</param>
    public void SetLocalMove(Motion motion)
    {
        //Get the angle of the two points
        float startAngle = Angle;
        float endAngle = startAngle + motion.AngularVelocity;

        if (motion.HasTranslation())
        {
            //Calculate the end position in local space
            Vector3 endPosition;
            if (motion.HasRotation())
                endPosition = Util.Rotate(Position - motion.RotationOrigin, motion.AngularVelocity) + motion.RotationOrigin;
            else
                endPosition = motion.LinearVelocity + Position;

            Set(endPosition.x, endPosition.z, endAngle);
        }
        else
        {
            Set(Position.x, Position.z, endAngle);
        }
    }

    /// <summary>
    /// Applies the provided local motion to this point.
    /// </summary>
    /// <param name="motion">The local motion to apply</param>
    /// <param name="parent">The motion point this one will internally be relative to</param>
    public void SetLocalMove(Motion motion, MotionPoint parent)
    {
        SetLocalMove(motion);
        SetLocalTo(parent);
    }

    /// <summary>
    /// Applies the provided relative motion to this point.
    /// This point will loose any relation it had to a parent point.
    /// </summary>
    /// <param name="motion">The relative motion to apply</param>
    public void SetRelativeMove(Motion motion)
    {
        //Get the angle of the two points
        float startAngle = Angle;
        float endAngle = startAngle + motion.AngularVelocity;

        if (motion.HasTranslation())
        {
            //Calculate the end position relative to the start point
            Vector3 endPosition;
            if (motion.HasRotation())
            {
                Vector3 localRotOrigin = Util.Rotate(motion.RotationOrigin, startAngle) + Position;
                endPosition = Util.Rotate(Position - localRotOrigin, motion.AngularVelocity) + localRotOrigin;
            }
            else
            {
                endPosition = Util.Rotate(motion.LinearVelocity, startAngle) + Position;
            }

            Set(endPosition.x, endPosition.z, endAngle);
        }
        else
        {
            Set(Position.x, Position.z, endAngle);
        }
    }

    /// <summary>
    /// Applies the provided relative motion to this point.
    /// </summary>
    /// <param name="motion">The relative motion to apply</param>
    /// <param name="parent">The motion point this one will internally be relative to</param>
    public void SetRelativeMove(Motion motion, MotionPoint parent)
    {
        SetRelativeMove(motion);
        SetLocalTo(parent);
    }

    /// <summary>
    /// Computes a new motion point by applying the provided local motion to this point.
    /// The returned point will loose any relation it had to a parent point.
    /// </summary>
    /// <param name="motion">The local motion to apply</param>
    /// <returns>A new motion point</returns>
    public MotionPoint LocalMove(Motion motion)
    {
        MotionPoint newPoint;

        //Get the angle of the two points
        float startAngle = Angle;
        float endAngle = startAngle + motion.AngularVelocity;

        if (motion.HasTranslation())
        {
            //Calculate the end position in local space
            Vector3 endPosition;
            if (motion.HasRotation())
                endPosition = Util.Rotate(Position - motion.RotationOrigin, motion.AngularVelocity) + motion.RotationOrigin;
            else
                endPosition = motion.LinearVelocity + Position;

            newPoint = new MotionPoint(endPosition.x, endPosition.z, endAngle);
        }
        else
        {
            newPoint = new MotionPoint(Position.x, Position.z, endAngle);
        }

        return newPoint;
    }

    /// <summary>
    /// Computes a new motion point by applying the provided local motion to this point.
    /// </summary>
    /// <param name="motion">The local motion to apply</param>
    /// <param name="parent">The motion point the new one will internally be relative to</param>
    /// <returns>A new motion point</returns>
    public MotionPoint LocalMove(Motion motion, MotionPoint parent)
    {
        MotionPoint newPoint = LocalMove(motion);
        newPoint.SetLocalTo(parent);
        return newPoint;
    }

    /// <summary>
    /// Computes a new motion point by applying the provided relative motion to this point.
    /// The returned point will loose any relation it had to a parent point.
    /// </summary>
    /// <param name="motion">The relative motion to apply</param>
    /// <returns>A new motion point</returns>
    public MotionPoint RelativeMove(Motion motion)
    {
        MotionPoint newPoint;

        //Get the angle of the two points
        float startAngle = Angle;
        float endAngle = startAngle + motion.AngularVelocity;

        if (motion.HasTranslation())
        {
            //Calculate the end position relative to the start point
            Vector3 endPosition;
            if (motion.HasRotation())
            {
                Vector3 localRotOrigin = Util.Rotate(motion.RotationOrigin, startAngle) + Position;
                endPosition = Util.Rotate(Position - localRotOrigin, motion.AngularVelocity) + localRotOrigin;
            }
            else
            {
                endPosition = Util.Rotate(motion.LinearVelocity, startAngle) + Position;
            }

            newPoint = new MotionPoint(endPosition.x, endPosition.z, endAngle);
        }
        else
        {
            newPoint = new MotionPoint(Position.x, Position.z, endAngle);
        }

        return newPoint;
    }

    /// <summary>
    /// Computes a new motion point by applying the provided relative motion to this point.
    /// </summary>
    /// <param name="motion">The relative motion to apply</param>
    /// <param name="parent">The motion point the new one will internally be relative to</param>
    /// <returns>A new motion point</returns>
    public MotionPoint RelativeMove(Motion motion, MotionPoint parent)
    {
        MotionPoint newPoint = RelativeMove(motion);
        newPoint.SetLocalTo(parent);
        return newPoint;
    }

    /// <summary>
    /// Computes the baked matrix transform and vectors of this point in local space.
    /// </summary>
    /// <param name="parentMatrix">The internal baked matrix of the parent</param>
    private void Bake(Matrix4x4 parentMatrix)
    {
        BakedMatrix = parentMatrix * Matrix4x4.Translate(new Vector3(RelativeX, 0.0f, RelativeZ)) * Matrix4x4.Rotate(Quaternion.Euler(0, RelativeAngle, 0));

        Position = new Vector3(BakedMatrix.GetColumn(3).x, BakedMatrix.GetColumn(3).y, BakedMatrix.GetColumn(3).z);
        Forward = new Vector3(BakedMatrix.GetColumn(2).x, BakedMatrix.GetColumn(2).y, BakedMatrix.GetColumn(2).z);
        Side = new Vector3(BakedMatrix.GetColumn(0).x, BakedMatrix.GetColumn(0).y, BakedMatrix.GetColumn(0).z);
    }

    /// <summary>
    /// Create a motion point from a provided transform, that is relative to a base transform.
    /// The returned MotionPoint will be local to a parent MotionPoint
    /// </summary>
    /// <param name="baseTrans"></param>
    /// <param name="trans"></param>
    /// <param name="parent"></param>
    /// <returns></returns>
    public static MotionPoint FromTransform(Transform baseTrans, Transform trans, MotionPoint parent)
    {
        Vector3 localPos; Quaternion localRot;
        Util.CalcRelativeTo(baseTrans, trans, out localPos, out localRot);
        return Local(localPos.x, localPos.z, localRot.eulerAngles.y, parent);
    }

    /// <summary>
    /// Debug renders coordinate axes of this point at its world position.
    /// </summary>
    /// <param name="transform"></param>
    public void Render(Transform transform)
    {
        Vector3 worldPosition = transform.TransformPoint(Position);
        Vector3 worldForward = transform.TransformDirection(Forward);
        Vector3 worldSide = transform.TransformDirection(Side);
        Vector3 worldUp = transform.TransformDirection(Vector3.up);

        Debug.DrawRay(worldPosition, worldForward * AXIS_LENGTH, Color.blue);
        Debug.DrawRay(worldPosition, worldSide * AXIS_LENGTH, Color.red);
        Debug.DrawRay(worldPosition, worldUp * AXIS_LENGTH, Color.green);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////