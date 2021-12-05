using Unity.Collections;

namespace Unity.Kinematica
{
    public interface IMotionMatchingQuery
    {
        string DebugTitle { get; }

        FixedString64Bytes DebugName { get; }
    }
}
