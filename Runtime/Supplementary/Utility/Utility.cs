using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Assertions;

namespace Unity.Kinematica
{
  /// <summary>
  /// Static class that contains various extension methods.
  /// </summary>
  public static class Utility
  {
    public static AffineTransform SampleTrajectoryAtTime(NativeSlice<AffineTransform> trajectory, float sampleTimeInSeconds, float timeHorizon)
    {
      int trajectoryLength = trajectory.Length;

      float adjustedTimeInSeconds =
        timeHorizon + sampleTimeInSeconds;

      float sampleRate = timeHorizon > 0.0f ? (trajectoryLength - 1) / (2.0f * timeHorizon) : 0.0f;

      float fractionalKeyFrame =
        adjustedTimeInSeconds * sampleRate;

      int sampleKeyFrame =
        Missing.truncToInt(fractionalKeyFrame);

      if (sampleKeyFrame < 0)
      {
        return trajectory[0];
      }
      else if (sampleKeyFrame >= trajectoryLength - 1)
      {
        return trajectory[trajectoryLength - 1];
      }

      float theta = math.saturate(fractionalKeyFrame - sampleKeyFrame);

      if (theta <= Missing.epsilon)
      {
        return trajectory[sampleKeyFrame];
      }

      AffineTransform t0 = trajectory[sampleKeyFrame + 0];
      AffineTransform t1 = trajectory[sampleKeyFrame + 1];

      return Missing.lerp(t0, t1, theta);
    }

    /// <summary>
    /// Returns the interpolated transform between the current and desired root motion, taking into account individual translation and rotation weights
    /// </summary>
    /// <param name="rootMotion">Root motion to steer, represents a root delta transform for a frame</param>
    /// <param name="desiredRootMotion">Desired root motion toward which we steer the current root motion</param>
    /// <param name="deltaTime">Delta time associated to the root motion. That means root velocity for the frame is <code>rootMotion.t / deltaTime</code></param>
    /// <param name="translationWeight">Interpolation weight between the root delta position at 0, and the desired delta position at 1</param>
    /// <param name="rotationWeight">Interpolation weight between the root delta rotation at 0, and the desired delta rotation at 1</param>
    /// <param name="startSpeed">Root speed (m/s) at which steering will start to be effective, steering weight will then increase linearly as root speed increases toward <code>endSpeed</code></param>
    /// <param name="endSpeed">Root speed (m/s) at which steering will be fully effective</param>
    /// <returns></returns>
    public static AffineTransform SteerRootMotion(AffineTransform rootMotion, AffineTransform desiredRootMotion, float deltaTime, float translationWeight, float rotationWeight, float startSpeed = 0.0f, float endSpeed = 0.15f)
    {
      if (deltaTime <= 0.0f)
      {
        return AffineTransform.identity;
      }

      AffineTransform steeredRootMotion = rootMotion;

      float speed = math.length(desiredRootMotion.t / deltaTime);
      float weight;
      if (endSpeed > startSpeed)
      {
        weight = math.clamp((speed - startSpeed) / (endSpeed - startSpeed), 0.0f, 1.0f);
      }
      else
      {
        weight = speed >= startSpeed ? 1.0f : 0.0f;
      }

      quaternion steerRotation = math.mul(desiredRootMotion.q, math.conjugate(rootMotion.q));
      steerRotation = math.slerp(quaternion.identity, steerRotation, rotationWeight * weight);
      steeredRootMotion.t = math.rotate(steerRotation, steeredRootMotion.t);
      steeredRootMotion.q = math.mul(steerRotation, steeredRootMotion.q);

      steeredRootMotion.t = math.lerp(steeredRootMotion.t, desiredRootMotion.t, translationWeight * weight);

      return steeredRootMotion;
    }
  }
}
