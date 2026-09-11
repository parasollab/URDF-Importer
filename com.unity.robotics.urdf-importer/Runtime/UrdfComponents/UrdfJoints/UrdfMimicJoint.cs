using UnityEngine;

namespace Unity.Robotics.UrdfImporter
{
    /// <summary>
    /// Keeps one joint locked to another: `position = multiplier * leader + offset`, the
    /// relationship a URDF &lt;mimic&gt; tag describes.
    ///
    /// Only follower joints carry this component, so joints with no coupling pay nothing for
    /// it. It is attached during import by <see cref="UrdfRobot.ResolveMimicJoints"/>, and can
    /// also be added by hand to couple joints a description left independent - the four
    /// telescoping sections of a Stretch arm, say, which real hardware drives as one axis.
    /// </summary>
    public class UrdfMimicJoint : MonoBehaviour
    {
        [Tooltip("The joint being driven - this joint.")]
        public UrdfJoint follower;

        [Tooltip("The joint whose position is followed.")]
        public UrdfJoint leader;

        [Tooltip("Scale applied to the leader's position.")]
        public float multiplier = 1f;

        [Tooltip("Added after scaling, in the follower's units (radians or meters).")]
        public float offset;

        /// <summary>Attaches or updates the coupling on <paramref name="follower"/>.</summary>
        public static UrdfMimicJoint Attach(UrdfJoint follower, UrdfJoint leader, float multiplier, float offset)
        {
            if (follower == null || leader == null)
            {
                return null;
            }

            var mimicJoint = follower.gameObject.GetComponent<UrdfMimicJoint>();
            if (mimicJoint == null)
            {
                mimicJoint = follower.gameObject.AddComponent<UrdfMimicJoint>();
            }

            mimicJoint.follower = follower;
            mimicJoint.leader = leader;
            mimicJoint.multiplier = multiplier;
            mimicJoint.offset = offset;
            return mimicJoint;
        }

        void FixedUpdate()
        {
            if (follower == null || leader == null)
            {
                return;
            }

            follower.SetPosition(multiplier * leader.GetPosition() + offset);
        }
    }
}
