using UnityEngine;

namespace Unity.Robotics.UrdfImporter.Control
{
    /// <summary>
    /// Drives a mobile base's three planar DOFs.
    ///
    /// ROS commands a mobile base by *velocity* - a geometry_msgs/Twist on /cmd_vel - and
    /// reports its position separately through odometry. The two are not symmetric, so this
    /// component exposes both: <see cref="SetTwist"/> for the command path, which is
    /// integrated locally so an operator sees motion immediately rather than after a network
    /// round trip, and <see cref="SetPose"/> for the authoritative correction that arrives
    /// from /odom or the map -> odom -> base_link transform chain.
    ///
    /// Everything here is in ROS convention: x forward, y left, yaw counter-clockwise, in
    /// meters and radians. It deliberately holds no ROS types, so the importer package stays
    /// independent of any particular ROS transport.
    /// </summary>
    [RequireComponent(typeof(UrdfPlanarBase))]
    public class UrdfBaseController : MonoBehaviour
    {
        [Tooltip("Whether the base can translate sideways. True for an omnidirectional base " +
                 "such as Stretch 4; false for a differential-drive base, whose linear.y is " +
                 "then ignored rather than silently obeyed.")]
        public bool holonomic;

        [Tooltip("Clamp applied to commanded linear velocity, in m/s. Zero disables the clamp.")]
        public float maxLinearSpeed = 1.0f;

        [Tooltip("Clamp applied to commanded angular velocity, in rad/s. Zero disables the clamp.")]
        public float maxAngularSpeed = 2.0f;

        [Tooltip("How quickly a pose correction from odometry is blended in, in units per " +
                 "second. Zero snaps immediately.")]
        public float poseCorrectionRate = 5f;

        UrdfPlanarBase m_PlanarBase;

        // Locally integrated estimate: (x, y, yaw) in the odom frame.
        Vector3 m_Pose;
        // Latest twist command: (vx, vy, wz).
        Vector3 m_Twist;
        // Latest authoritative pose, and whether one is pending.
        Vector3 m_TargetPose;
        bool m_HasPoseCorrection;

        /// <summary>Locally integrated base pose as (x, y, yaw).</summary>
        public Vector3 Pose => m_Pose;

        /// <summary>Twist most recently commanded, as (vx, vy, wz).</summary>
        public Vector3 Twist => m_Twist;

        void Awake()
        {
            m_PlanarBase = GetComponent<UrdfPlanarBase>();
        }

        /// <summary>
        /// Commands a base velocity, as a ROS Twist would. Publish the same values to
        /// /cmd_vel to keep the real robot and this preview in step.
        /// </summary>
        /// <param name="linearX">Forward velocity, m/s.</param>
        /// <param name="linearY">Leftward velocity, m/s. Ignored unless <see cref="holonomic"/>.</param>
        /// <param name="angularZ">Yaw rate, rad/s, counter-clockwise positive.</param>
        public void SetTwist(float linearX, float linearY, float angularZ)
        {
            if (!holonomic && Mathf.Abs(linearY) > Mathf.Epsilon)
            {
                // A differential-drive base physically cannot do this. Dropping it is more
                // honest than integrating a motion the real robot will not perform.
                linearY = 0f;
            }

            var linear = new Vector2(linearX, linearY);
            if (maxLinearSpeed > 0f && linear.magnitude > maxLinearSpeed)
            {
                linear = linear.normalized * maxLinearSpeed;
            }

            if (maxAngularSpeed > 0f)
            {
                angularZ = Mathf.Clamp(angularZ, -maxAngularSpeed, maxAngularSpeed);
            }

            m_Twist = new Vector3(linear.x, linear.y, angularZ);
        }

        /// <summary>Stops the base. Equivalent to commanding a zero twist.</summary>
        public void Stop()
        {
            m_Twist = Vector3.zero;
        }

        /// <summary>
        /// Supplies the authoritative base pose, as reported by odometry or TF. Blended in at
        /// <see cref="poseCorrectionRate"/> so a correction does not jolt the view.
        /// </summary>
        public void SetPose(float x, float y, float yawRadians)
        {
            m_TargetPose = new Vector3(x, y, yawRadians);
            m_HasPoseCorrection = true;
        }

        /// <summary>
        /// Jumps the base straight to a pose, skipping both the drives and the blend. Use it
        /// to seed a starting pose or to scrub a trajectory.
        /// </summary>
        public void SetPoseImmediate(float x, float y, float yawRadians)
        {
            m_Pose = new Vector3(x, y, yawRadians);
            m_Twist = Vector3.zero;
            m_HasPoseCorrection = false;
            m_PlanarBase?.SetPoseImmediate(x, y, yawRadians);
        }

        void FixedUpdate()
        {
            if (m_PlanarBase == null || !m_PlanarBase.IsValid)
            {
                return;
            }

            float dt = Time.fixedDeltaTime;

            IntegrateTwist(dt);
            ApplyPoseCorrection(dt);

            m_PlanarBase.SetPose(m_Pose.x, m_Pose.y, m_Pose.z);
        }

        void IntegrateTwist(float dt)
        {
            if (m_Twist == Vector3.zero)
            {
                return;
            }

            // The twist is expressed in the base frame, so rotate it into the odom frame
            // before integrating. Using the mid-interval heading keeps a simultaneous
            // translation and rotation from bowing away from the true arc.
            float midYaw = m_Pose.z + 0.5f * m_Twist.z * dt;
            float cos = Mathf.Cos(midYaw);
            float sin = Mathf.Sin(midYaw);

            m_Pose.x += (m_Twist.x * cos - m_Twist.y * sin) * dt;
            m_Pose.y += (m_Twist.x * sin + m_Twist.y * cos) * dt;
            m_Pose.z += m_Twist.z * dt;
        }

        void ApplyPoseCorrection(float dt)
        {
            if (!m_HasPoseCorrection)
            {
                return;
            }

            if (poseCorrectionRate <= 0f)
            {
                m_Pose = m_TargetPose;
                m_HasPoseCorrection = false;
                return;
            }

            float t = Mathf.Clamp01(poseCorrectionRate * dt);
            m_Pose.x = Mathf.Lerp(m_Pose.x, m_TargetPose.x, t);
            m_Pose.y = Mathf.Lerp(m_Pose.y, m_TargetPose.y, t);
            // Interpolate the shortest way round so a correction across the +/-pi wrap does
            // not spin the base the long way.
            m_Pose.z = Mathf.LerpAngle(
                m_Pose.z * Mathf.Rad2Deg, m_TargetPose.z * Mathf.Rad2Deg, t) * Mathf.Deg2Rad;

            if (Mathf.Abs(m_Pose.x - m_TargetPose.x) < 1e-4f &&
                Mathf.Abs(m_Pose.y - m_TargetPose.y) < 1e-4f &&
                Mathf.Abs(Mathf.DeltaAngle(m_Pose.z * Mathf.Rad2Deg, m_TargetPose.z * Mathf.Rad2Deg)) < 1e-2f)
            {
                m_HasPoseCorrection = false;
            }
        }
    }
}
