using UnityEngine;

namespace Unity.Robotics.UrdfImporter
{
    /// <summary>
    /// The three degrees of freedom a mobile base has on the ground plane, expressed as real
    /// articulation joints rather than as a transform the importer moves behind the physics
    /// engine's back.
    ///
    /// This mirrors how ROS models a mobile base: a virtual planar joint between `odom` and
    /// `base_link`. Because the DOFs are ordinary joints, the base participates in the same
    /// articulation as the arm, and whole-body trajectories - which name base DOFs alongside
    /// arm joints - can be replayed through the same code path as any other joint.
    ///
    /// Poses are in ROS convention throughout: x forward, y left, yaw counter-clockwise about
    /// z, in meters and radians. The Unity-space mapping is handled internally.
    /// </summary>
    public class UrdfPlanarBase : MonoBehaviour
    {
        /// <summary>Anchors the articulation to the world; carries no joint of its own.</summary>
        public ArticulationBody anchorBody;
        /// <summary>Translation along ROS +x (forward).</summary>
        public ArticulationBody forwardBody;
        /// <summary>Translation along ROS +y (left).</summary>
        public ArticulationBody lateralBody;
        /// <summary>Rotation about ROS +z (yaw).</summary>
        public ArticulationBody yawBody;

        // ROS x (forward) is Unity +z, ROS y (left) is Unity -x, ROS yaw about +z is a
        // negative rotation about Unity +y - Unity being left-handed flips the sign.
        static readonly Vector3 k_ForwardAxisUnity = Vector3.forward;
        static readonly Vector3 k_LateralAxisUnity = Vector3.left;
        static readonly Vector3 k_YawAxisUnity = Vector3.up;

        const float k_DefaultStiffness = 100000f;
        const float k_DefaultDamping = 10000f;

        /// <summary>
        /// Inserts the planar chain between <paramref name="robotRoot"/> and the robot's root
        /// link, and returns the component describing it. The root link keeps its own
        /// ArticulationBody and simply becomes a fixed child of the yaw body.
        /// </summary>
        public static UrdfPlanarBase Create(
            Transform robotRoot, Transform baseLink, float effortLimit, float velocityLimit)
        {
            if (robotRoot == null || baseLink == null)
            {
                return null;
            }

            var planarBase = robotRoot.gameObject.GetComponent<UrdfPlanarBase>();
            if (planarBase != null)
            {
                return planarBase;
            }
            planarBase = robotRoot.gameObject.AddComponent<UrdfPlanarBase>();

            // The articulation root cannot itself carry a joint, so the chain starts with a
            // jointless anchor body. `immovable` pins that anchor to the world; everything
            // below it still moves freely through the three drives.
            planarBase.anchorBody = CreateBody("odom", robotRoot);
            planarBase.anchorBody.immovable = true;

            planarBase.forwardBody = CreateLinearBody(
                "mobile_base_x", planarBase.anchorBody.transform, k_ForwardAxisUnity, effortLimit, velocityLimit);
            planarBase.lateralBody = CreateLinearBody(
                "mobile_base_y", planarBase.forwardBody.transform, k_LateralAxisUnity, effortLimit, velocityLimit);
            planarBase.yawBody = CreateAngularBody(
                "mobile_base_theta", planarBase.lateralBody.transform, k_YawAxisUnity, effortLimit, velocityLimit);

            // Re-parent the robot's root link under the chain, preserving its local pose.
            Vector3 localPosition = baseLink.localPosition;
            Quaternion localRotation = baseLink.localRotation;
            baseLink.SetParent(planarBase.yawBody.transform, false);
            baseLink.localPosition = localPosition;
            baseLink.localRotation = localRotation;

            return planarBase;
        }

        /// <summary>Current base pose as (x, y, yaw) in ROS meters and radians.</summary>
        public Vector3 GetPose()
        {
            return new Vector3(
                ReadDof(forwardBody),
                ReadDof(lateralBody),
                -ReadDof(yawBody));
        }

        /// <summary>Drives the base towards an absolute pose in the odom frame.</summary>
        public void SetPose(float x, float y, float yawRadians)
        {
            DriveDof(forwardBody, x);
            DriveDof(lateralBody, y);
            // ArticulationDrive targets are in degrees for a revolute DOF, and Unity's
            // rotation sense about +y is opposite to ROS yaw.
            DriveDof(yawBody, -yawRadians * Mathf.Rad2Deg);
        }

        /// <summary>
        /// Teleports the base to an absolute pose instead of letting the drives travel there.
        /// Use it to seed a starting pose or to jump to a trajectory sample.
        /// </summary>
        public void SetPoseImmediate(float x, float y, float yawRadians)
        {
            SetPose(x, y, yawRadians);
            TeleportDof(forwardBody, x);
            TeleportDof(lateralBody, y);
            TeleportDof(yawBody, -yawRadians);
        }

        /// <summary>True when the chain was built and all three DOFs are present.</summary>
        public bool IsValid =>
            forwardBody != null && lateralBody != null && yawBody != null;

        static float ReadDof(ArticulationBody body)
        {
            return (body != null && body.dofCount == 1) ? body.jointPosition[0] : 0f;
        }

        static void DriveDof(ArticulationBody body, float target)
        {
            if (body == null)
            {
                return;
            }
            ArticulationDrive drive = body.xDrive;
            drive.target = target;
            body.xDrive = drive;
        }

        static void TeleportDof(ArticulationBody body, float position)
        {
            if (body == null || body.dofCount != 1)
            {
                return;
            }
            body.jointPosition = new ArticulationReducedSpace(position);
            body.jointVelocity = new ArticulationReducedSpace(0f);
        }

        static ArticulationBody CreateBody(string name, Transform parent)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.transform.localPosition = Vector3.zero;
            go.transform.localRotation = Quaternion.identity;

            var body = go.AddComponent<ArticulationBody>();
            // These are virtual DOFs with no physical link behind them; keeping their mass
            // negligible stops them from perturbing the robot's real inertia.
            body.mass = 0.001f;
            body.useGravity = false;
            return body;
        }

        static ArticulationBody CreateLinearBody(
            string name, Transform parent, Vector3 axis, float effortLimit, float velocityLimit)
        {
            ArticulationBody body = CreateBody(name, parent);
            body.jointType = ArticulationJointType.PrismaticJoint;
            body.linearLockX = ArticulationDofLock.FreeMotion;
            body.linearLockY = ArticulationDofLock.LockedMotion;
            body.linearLockZ = ArticulationDofLock.LockedMotion;
            body.anchorRotation = Quaternion.FromToRotation(Vector3.right, axis);
            body.maxLinearVelocity = velocityLimit;
            body.xDrive = MakeDrive(effortLimit);
            return body;
        }

        static ArticulationBody CreateAngularBody(
            string name, Transform parent, Vector3 axis, float effortLimit, float velocityLimit)
        {
            ArticulationBody body = CreateBody(name, parent);
            body.jointType = ArticulationJointType.RevoluteJoint;
            body.linearLockX = ArticulationDofLock.LockedMotion;
            body.linearLockY = ArticulationDofLock.LockedMotion;
            body.linearLockZ = ArticulationDofLock.LockedMotion;
            body.twistLock = ArticulationDofLock.FreeMotion;
            body.anchorRotation = Quaternion.FromToRotation(Vector3.right, axis);
            body.maxAngularVelocity = velocityLimit;
            body.xDrive = MakeDrive(effortLimit);
            return body;
        }

        static ArticulationDrive MakeDrive(float effortLimit)
        {
            return new ArticulationDrive
            {
                stiffness = k_DefaultStiffness,
                damping = k_DefaultDamping,
                forceLimit = effortLimit,
            };
        }
    }
}
