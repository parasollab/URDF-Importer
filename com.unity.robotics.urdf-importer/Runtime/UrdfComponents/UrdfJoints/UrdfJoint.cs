/*
© Siemens AG, 2017-2019
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using UnityEngine;

namespace Unity.Robotics.UrdfImporter
{
#if UNITY_2020_1_OR_NEWER
    [RequireComponent(typeof(ArticulationBody))]
#else
        [RequireComponent(typeof(Joint))]
#endif
    public abstract class UrdfJoint : MonoBehaviour
    {
        public enum JointTypes
        {
            Fixed,
            Continuous,
            Revolute,
            Floating,
            Prismatic,
            Planar
        }

        public int xAxis = 0;

#if UNITY_2020_1_OR_NEWER
        protected ArticulationBody unityJoint;
        protected Vector3 axisofMotion;
#else
        protected UnityEngine.Joint unityJoint;
#endif
        public string jointName;

        public abstract JointTypes JointType { get; } // Clear out syntax
        public bool IsRevoluteOrContinuous => JointType == JointTypes.Revolute || JointType == JointTypes.Revolute;
        public double EffortLimit = 1e3;
        public double VelocityLimit = 1e3;

        public bool mimic = false;
        public string mimicJointName;
        public double mimicMultiplier = 1;
        public double mimicOffset = 0;

        protected const int RoundDigits = 6;
        protected const float Tolerance = 0.0000001f;

        protected int defaultDamping = 0;
        protected int defaultFriction = 0;

        public static UrdfJoint Create(GameObject linkObject, JointTypes jointType, Joint joint = null)
        {
#if UNITY_2020_1_OR_NEWER
#else
            Rigidbody parentRigidbody = linkObject.transform.parent.gameObject.GetComponent<Rigidbody>();
            if (parentRigidbody == null) return;
#endif
            UrdfJoint urdfJoint = AddCorrectJointType(linkObject, jointType);

            if (joint != null)
            {
                urdfJoint.jointName = joint.name;
                urdfJoint.ImportMimicData(joint.mimic);
                urdfJoint.ImportJointData(joint);
            }
            return urdfJoint;
        }

        private static UrdfJoint AddCorrectJointType(GameObject linkObject, JointTypes jointType)
        {
            UrdfJoint urdfJoint = null;

            switch (jointType)
            {
                case JointTypes.Fixed:
                    urdfJoint = UrdfJointFixed.Create(linkObject);
                    break;
                case JointTypes.Continuous:
                    urdfJoint = UrdfJointContinuous.Create(linkObject);
                    break;
                case JointTypes.Revolute:
                    urdfJoint = UrdfJointRevolute.Create(linkObject);
                    break;
                case JointTypes.Floating:
                    urdfJoint = UrdfJointFloating.Create(linkObject);
                    break;
                case JointTypes.Prismatic:
                    urdfJoint = UrdfJointPrismatic.Create(linkObject);
                    break;
                case JointTypes.Planar:
                    urdfJoint = UrdfJointPlanar.Create(linkObject);
                    break;
            }


#if UNITY_2020_1_OR_NEWER
#else
            UnityEngine.Joint unityJoint = linkObject.GetComponent<UnityEngine.Joint>();
            unityJoint.connectedBody = linkObject.transform.parent.gameObject.GetComponent<Rigidbody>();
            unityJoint.autoConfigureConnectedAnchor = true;
#endif

            return urdfJoint;
        }

        /// <summary>
        /// Changes the type of the joint
        /// </summary>
        /// <param name="linkObject">Joint whose type is to be changed</param>
        /// <param name="newJointType">Type of the new joint</param>
        public static void ChangeJointType(GameObject linkObject, JointTypes newJointType)
        {
            linkObject.transform.DestroyImmediateIfExists<UrdfJoint>();
            linkObject.transform.DestroyImmediateIfExists<HingeJointLimitsManager>();
            linkObject.transform.DestroyImmediateIfExists<PrismaticJointLimitsManager>();
#if UNITY_2020_1_OR_NEWER
            linkObject.transform.DestroyImmediateIfExists<UnityEngine.ArticulationBody>();
#else
                        linkObject.transform.DestroyImmediateIfExists<UnityEngine.Joint>();
#endif
            AddCorrectJointType(linkObject, newJointType);
        }

        #region Runtime

        public void Start()
        {
#if UNITY_2020_1_OR_NEWER
            unityJoint = GetComponent<ArticulationBody>();
#else
                        unityJoint = GetComponent<Joint>();
#endif
        }

        public virtual float GetPosition()
        {
            return 0;
        }

        public virtual float GetVelocity()
        {
            return 0;
        }

        public virtual float GetEffort()
        {
            return 0;
        }

        public void UpdateJointState(float deltaState)
        {
            OnUpdateJointState(deltaState);
        }
        protected virtual void OnUpdateJointState(float deltaState) { }

        /// <summary>
        /// Drives the joint towards an absolute position, in the units the URDF and ROS use:
        /// radians for revolute and continuous joints, meters for prismatic ones. This is
        /// the counterpart to <see cref="GetPosition"/> and the surface a /joint_states
        /// subscriber should drive - <see cref="UpdateJointState"/> is relative, so feeding
        /// it absolute values accumulates error.
        /// </summary>
        public void SetPosition(float position)
        {
            if (!EnsureUnityJoint())
            {
                return;
            }
            OnSetPosition(position);
        }

        /// <summary>
        /// As <see cref="SetPosition"/>, but also teleports the body to that position rather
        /// than letting the drive travel there. Use it to seed an initial pose or to scrub a
        /// trajectory, not for continuous control - it bypasses the solver.
        /// </summary>
        public void SetPositionImmediate(float position)
        {
            if (!EnsureUnityJoint())
            {
                return;
            }

            OnSetPosition(position);

#if UNITY_2020_1_OR_NEWER
            if (unityJoint.dofCount == 1)
            {
                // jointPosition is in radians / meters, matching this method's contract.
                unityJoint.jointPosition = new ArticulationReducedSpace(position);
                unityJoint.jointVelocity = new ArticulationReducedSpace(0f);
            }
#endif
        }

        /// <summary>Applies an absolute position to the drive. Units per <see cref="SetPosition"/>.</summary>
        protected virtual void OnSetPosition(float position) { }

        /// <summary>
        /// The Unity joint is cached in Start(), which has not necessarily run when an
        /// importer or a ROS subscriber first reaches for it.
        /// </summary>
        protected bool EnsureUnityJoint()
        {
            if (unityJoint == null)
            {
#if UNITY_2020_1_OR_NEWER
                unityJoint = GetComponent<ArticulationBody>();
#else
                unityJoint = GetComponent<UnityEngine.Joint>();
#endif
            }
            return unityJoint != null;
        }

        #endregion

        #region Import Helpers

        public static JointTypes GetJointType(string jointType)
        {
            switch (jointType)
            {
                case "fixed":
                    return JointTypes.Fixed;
                case "continuous":
                    return JointTypes.Continuous;
                case "revolute":
                    return JointTypes.Revolute;
                case "floating":
                    return JointTypes.Floating;
                case "prismatic":
                    return JointTypes.Prismatic;
                case "planar":
                    return JointTypes.Planar;
                default:
                    Debug.LogWarning($"Unsupported joint type '{jointType}'; importing it as a fixed joint.");
                    return JointTypes.Fixed;
            }
        }

        protected virtual void ImportJointData(Joint joint) { }

        /// <summary>
        /// Records a &lt;mimic&gt; relationship. Handled here rather than per joint type so that
        /// prismatic mimics - the usual shape of a parallel-jaw gripper - are captured too.
        /// The coupling itself is wired up after import by
        /// <see cref="UrdfRobot.ResolveMimicJoints"/>.
        /// </summary>
        public void ImportMimicData(Joint.Mimic jointMimic)
        {
            mimic = jointMimic != null && !string.IsNullOrEmpty(jointMimic.joint);
            if (!mimic)
            {
                return;
            }

            mimicJointName = jointMimic.joint;
            mimicMultiplier = double.IsNaN(jointMimic.multiplier) ? 1 : jointMimic.multiplier;
            mimicOffset = double.IsNaN(jointMimic.offset) ? 0 : jointMimic.offset;
        }

        protected static Vector3 GetAxis(Joint.Axis axis)
        {
            return axis.xyz.ToVector3().Ros2Unity();
        }

        protected static Vector3 GetDefaultAxis()
        {
            return new Vector3(-1, 0, 0);
        }

        /// <summary>
        /// Settings for the import in progress, or freshly built defaults when a joint is
        /// created outside the import pipeline (editor menu items, tests).
        /// </summary>
        static readonly ImportSettings k_FallbackImportSettings = ImportSettings.DefaultSettings();

        protected static ImportSettings ActiveImportSettings =>
            UrdfRobotExtensions.importsettings ?? k_FallbackImportSettings;

        /// <summary>
        /// The URDF spec marks `effort` as required, but many exporters emit zero or omit
        /// it. Writing that straight into ArticulationDrive.forceLimit leaves the joint
        /// unable to move at all, so treat any non-positive or missing value as
        /// "unspecified" and substitute the import default.
        /// </summary>
        protected static float ResolveEffort(Joint.Limit limit)
        {
            return ResolveLimitValue(limit?.effort, ActiveImportSettings.defaultEffortLimit);
        }

        /// <summary>
        /// As <see cref="ResolveEffort"/>, for the `velocity` attribute. A zero here caps
        /// maxLinearVelocity / maxAngularVelocity at zero, which freezes the joint just as
        /// effectively as a zero force limit.
        /// </summary>
        protected static float ResolveVelocity(Joint.Limit limit)
        {
            return ResolveLimitValue(limit?.velocity, ActiveImportSettings.defaultVelocityLimit);
        }

        /// <summary>
        /// Stands in for a missing linear bound. PhysX rejects infinities in an
        /// ArticulationDrive, so this is a finite value far beyond any real robot's reach.
        /// </summary>
        protected const float UnboundedLinearLimit = 1e6f;

        /// <summary>
        /// `lower` and `upper` are optional, and arrive as NaN when absent. Feeding NaN to
        /// an ArticulationDrive corrupts the whole articulation, so substitute a bound.
        /// </summary>
        protected static float ResolveBound(double value, float fallback)
        {
            return double.IsNaN(value) ? fallback : (float)value;
        }

        static float ResolveLimitValue(double? value, float fallback)
        {
            if (value == null || double.IsNaN(value.Value) || value.Value <= 0)
            {
                return fallback;
            }
            return (float)value.Value;
        }

        /// <summary>
        /// Resolves a joint's axis of motion in ROS coordinates, substituting the URDF
        /// default of (1, 0, 0) for a missing or zero-length axis. Exporters routinely emit
        /// axis="0 0 0" on fixed joints, and normalizing that would yield NaN.
        ///
        /// The axis is deliberately returned as written rather than normalized: it is stored
        /// on the joint and written back out on export, and callers feed it to
        /// Quaternion.SetFromToRotation, which normalizes for itself.
        /// </summary>
        protected Vector3 ResolveAxis(Joint joint)
        {
            Vector3 axis = (joint?.axis?.xyz != null) ? joint.axis.xyz.ToVector3() : Vector3.zero;

            if (axis.sqrMagnitude < Tolerance)
            {
                if (JointType != JointTypes.Fixed)
                {
                    Debug.LogWarning(
                        $"Joint '{joint?.name ?? name}' has no usable axis; falling back to the URDF default (1, 0, 0).");
                }
                return new Vector3(1, 0, 0);
            }

            return axis;
        }

        protected virtual void AdjustMovement(Joint joint) { }

        protected void SetDynamics(Joint.Dynamics dynamics)
        {
            if (unityJoint == null)
            {
                unityJoint = GetComponent<ArticulationBody>();
            }

            if (dynamics != null)
            {
                float damping = (double.IsNaN(dynamics.damping)) ? defaultDamping : (float)dynamics.damping;
                unityJoint.linearDamping = damping;
                unityJoint.angularDamping = damping;
                unityJoint.jointFriction = (double.IsNaN(dynamics.friction)) ? defaultFriction : (float)dynamics.friction;
            }
            else
            {
                unityJoint.linearDamping = defaultDamping;
                unityJoint.angularDamping = defaultDamping;
                unityJoint.jointFriction = defaultFriction;
            }
        }

        #endregion

        #region Export

        public Joint ExportJointData()
        {
#if UNITY_2020_1_OR_NEWER
            unityJoint = GetComponent<UnityEngine.ArticulationBody>();
#else
                        unityJoint = GetComponent<UnityEngine.Joint>();
#endif
            CheckForUrdfCompatibility();

            //Data common to all joints
            Joint joint = new Joint(
                jointName,
                JointType.ToString().ToLower(),
                gameObject.transform.parent.name,
                gameObject.name,
                UrdfOrigin.ExportOriginData(transform));

            joint.limit = ExportLimitData();

            if (mimic && !string.IsNullOrEmpty(mimicJointName))
            {
                joint.mimic = new Joint.Mimic(mimicJointName, mimicMultiplier, mimicOffset);
            }

            return ExportSpecificJointData(joint);
        }

        public static Joint ExportDefaultJoint(Transform transform)
        {
            return new Joint(
                transform.parent.name + "_" + transform.name + "_joint",
                JointTypes.Fixed.ToString().ToLower(),
                transform.parent.name,
                transform.name,
                UrdfOrigin.ExportOriginData(transform));
        }

        #region ExportHelpers

        protected virtual Joint ExportSpecificJointData(Joint joint)
        {
            return joint;
        }

        protected virtual Joint.Limit ExportLimitData()
        {
            return null; // limits aren't used
        }

        public virtual bool AreLimitsCorrect()
        {
            return true; // limits aren't needed
        }

        protected virtual bool IsJointAxisDefined()
        {
#if UNITY_2020_1_OR_NEWER
            if (axisofMotion == null)
                return false;
            else
                return true;
#else
                        UnityEngine.Joint joint = GetComponent<UnityEngine.Joint>();
                        return !(Math.Abs(joint.axis.x) < Tolerance &&
                                 Math.Abs(joint.axis.y) < Tolerance &&
                                 Math.Abs(joint.axis.z) < Tolerance);
#endif
        }

        public void GenerateUniqueJointName()
        {
            jointName = transform.parent.name + "_" + transform.name + "_joint";
        }

        protected static Joint.Axis GetAxisData(Vector3 axis)
        {
            double[] rosAxis = axis.ToRoundedDoubleArray();
            return new Joint.Axis(rosAxis);
        }

        private bool IsAnchorTransformed() // TODO : Check for tolerances before implementation
        {

            UnityEngine.Joint joint = GetComponent<UnityEngine.Joint>();

            return Math.Abs(joint.anchor.x) > Tolerance ||
                Math.Abs(joint.anchor.x) > Tolerance ||
                Math.Abs(joint.anchor.x) > Tolerance;
        }

        private void CheckForUrdfCompatibility()
        {
            if (!AreLimitsCorrect())
                Debug.LogWarning("Limits are not defined correctly for Joint " + jointName + " in Link " + name +
                                 ". This may cause problems when visualizing the robot in RVIZ or Gazebo.",
                                 gameObject);
            if (!IsJointAxisDefined())
                Debug.LogWarning("Axis for joint " + jointName + " is undefined. Axis will not be written to URDF, " +
                                 "and the default axis will be used instead.",
                                 gameObject);
#if UNITY_2020_1_OR_NEWER

#else
            if (IsAnchorTransformed())
                Debug.LogWarning("The anchor position defined in the joint connected to " + name + " will be" +
                                 " ignored in URDF. Instead of modifying anchor, change the position of the link.", 
                                 gameObject);
#endif

        }

        #endregion

        #endregion
    }
}

