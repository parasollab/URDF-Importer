using NUnit.Framework;
using UnityEngine;
using Joint = Unity.Robotics.UrdfImporter.Joint;

namespace Unity.Robotics.UrdfImporter.Tests
{
    /// <summary>
    /// Covers the shapes real exporters produce that the importer used to mishandle: limits
    /// with zero or missing effort/velocity, zero-length axes, joints with no limit at all,
    /// and links with no inertial block.
    /// </summary>
    public class UrdfJointRobustnessTests
    {
        GameObject m_BaseObject;
        GameObject m_LinkObject;

        [SetUp]
        public void SetUp()
        {
            m_BaseObject = new GameObject("base");
            m_LinkObject = new GameObject("link");
            m_LinkObject.transform.parent = m_BaseObject.transform;
            UrdfJoint.Create(m_BaseObject, UrdfJoint.JointTypes.Fixed);
        }

        [TearDown]
        public void TearDown()
        {
            if (m_BaseObject != null)
            {
                Object.DestroyImmediate(m_BaseObject);
            }
        }

        ArticulationBody Body => m_LinkObject.GetComponent<ArticulationBody>();

        static Joint MakeJoint(string type, Joint.Limit limit = null, Joint.Axis axis = null)
        {
            return new Joint(
                name: $"custom_{type}_joint", type: type, parent: "base", child: "link",
                axis: axis, limit: limit);
        }

        // SolidWorks-exported descriptions - Stretch 4's among them - write effort="0"
        // velocity="0" on every limit. Passing those through leaves the drive unable to exert
        // any force and capped at zero speed, so the joint imports frozen.
        [Test]
        public void ImportJointData_ZeroEffortAndVelocity_SubstitutesDefaults()
        {
            var settings = ImportSettings.DefaultSettings();

            UrdfJoint.Create(
                m_LinkObject, UrdfJoint.JointTypes.Revolute,
                MakeJoint("revolute", new Joint.Limit(-1, 1, 0, 0), new Joint.Axis(new double[] { 0, 0, 1 })));

            Assert.AreEqual(settings.defaultEffortLimit, Body.xDrive.forceLimit);
            Assert.AreEqual(settings.defaultVelocityLimit, Body.maxAngularVelocity);
        }

        [Test]
        public void ImportJointData_ZeroEffortAndVelocity_Prismatic_SubstitutesDefaults()
        {
            var settings = ImportSettings.DefaultSettings();

            UrdfJoint.Create(
                m_LinkObject, UrdfJoint.JointTypes.Prismatic,
                MakeJoint("prismatic", new Joint.Limit(0, 0.13, 0, 0), new Joint.Axis(new double[] { 1, 0, 0 })));

            Assert.AreEqual(settings.defaultEffortLimit, Body.xDrive.forceLimit);
            Assert.AreEqual(settings.defaultVelocityLimit, Body.maxLinearVelocity);
            Assert.AreEqual(0f, Body.xDrive.lowerLimit);
            UnityEngine.Assertions.Assert.AreApproximatelyEqual(0.13f, Body.xDrive.upperLimit);
        }

        // A continuous joint has no <limit> by definition. The drive setup used to sit behind
        // a null check on it, so wheels imported with an unusable drive.
        [Test]
        public void ImportJointData_ContinuousWithoutLimit_StillConfiguresDrive()
        {
            var settings = ImportSettings.DefaultSettings();

            UrdfJoint.Create(
                m_LinkObject, UrdfJoint.JointTypes.Continuous,
                MakeJoint("continuous", axis: new Joint.Axis(new double[] { 0, 0, 1 })));

            Assert.AreEqual(settings.defaultEffortLimit, Body.xDrive.forceLimit);
            Assert.AreEqual(settings.defaultVelocityLimit, Body.maxAngularVelocity);
            Assert.AreEqual(ArticulationDofLock.FreeMotion, Body.twistLock);
        }

        // A continuous joint with no <axis> used to throw a NullReferenceException.
        [Test]
        public void ImportJointData_ContinuousWithoutAxis_DoesNotThrow()
        {
            Assert.DoesNotThrow(() => UrdfJoint.Create(
                m_LinkObject, UrdfJoint.JointTypes.Continuous, MakeJoint("continuous")));
        }

        // Every fixed joint in a SolidWorks export carries axis="0 0 0"; normalizing that
        // would produce NaN and corrupt the anchor rotation.
        [Test]
        public void ImportJointData_ZeroLengthAxis_FallsBackToUrdfDefault()
        {
            UrdfJoint.Create(
                m_LinkObject, UrdfJoint.JointTypes.Prismatic,
                MakeJoint("prismatic", new Joint.Limit(0, 1, 1, 1), new Joint.Axis(new double[] { 0, 0, 0 })));

            Quaternion anchor = Body.anchorRotation;
            Assert.IsFalse(float.IsNaN(anchor.x) || float.IsNaN(anchor.y) ||
                           float.IsNaN(anchor.z) || float.IsNaN(anchor.w));

            var expected = new Quaternion();
            expected.SetFromToRotation(Vector3.right, new Vector3(1, 0, 0).Ros2Unity());
            UnityEngine.Assertions.Assert.AreApproximatelyEqual(expected.w, anchor.w);
        }

        // lower/upper are optional and arrive as NaN. NaN in an ArticulationDrive corrupts
        // the whole articulation, so a bound has to be substituted.
        [Test]
        public void ImportJointData_MissingBounds_DoNotProduceNaN()
        {
            UrdfJoint.Create(
                m_LinkObject, UrdfJoint.JointTypes.Prismatic,
                MakeJoint("prismatic", new Joint.Limit(double.NaN, double.NaN, 5, 5),
                    new Joint.Axis(new double[] { 1, 0, 0 })));

            Assert.IsFalse(float.IsNaN(Body.xDrive.lowerLimit));
            Assert.IsFalse(float.IsNaN(Body.xDrive.upperLimit));
            Assert.Less(Body.xDrive.lowerLimit, Body.xDrive.upperLimit);
        }

        [Test]
        public void AxisofMotion_NegativeAxis_ReturnsDominantIndex()
        {
            Assert.AreEqual(2, new Joint.Axis(new double[] { 0, 0, -1 }).AxisofMotion());
            Assert.AreEqual(1, new Joint.Axis(new double[] { 0, -1, 0 }).AxisofMotion());
            Assert.AreEqual(0, new Joint.Axis(new double[] { 1, 0, 0 }).AxisofMotion());
            Assert.AreEqual(-1, new Joint.Axis(new double[] { 0, 0, 0 }).AxisofMotion());
        }

        [Test]
        public void MimicDefaults_MissingAttributes_UseSpecDefaults()
        {
            UrdfJoint urdfJoint = UrdfJoint.Create(
                m_LinkObject, UrdfJoint.JointTypes.Prismatic,
                new Joint(
                    name: "follower", type: "prismatic", parent: "base", child: "link",
                    axis: new Joint.Axis(new double[] { 1, 0, 0 }),
                    limit: new Joint.Limit(0, 1, 1, 1),
                    mimic: new Joint.Mimic("leader", double.NaN, double.NaN)));

            Assert.IsTrue(urdfJoint.mimic);
            Assert.AreEqual("leader", urdfJoint.mimicJointName);
            Assert.AreEqual(1, urdfJoint.mimicMultiplier);
            Assert.AreEqual(0, urdfJoint.mimicOffset);
        }

        // Mimic used to be captured only for revolute joints, dropping the parallel-jaw
        // gripper case entirely.
        [Test]
        public void MimicData_IsCapturedForPrismaticJoints()
        {
            UrdfJoint urdfJoint = UrdfJoint.Create(
                m_LinkObject, UrdfJoint.JointTypes.Prismatic,
                new Joint(
                    name: "finger_right", type: "prismatic", parent: "base", child: "link",
                    axis: new Joint.Axis(new double[] { 1, 0, 0 }),
                    limit: new Joint.Limit(0, 1, 1, 1),
                    mimic: new Joint.Mimic("finger_left", -1, 0.5)));

            Assert.IsTrue(urdfJoint.mimic);
            Assert.AreEqual(-1, urdfJoint.mimicMultiplier);
            Assert.AreEqual(0.5, urdfJoint.mimicOffset);
        }
    }

    /// <summary>
    /// SetPosition is the counterpart to GetPosition and the surface a /joint_states
    /// subscriber drives. Both must agree on units: radians for rotation, meters for travel.
    /// </summary>
    public class UrdfJointSetPositionTests
    {
        GameObject m_BaseObject;
        GameObject m_LinkObject;

        [SetUp]
        public void SetUp()
        {
            m_BaseObject = new GameObject("base");
            m_LinkObject = new GameObject("link");
            m_LinkObject.transform.parent = m_BaseObject.transform;
            UrdfJoint.Create(m_BaseObject, UrdfJoint.JointTypes.Fixed);
        }

        [TearDown]
        public void TearDown()
        {
            if (m_BaseObject != null)
            {
                Object.DestroyImmediate(m_BaseObject);
            }
        }

        [Test]
        public void SetPosition_Revolute_WritesDegreesToDrive()
        {
            UrdfJoint joint = UrdfJointRevolute.Create(m_LinkObject);
            joint.SetPosition(Mathf.PI / 2f);

            UnityEngine.Assertions.Assert.AreApproximatelyEqual(
                90f, m_LinkObject.GetComponent<ArticulationBody>().xDrive.target, 0.001f);
        }

        // Continuous joints skipped the radian-to-degree conversion, making every relative
        // update roughly 57 times too small.
        [Test]
        public void SetPosition_Continuous_WritesDegreesToDrive()
        {
            UrdfJoint joint = UrdfJointContinuous.Create(m_LinkObject);
            joint.SetPosition(Mathf.PI);

            UnityEngine.Assertions.Assert.AreApproximatelyEqual(
                180f, m_LinkObject.GetComponent<ArticulationBody>().xDrive.target, 0.001f);
        }

        [Test]
        public void UpdateJointState_Continuous_ConvertsRadiansToDegrees()
        {
            UrdfJoint joint = UrdfJointContinuous.Create(m_LinkObject);
            joint.UpdateJointState(Mathf.PI);

            UnityEngine.Assertions.Assert.AreApproximatelyEqual(
                180f, m_LinkObject.GetComponent<ArticulationBody>().xDrive.target, 0.001f);
        }

        [Test]
        public void SetPosition_Prismatic_WritesMetersToDrive()
        {
            UrdfJoint joint = UrdfJointPrismatic.Create(m_LinkObject);
            joint.SetPosition(0.42f);

            UnityEngine.Assertions.Assert.AreApproximatelyEqual(
                0.42f, m_LinkObject.GetComponent<ArticulationBody>().xDrive.target, 0.0001f);
        }

        [Test]
        public void SetPositionImmediate_Prismatic_RoundTripsThroughGetPosition()
        {
            UrdfJoint joint = UrdfJointPrismatic.Create(m_LinkObject);
            joint.SetPositionImmediate(0.25f);

            UnityEngine.Assertions.Assert.AreApproximatelyEqual(0.25f, joint.GetPosition(), 0.0001f);
        }

        [Test]
        public void SetPositionImmediate_Revolute_RoundTripsInRadians()
        {
            UrdfJoint joint = UrdfJointRevolute.Create(m_LinkObject);
            joint.SetPositionImmediate(1.0f);

            UnityEngine.Assertions.Assert.AreApproximatelyEqual(1.0f, joint.GetPosition(), 0.0001f);
        }
    }
}
