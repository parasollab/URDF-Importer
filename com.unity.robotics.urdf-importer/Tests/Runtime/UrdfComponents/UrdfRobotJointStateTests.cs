using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Joint = Unity.Robotics.UrdfImporter.Joint;

namespace Unity.Robotics.UrdfImporter.Tests
{
    /// <summary>
    /// The robot-level joint API: absolute positions by name, driver-to-URDF name aliases,
    /// and mimic resolution.
    /// </summary>
    public class UrdfRobotJointStateTests
    {
        GameObject m_RobotObject;
        UrdfRobot m_Robot;

        [SetUp]
        public void SetUp()
        {
            m_RobotObject = new GameObject("robot");
            m_Robot = m_RobotObject.AddComponent<UrdfRobot>();
        }

        [TearDown]
        public void TearDown()
        {
            if (m_RobotObject != null)
            {
                Object.DestroyImmediate(m_RobotObject);
            }
        }

        UrdfJoint AddJoint(string jointName, UrdfJoint.JointTypes type, Transform parent, Joint.Mimic mimic = null)
        {
            var linkObject = new GameObject(jointName + "_link");
            linkObject.transform.parent = parent;

            var joint = new Joint(
                name: jointName, type: type.ToString().ToLower(), parent: parent.name, child: linkObject.name,
                axis: new Joint.Axis(new double[] { 1, 0, 0 }),
                limit: new Joint.Limit(-10, 10, 1, 1),
                mimic: mimic);

            return UrdfJoint.Create(linkObject, type, joint);
        }

        [Test]
        public void SetJointPositions_DrivesJointsByUrdfName()
        {
            UrdfJoint lift = AddJoint("lift_joint", UrdfJoint.JointTypes.Prismatic, m_RobotObject.transform);
            UrdfJoint yaw = AddJoint("wrist_yaw_joint", UrdfJoint.JointTypes.Revolute, m_RobotObject.transform);

            int applied = m_Robot.SetJointPositions(new Dictionary<string, float>
            {
                { "lift_joint", 0.6f },
                { "wrist_yaw_joint", 1.0f },
            });

            Assert.AreEqual(2, applied);
            UnityEngine.Assertions.Assert.AreApproximatelyEqual(
                0.6f, lift.GetComponent<ArticulationBody>().xDrive.target, 0.0001f);
            UnityEngine.Assertions.Assert.AreApproximatelyEqual(
                1.0f * Mathf.Rad2Deg, yaw.GetComponent<ArticulationBody>().xDrive.target, 0.01f);
        }

        [Test]
        public void SetJointPositions_UnknownName_IsIgnored()
        {
            AddJoint("lift_joint", UrdfJoint.JointTypes.Prismatic, m_RobotObject.transform);

            int applied = m_Robot.SetJointPositions(new Dictionary<string, float>
            {
                { "no_such_joint", 1f },
            });

            Assert.AreEqual(0, applied);
        }

        // Stretch's driver publishes `joint_lift` for the joint its URDF calls `lift_joint`.
        [Test]
        public void SetJointPositions_DriverNameAlias_ResolvesToUrdfJoint()
        {
            UrdfJoint lift = AddJoint("lift_joint", UrdfJoint.JointTypes.Prismatic, m_RobotObject.transform);
            m_Robot.jointNameAliases.Add(new UrdfRobot.JointNameAlias
            {
                sourceName = "joint_lift",
                urdfJointName = "lift_joint",
            });
            m_Robot.InvalidateJointCache();

            int applied = m_Robot.SetJointPositions(new Dictionary<string, float> { { "joint_lift", 0.3f } });

            Assert.AreEqual(1, applied);
            UnityEngine.Assertions.Assert.AreApproximatelyEqual(
                0.3f, lift.GetComponent<ArticulationBody>().xDrive.target, 0.0001f);
        }

        [Test]
        public void ResolveMimicJoints_BindsFollowerToLeader()
        {
            UrdfJoint leader = AddJoint("finger_left_joint", UrdfJoint.JointTypes.Prismatic, m_RobotObject.transform);
            UrdfJoint follower = AddJoint(
                "finger_right_joint", UrdfJoint.JointTypes.Prismatic, m_RobotObject.transform,
                new Joint.Mimic("finger_left_joint", -1, 0.02));

            int resolved = m_Robot.ResolveMimicJoints();

            Assert.AreEqual(1, resolved);
            var mimicJoint = follower.GetComponent<UrdfMimicJoint>();
            Assert.IsNotNull(mimicJoint);
            Assert.AreEqual(leader, mimicJoint.leader);
            Assert.AreEqual(follower, mimicJoint.follower);
            UnityEngine.Assertions.Assert.AreApproximatelyEqual(-1f, mimicJoint.multiplier);
            UnityEngine.Assertions.Assert.AreApproximatelyEqual(0.02f, mimicJoint.offset);
        }

        [Test]
        public void ResolveMimicJoints_UnknownLeader_IsSkipped()
        {
            AddJoint(
                "finger_right_joint", UrdfJoint.JointTypes.Prismatic, m_RobotObject.transform,
                new Joint.Mimic("no_such_joint", 1, 0));

            LogAssert.Expect(LogType.Warning, new System.Text.RegularExpressions.Regex(".*no_such_joint.*"));
            Assert.AreEqual(0, m_Robot.ResolveMimicJoints());
        }
    }
}
