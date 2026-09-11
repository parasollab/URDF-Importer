/*
© Siemens AG, 2018
Author: Suzannah Smith (suzannah.smith@siemens.com)
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

using UnityEngine;
using System;
using System.Collections.Generic;

namespace Unity.Robotics.UrdfImporter
{
    public enum GeometryTypes { Box, Cylinder, Sphere, Mesh }

    public class UrdfRobot : MonoBehaviour
    {
        public string FilePath;
        public ImportSettings.axisType chosenAxis ;
        [SerializeField]
        private ImportSettings.axisType currentOrientation = ImportSettings.axisType.yAxis;
        public List<CollisionIgnore> collisionExceptions;

        [Tooltip("Maps the joint names a ROS driver publishes onto the joint names in this " +
                 "URDF. Leave empty when they already agree.")]
        public List<JointNameAlias> jointNameAliases = new List<JointNameAlias>();

        Dictionary<string, UrdfJoint> m_JointsByName;
        Dictionary<string, string> m_AliasToUrdfName;

        //Current Settings
        public static bool collidersConvex = true;
        public static bool useUrdfInertiaData = false;
        public static bool useGravity = true;
        public static bool addController = true;
        public static bool addFkRobot = true;
        public static bool changetoCorrectedSpace = false;

        #region Configure Robot

        public void SetCollidersConvex()
        {
            foreach (MeshCollider meshCollider in GetComponentsInChildren<MeshCollider>())
                meshCollider.convex = !collidersConvex;
            collidersConvex = !collidersConvex;
        }


        public void SetUseUrdfInertiaData()
        {
            foreach (UrdfInertial urdfInertial in GetComponentsInChildren<UrdfInertial>())
                urdfInertial.useUrdfData = !useUrdfInertiaData;
            useUrdfInertiaData = !useUrdfInertiaData;
        }

        public void SetRigidbodiesUseGravity()
        {
            foreach (ArticulationBody ar in GetComponentsInChildren<ArticulationBody>())
                ar.useGravity = !useGravity;
            useGravity = !useGravity;

        }

        public void GenerateUniqueJointNames()
        {
            foreach (UrdfJoint urdfJoint in GetComponentsInChildren<UrdfJoint>())
                urdfJoint.GenerateUniqueJointName();
        }

        // Add a rotation in the model which gives the correct correspondence between UnitySpace and RosSpace
        public void ChangeToCorrectedSpace()
        {
            this.transform.Rotate(0, 180, 0);
            changetoCorrectedSpace = !changetoCorrectedSpace;
        }

        public bool CheckOrientation()
        {
            return currentOrientation == chosenAxis;
        }

        public void SetOrientation()
        {
            currentOrientation = chosenAxis;
        }

        public void AddController()
        {
            if (!addController && this.gameObject.GetComponent< Unity.Robotics.UrdfImporter.Control.Controller>() == null)
            {
                this.gameObject.AddComponent<Unity.Robotics.UrdfImporter.Control.Controller>();
            }
            else
            {
                DestroyImmediate(this.gameObject.GetComponent<Unity.Robotics.UrdfImporter.Control.Controller>());
                DestroyImmediate(this.gameObject.GetComponent<Unity.Robotics.UrdfImporter.Control.FKRobot>());
                JointControl[] scriptList = GetComponentsInChildren<JointControl>();
                foreach (JointControl script in scriptList)
                    DestroyImmediate(script);
            }
            addController = !addController;
        }

        public void AddFkRobot()
        {
            if (!addFkRobot && this.gameObject.GetComponent<Unity.Robotics.UrdfImporter.Control.FKRobot>() == null)
            {
                this.gameObject.AddComponent<Unity.Robotics.UrdfImporter.Control.FKRobot>();
            }
            else
            {
                DestroyImmediate(this.gameObject.GetComponent<Unity.Robotics.UrdfImporter.Control.FKRobot>());
            }
            addFkRobot = !addFkRobot;
        }

        public void SetAxis(ImportSettings.axisType setAxis)
        {
            this.chosenAxis = setAxis;
        }

        void Start()
        {
            CreateCollisionExceptions();
        }

        #region Joint state

        /// <summary>
        /// A ROS driver's joint names often differ from the URDF's - Stretch publishes
        /// `joint_lift` for a joint the description calls `lift_joint`, and exposes virtual
        /// aggregate joints such as `wrist_extension` that have no URDF counterpart at all.
        /// </summary>
        [Serializable]
        public struct JointNameAlias
        {
            [Tooltip("Name as published by the driver, e.g. joint_lift")]
            public string sourceName;
            [Tooltip("Joint name in this URDF, e.g. lift_joint")]
            public string urdfJointName;
        }

        /// <summary>
        /// Every joint in this robot, keyed by its URDF joint name. Built lazily and cached;
        /// call <see cref="InvalidateJointCache"/> after adding or removing joints.
        /// </summary>
        public IReadOnlyDictionary<string, UrdfJoint> JointsByName
        {
            get
            {
                BuildJointCache();
                return m_JointsByName;
            }
        }

        /// <summary>
        /// Drives joints to absolute positions, in URDF/ROS units - radians for revolute and
        /// continuous joints, meters for prismatic ones. Names may be either URDF joint names
        /// or driver-side names listed in <see cref="jointNameAliases"/>.
        /// </summary>
        /// <returns>How many of the supplied names matched a joint.</returns>
        public int SetJointPositions(IReadOnlyDictionary<string, float> positions, bool immediate = false)
        {
            if (positions == null)
            {
                return 0;
            }

            BuildJointCache();

            int applied = 0;
            foreach (KeyValuePair<string, float> entry in positions)
            {
                if (!TryGetJoint(entry.Key, out UrdfJoint joint))
                {
                    continue;
                }

                if (immediate)
                {
                    joint.SetPositionImmediate(entry.Value);
                }
                else
                {
                    joint.SetPosition(entry.Value);
                }
                applied++;
            }
            return applied;
        }

        /// <summary>
        /// Reads back the current position of every joint, in the same units
        /// <see cref="SetJointPositions"/> accepts, keyed by URDF joint name.
        /// </summary>
        public Dictionary<string, float> GetJointPositions()
        {
            BuildJointCache();

            var positions = new Dictionary<string, float>(m_JointsByName.Count);
            foreach (KeyValuePair<string, UrdfJoint> entry in m_JointsByName)
            {
                positions[entry.Key] = entry.Value.GetPosition();
            }
            return positions;
        }

        /// <summary>Resolves a URDF or driver-side joint name to the joint it names.</summary>
        public bool TryGetJoint(string jointName, out UrdfJoint joint)
        {
            joint = null;
            if (string.IsNullOrEmpty(jointName))
            {
                return false;
            }

            BuildJointCache();

            if (m_JointsByName.TryGetValue(jointName, out joint))
            {
                return true;
            }

            return m_AliasToUrdfName.TryGetValue(jointName, out string urdfName) &&
                   m_JointsByName.TryGetValue(urdfName, out joint);
        }

        /// <summary>
        /// Binds every joint that declares a &lt;mimic&gt; to the joint it follows, attaching a
        /// <see cref="UrdfMimicJoint"/> to drive the coupling. Run once after import; the
        /// URDF's mimic tags name a joint that may not exist yet while links are still being
        /// created.
        /// </summary>
        /// <returns>How many couplings were established.</returns>
        public int ResolveMimicJoints()
        {
            BuildJointCache();

            int resolved = 0;
            foreach (UrdfJoint joint in GetComponentsInChildren<UrdfJoint>(true))
            {
                if (!joint.mimic || string.IsNullOrEmpty(joint.mimicJointName))
                {
                    continue;
                }

                if (!TryGetJoint(joint.mimicJointName, out UrdfJoint leader))
                {
                    Debug.LogWarning(
                        $"Joint '{joint.jointName}' mimics '{joint.mimicJointName}', which this robot has no joint for.",
                        joint);
                    continue;
                }

                if (leader == joint)
                {
                    Debug.LogWarning($"Joint '{joint.jointName}' mimics itself; ignoring.", joint);
                    continue;
                }

                UrdfMimicJoint.Attach(joint, leader, (float)joint.mimicMultiplier, (float)joint.mimicOffset);
                resolved++;
            }
            return resolved;
        }

        /// <summary>Drops the cached joint lookup so it is rebuilt on next use.</summary>
        public void InvalidateJointCache()
        {
            m_JointsByName = null;
            m_AliasToUrdfName = null;
        }

        void BuildJointCache()
        {
            if (m_JointsByName != null)
            {
                return;
            }

            m_JointsByName = new Dictionary<string, UrdfJoint>();
            foreach (UrdfJoint joint in GetComponentsInChildren<UrdfJoint>(true))
            {
                string key = string.IsNullOrEmpty(joint.jointName) ? joint.name : joint.jointName;
                if (string.IsNullOrEmpty(key))
                {
                    continue;
                }

                if (m_JointsByName.ContainsKey(key))
                {
                    Debug.LogWarning(
                        $"Duplicate joint name '{key}' on robot '{name}'; only the first is addressable by name.",
                        this);
                    continue;
                }
                m_JointsByName[key] = joint;
            }

            m_AliasToUrdfName = new Dictionary<string, string>();
            if (jointNameAliases != null)
            {
                foreach (JointNameAlias alias in jointNameAliases)
                {
                    if (!string.IsNullOrEmpty(alias.sourceName) && !string.IsNullOrEmpty(alias.urdfJointName))
                    {
                        m_AliasToUrdfName[alias.sourceName] = alias.urdfJointName;
                    }
                }
            }
        }

        #endregion

        public void CreateCollisionExceptions()
        {
            if (collisionExceptions != null)
            {
                foreach (CollisionIgnore ignoreCollision in collisionExceptions)
                {
                    Collider[] collidersObject1 = ignoreCollision.Link1.GetComponentsInChildren<Collider>();
                    Collider[] collidersObject2 = ignoreCollision.Link2.GetComponentsInChildren<Collider>();
                    foreach (Collider colliderMesh1 in collidersObject1)
                    {
                        foreach (Collider colliderMesh2 in collidersObject2)
                        {
                            Physics.IgnoreCollision(colliderMesh1, colliderMesh2);
                        }
                    }
                }
            }
        }
        #endregion
    }
}
