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

namespace Unity.Robotics.UrdfImporter
{
    public static class UrdfLinkExtensions
    { 
        public static GameObject Create(Transform parent, Link link = null, Joint joint = null)
        {
            GameObject linkObject = new GameObject("link");
            linkObject.transform.SetParentAndAlign(parent);
            UrdfLink urdfLink = linkObject.AddComponent<UrdfLink>();
            UrdfCollisionsExtensions.Create(linkObject.transform, link?.collisions);
            UrdfVisualsExtensions.Create(linkObject.transform, link?.visuals);

            if (link != null)
            {
                urdfLink.ImportLinkData(link, joint);
            }
            else
            {
                UrdfInertial.Create(linkObject);
#if UNITY_EDITOR
                UnityEditor.EditorGUIUtility.PingObject(linkObject);
#endif
            }

            return linkObject;
        }

        private static void ImportLinkData(this UrdfLink urdfLink, Link link, Joint joint)
        {
            // The root is the link reached with no joint above it. Tagging it only when it
            // *also* lacked <inertial> meant a root that declares mass was never marked as
            // the base link, which in turn made export emit a spurious fixed joint for it.
            urdfLink.IsBaseLink = joint == null;

            urdfLink.gameObject.name = link.name;
            if (joint?.origin != null)
                UrdfOrigin.ImportOriginData(urdfLink.transform, joint.origin);

            // Every link gets an ArticulationBody, a root with no <inertial> included.
            // Previously that case created no body at all, so each first-level child became
            // its own articulation root and the robot imported as several loose pieces.
            UrdfInertial.Create(urdfLink.gameObject, link.inertial);

            if (joint != null)
                UrdfJoint.Create(urdfLink.gameObject, UrdfJoint.GetJointType(joint.type), joint);
        } 
        
        public static Link ExportLinkData(this UrdfLink urdfLink)
        {
            if (urdfLink.transform.localScale != Vector3.one)
                Debug.LogWarning("Only visuals should be scaled. Scale on link \"" + urdfLink.gameObject.name + "\" cannot be saved to the URDF file.", urdfLink.gameObject);
            UrdfInertial urdfInertial = urdfLink.gameObject.GetComponent<UrdfInertial>();
            Link link = new Link(urdfLink.gameObject.name)
            {
                visuals = urdfLink.GetComponentInChildren<UrdfVisuals>().ExportVisualsData(),
                collisions = urdfLink.GetComponentInChildren<UrdfCollisions>().ExportCollisionsData(),
                inertial = urdfInertial == null ? null : urdfInertial.ExportInertialData()
            };
            
            return link;
        }
    }
}