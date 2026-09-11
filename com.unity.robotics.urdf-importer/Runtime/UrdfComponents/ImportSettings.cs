using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.UrdfImporter
{
    public class ImportSettings
    {
        public enum axisType
        {
            yAxis,
            zAxis,
        }

        public enum convexDecomposer
        {
            unity,
            vHACD,
        }

        public enum rootMotionType
        {
            /// <summary>Root link is welded to the world (fixed-base arms).</summary>
            fixedToWorld,
            /// <summary>Root link gains 3 planar DOFs (x, z, yaw) - a mobile base.</summary>
            planar,
            /// <summary>Root link is free in all 6 DOFs.</summary>
            floating,
        }

        public axisType chosenAxis = axisType.yAxis;
        public convexDecomposer convexMethod = convexDecomposer.vHACD;
        public rootMotionType rootMotion = rootMotionType.fixedToWorld;

        /// <summary>
        /// Substituted when a joint's limit omits `effort`, or gives it as zero or a
        /// negative value. Mapping those straight through would leave the drive's
        /// forceLimit at zero, which freezes the joint.
        /// </summary>
        public float defaultEffortLimit = 1000f;

        /// <summary>
        /// Substituted when a joint's limit omits `velocity`, or gives it as zero or a
        /// negative value. Same reasoning as <see cref="defaultEffortLimit"/>.
        /// </summary>
        public float defaultVelocityLimit = 100f;

        public bool OverwriteExistingPrefabs { get; set; } = false;

        public int linksLoaded = 0;
        public int totalLinks = 0;

        static public ImportSettings DefaultSettings()
        {
            return new ImportSettings();
        }
    }
}
