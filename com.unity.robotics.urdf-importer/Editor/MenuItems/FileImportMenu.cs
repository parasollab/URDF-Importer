using System.IO;
using System;
using System.Collections;
using Unity.EditorCoroutines.Editor;
using UnityEditor;
using UnityEngine;

namespace Unity.Robotics.UrdfImporter.Editor
{
    public class FileImportMenu : EditorWindow
    {
        public string urdfFile;
        public ImportSettings settings = new ImportSettings();

        private static string[] windowOptions = { };
        private bool showLoadBar = false;
        private void Awake()
        {
            this.titleContent = new GUIContent("URDF Import Settings");
        }

        private void OnGUI()
        {
            //Styles definitions

            GUIStyle titleStyle = new GUIStyle(EditorStyles.boldLabel)
            {
                alignment = TextAnchor.MiddleLeft,
                fontSize = 13
            };

            GUIStyle buttonStyle = new GUIStyle(EditorStyles.miniButtonRight) { fixedWidth = 75 };

            //Window title
            GUILayout.Space(10);
            GUILayout.Label("Select Axis Type", titleStyle);

            //Select the original up axis of the imported mesh
            GUILayout.Space(5);
            EditorGUILayout.BeginHorizontal();
            settings.chosenAxis = (ImportSettings.axisType)EditorGUILayout.EnumPopup(
                "Select Axis Type" , settings.chosenAxis);
            EditorGUILayout.EndHorizontal();

            //Window title
            GUILayout.Space(10);
            GUILayout.Label("Select Convex Decomposer", titleStyle);

            //Select the mesh decomposer
            GUILayout.Space(5);
            EditorGUILayout.BeginHorizontal();
            settings.convexMethod = (ImportSettings.convexDecomposer)EditorGUILayout.EnumPopup(
                "Mesh Decomposer", settings.convexMethod);
            EditorGUILayout.EndHorizontal();

            //Window title
            GUILayout.Space(10);
            GUILayout.Label("Select Root Motion", titleStyle);

            //How the root link relates to the world: welded for a fixed-base arm, or given
            //planar / free DOFs for a mobile base.
            GUILayout.Space(5);
            EditorGUILayout.BeginHorizontal();
            settings.rootMotion = (ImportSettings.rootMotionType)EditorGUILayout.EnumPopup(
                "Root Motion", settings.rootMotion);
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.HelpBox(
                RootMotionHelp(settings.rootMotion), MessageType.Info);

            GUILayout.Space(10);
            settings.OverwriteExistingPrefabs = GUILayout.Toggle(settings.OverwriteExistingPrefabs, "Overwrite Existing Prefabs");
            
            //Import Robot button
            GUILayout.Space(10);
            if (GUILayout.Button("Import URDF"))
            {
                if (urdfFile != "")
                {
                    showLoadBar = true;
                    EditorCoroutineUtility.StartCoroutine(UrdfRobotExtensions.Create(urdfFile, settings,showLoadBar), this);
                }
            }

            if (showLoadBar)
            {
                float progress = (settings.totalLinks == 0) ? 0 : ((float)settings.linksLoaded / (float)settings.totalLinks);
                EditorGUI.ProgressBar(new Rect(3, 400, position.width - 6, 20), progress, String.Format("{0}/{1} Links Loaded",settings.linksLoaded,settings.totalLinks));
                if (progress == 1)
                    Close();
            }
        }

        private static string RootMotionHelp(ImportSettings.rootMotionType rootMotion)
        {
            switch (rootMotion)
            {
                case ImportSettings.rootMotionType.planar:
                    return "Mobile base: the root link gains x / y / yaw degrees of freedom and " +
                           "a UrdfBaseController, which accepts /cmd_vel-style twists and /odom pose corrections.";
                case ImportSettings.rootMotionType.floating:
                    return "The root link is free in all 6 degrees of freedom and will fall under gravity " +
                           "unless something holds it up.";
                default:
                    return "Fixed base: the root link is welded to the world. Correct for arms mounted on a table or cart.";
            }
        }

        private void OnInspectorUpdate()
        {
            Repaint();
        }

    }
}