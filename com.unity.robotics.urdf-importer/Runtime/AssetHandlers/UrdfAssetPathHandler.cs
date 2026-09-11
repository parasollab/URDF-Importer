/*
© Siemens AG, 2017-2018
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

using System.IO;
using UnityEngine;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Unity.Robotics.UrdfImporter
{
    public static class UrdfAssetPathHandler
    {
        //Relative to Assets folder
        private static string packageRoot;
        private const string MaterialFolderName = "Materials";

        #region SetAssetRootFolder
        public static void SetPackageRoot(string newPath, bool correctingIncorrectPackageRoot = false)
        {
            string oldPackagePath = packageRoot;

            packageRoot = GetRelativeAssetPath(newPath);

            if (!RuntimeUrdf.AssetDatabase_IsValidFolder(Path.Combine(packageRoot, MaterialFolderName)))
            {
                RuntimeUrdf.AssetDatabase_CreateFolder(packageRoot, MaterialFolderName);
            }

            if (correctingIncorrectPackageRoot)
            {
                MoveMaterialsToNewLocation(oldPackagePath);
            }
        }
        #endregion

        #region GetPaths
        public static string GetPackageRoot()
        {
            return packageRoot;
        }
        
        public static string GetRelativeAssetPath(string absolutePath)
        {
            string assetPath = absolutePath;
            var absolutePathUnityFormat = absolutePath.SetSeparatorChar();
            if (!absolutePathUnityFormat.StartsWith(Application.dataPath.SetSeparatorChar()))
            {
#if UNITY_EDITOR
                if (!RuntimeUrdf.IsRuntimeMode())
                {
                    if (absolutePath.Length > Application.dataPath.Length)
                    {
                        assetPath = absolutePath.Substring(Application.dataPath.Length - "Assets".Length);
                    }
                }
#endif
            }
            else 
            {
                assetPath = "Assets" + absolutePath.Substring(Application.dataPath.Length);
            }
            return assetPath.SetSeparatorChar();
        }

        public static string GetFullAssetPath(string relativePath)
        {
            string fullPath = Application.dataPath;
            if (relativePath != null && relativePath.StartsWith("Assets"))
            {
                fullPath += relativePath.Substring("Assets".Length);
            }
            else 
            {
                fullPath = fullPath.Substring(0, fullPath.Length - "Assets".Length) + relativePath;
            }
            return fullPath.SetSeparatorChar();
        }

        /// <summary>
        /// Turns a mesh/texture reference taken from a URDF into a path this project can
        /// load. Handles `package://`, `model://` and `file://` URIs as well as bare
        /// absolute and relative paths.
        ///
        /// The result is normally project-relative (`Assets/...` or `Packages/...`). A file
        /// that lives outside the project is copied in first; if that isn't possible - at
        /// runtime, where there is no AssetDatabase - its absolute path is returned instead.
        /// </summary>
        public static string GetRelativeAssetPathFromUrdfPath(string urdfPath, bool convertToPrefab=true)
        {
            string path = ResolveUrdfPath(urdfPath);

            if (convertToPrefab)
            {
                if (Path.GetExtension(path)?.ToLowerInvariant() == ".stl")
                    path = path.Substring(0, path.Length - 3) + "prefab";
            }

            return path;
        }

        static string ResolveUrdfPath(string urdfPath)
        {
            if (string.IsNullOrEmpty(urdfPath))
            {
                return urdfPath;
            }

            // `../foo` would escape the package root; rewrite it to package notation. The
            // length check matters - a bare "..", or any name shorter than three
            // characters, used to throw out of Substring.
            if (!urdfPath.StartsWith(@"file://") && !urdfPath.StartsWith(@"package://") &&
                urdfPath.Length >= 3 && urdfPath.Substring(0, 3) == "../")
            {
                UnityEngine.Debug.LogWarning("Attempting to replace file path's starting instance of `../` with standard package notation `package://` to prevent manual path traversal at root of directory!");
                urdfPath = $@"package://{urdfPath.Substring(3)}";
            }

            // ROS/ROS2 package reference, or its Gazebo/SDF equivalent.
            if (urdfPath.StartsWith(@"package://"))
            {
                return ResolvePackagePath(urdfPath.Substring("package://".Length).SetSeparatorChar());
            }
            if (urdfPath.StartsWith(@"model://"))
            {
                return ResolvePackagePath(urdfPath.Substring("model://".Length).SetSeparatorChar());
            }

            // Absolute location on disk, either as a file:// URI or written plainly. This is
            // what xacro produces once its $(arg ...) mesh directories have been expanded.
            if (urdfPath.StartsWith(@"file://"))
            {
                return ResolveAbsolutePath(urdfPath.Substring("file://".Length).SetSeparatorChar());
            }

            string plainPath = urdfPath.SetSeparatorChar();
            if (Path.IsPathRooted(plainPath))
            {
                return ResolveAbsolutePath(plainPath);
            }

            return Path.Combine(packageRoot, plainPath).SetSeparatorChar();
        }

        /// <summary>
        /// Resolves `package://<pkg>/<rest>`. The package name is first treated as a plain
        /// folder under the package root, which is where an in-project copy of a robot
        /// description lives. Only when that turns up nothing do we search the ROS package
        /// paths, so an in-project asset always wins over an installed one.
        /// </summary>
        static string ResolvePackagePath(string packageRelativePath)
        {
            string underPackageRoot = Path.Combine(packageRoot, packageRelativePath).SetSeparatorChar();

            if (RuntimeUrdf.IsRuntimeMode() || File.Exists(GetFullAssetPath(underPackageRoot)))
            {
                return underPackageRoot;
            }

            foreach (string searchRoot in GetRosPackageSearchPaths())
            {
                string candidate = Path.Combine(searchRoot, packageRelativePath).SetSeparatorChar();
                if (File.Exists(candidate))
                {
                    return ResolveAbsolutePath(candidate);
                }
            }

            // Nothing matched. Hand back the package-root form so LocateAssetHandler can
            // report a path the user recognises and offer to locate the file.
            return underPackageRoot;
        }

        /// <summary>
        /// Converts an absolute filesystem path into something loadable: project-relative
        /// when it is already inside the project, a staged copy when it isn't.
        /// </summary>
        static string ResolveAbsolutePath(string absolutePath)
        {
            if (IsInsideProject(absolutePath))
            {
                return GetRelativeAssetPath(absolutePath);
            }

            string staged = UrdfAssetStaging.StageExternalAsset(absolutePath);

            // At runtime there is no AssetDatabase to stage into, and the mesh loaders read
            // straight from disk - so the absolute path is the right answer there.
            return staged ?? absolutePath;
        }

        /// <summary>
        /// True when an absolute path sits under the project's Assets or Packages folder.
        /// Tested against the absolute form because GetRelativeAssetPath produces a garbled
        /// string for anything outside the project.
        /// </summary>
        static bool IsInsideProject(string absolutePath)
        {
            if (string.IsNullOrEmpty(absolutePath))
            {
                return false;
            }

            string normalized = absolutePath.SetSeparatorChar();
            string assets = Application.dataPath.SetSeparatorChar();
            string packages = Path.Combine(
                assets.Substring(0, assets.Length - "Assets".Length), "Packages").SetSeparatorChar();

            return normalized.StartsWith(assets + "/") || normalized.StartsWith(packages + "/");
        }

        /// <summary>
        /// Directories named by the standard ROS 1 / ROS 2 environment variables, each of
        /// which holds packages as immediate subdirectories.
        /// </summary>
        static IEnumerable<string> GetRosPackageSearchPaths()
        {
            foreach (string variable in new[] { "ROS_PACKAGE_PATH", "AMENT_PREFIX_PATH" })
            {
                string value = Environment.GetEnvironmentVariable(variable);
                if (string.IsNullOrEmpty(value))
                {
                    continue;
                }

                foreach (string entry in value.Split(Path.PathSeparator))
                {
                    string trimmed = entry.Trim();
                    if (trimmed.Length == 0)
                    {
                        continue;
                    }

                    yield return trimmed.SetSeparatorChar();

                    // ament installs put package data under <prefix>/share/<pkg>.
                    string share = Path.Combine(trimmed, "share").SetSeparatorChar();
                    if (Directory.Exists(share))
                    {
                        yield return share;
                    }
                }
            }
        }
        #endregion

        public static bool IsValidAssetPath(string path)
        {
#if UNITY_EDITOR
            if (!RuntimeUrdf.IsRuntimeMode())
            {
                return Directory.Exists(path) || File.Exists(path);
            }
#endif
            //RuntimeImporter. TODO: check if the path really exists
            return true;
        }

        #region Materials

        private static void MoveMaterialsToNewLocation(string oldPackageRoot)
        {
            if (RuntimeUrdf.AssetDatabase_IsValidFolder(Path.Combine(oldPackageRoot, MaterialFolderName)))
            {
                RuntimeUrdf.AssetDatabase_MoveAsset(
                    Path.Combine(oldPackageRoot, MaterialFolderName),
                    Path.Combine(UrdfAssetPathHandler.GetPackageRoot(), MaterialFolderName));
            }
            else
            {
                RuntimeUrdf.AssetDatabase_CreateFolder(UrdfAssetPathHandler.GetPackageRoot(), MaterialFolderName);
            }
        }

        public static string GetMaterialAssetPath(string materialName)
        {
            return Path.Combine(packageRoot, MaterialFolderName, Path.GetFileName(materialName) + ".mat");
        }

        #endregion
    }

}