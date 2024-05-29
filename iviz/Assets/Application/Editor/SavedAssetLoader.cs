﻿using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using Iviz.Core;
using Iviz.Displays;
using Iviz.Msgs;
using Iviz.Resources;
using JetBrains.Annotations;
using UnityEditor;
using UnityEngine;
using LightType = Iviz.Sdf.LightType;

namespace Iviz.Editor
{
    public class SavedAssetLoader : UnityEditor.Editor
    {
        //[MenuItem("Iviz/Import Saved Assets To Unity")]
        public static async void CreateAllAssets()
        {
            Resource.ClearResources();
            ExternalResourceManager manager = Resource.External;
            IReadOnlyList<string> resourceUris = manager.GetListOfModels();

            Debug.Log("SavedAssetLoader: Transferring " + resourceUris.Count + " assets...");
            foreach (string uri in resourceUris)
            {
                await CreateAssetAsync(new Uri(uri), manager);
            }

            await CreateRobotsAsync(manager);
            AssetDatabase.Refresh();

            Debug.Log("SavedAssetLoader: Done!");

            // ugly workaround
            GameObject managerNode = GameObject.Find("External Resources");
            if (managerNode != null)
            {
                DestroyImmediate(managerNode);
            }

            Resource.ClearResources();
        }

        /*
        static void LoadWorld()
        {
        string packagePath = "/Users/akzeac/Shared/aws-robomaker-hospital-world";
        string localPath = "/worlds/hospital.world";
        
        string xmlData = File.ReadAllText(packagePath + localPath);
        Sdf.SdfFile sdf = Sdf.SdfFile.Create(xmlData);
        
        var modelPaths = Sdf.SdfFile.CreateModelPaths(packagePath);
        Sdf.SdfFile newSdf = sdf.ResolveIncludes(modelPaths);
        
        CreateWorld(newSdf.Worlds[0]);
        }
        */


        static async ValueTask CreateRobotsAsync([NotNull] ExternalResourceManager manager)
        {
            string unityDirectory = "Resources/Package/iviz/robots";
            string absolutePath = $"{Application.dataPath}/{unityDirectory}";
            Directory.CreateDirectory(absolutePath);

            Debug.LogWarning("SavedAssetLoader: Not writing robot resource files.");

            foreach (string robotName in manager.GetRobotNames())
            {
                var (_, robotDescription) = await manager.TryGetRobotAsync(robotName);
                string filename = ExternalResourceManager.SanitizeFilename(robotName).Replace(".", "_");
                await File.WriteAllTextAsync($"{absolutePath}/{filename}.txt", robotDescription);
            }
        }


        static async ValueTask CreateAssetAsync([NotNull] Uri assetUri, ExternalResourceManager manager)
        {
            const string basePath = "Assets/Resources/Package/";
            string uriPath = assetUri.Host + Uri.UnescapeDataString(assetUri.AbsolutePath);

            string relativePath = Path.GetDirectoryName(uriPath);
            if (!string.IsNullOrWhiteSpace(relativePath) && Path.DirectorySeparatorChar == '\\')
            {
                relativePath = relativePath.Replace('\\', '/');
            }

            string filename = Path.GetFileNameWithoutExtension(uriPath);
            string filenameWithExtension = Path.GetFileName(uriPath);

            string topPath = basePath + relativePath;
            string innerPath = topPath + "/" + filename;
            string unityDirectory = "Resources/Package/" + relativePath + "/" + filename;

            string absolutePath = UnityEngine.Application.dataPath + "/" + unityDirectory;

            if (Directory.Exists(absolutePath))
            {
                Debug.Log("SavedAssetLoader: Skipping " + assetUri + "...");
                return;
            }

            Directory.CreateDirectory(absolutePath);

            var resourceInfo = await manager.TryGetGameObjectAsync(assetUri.ToString(), null, default);
            if (resourceInfo == null)
            {
                throw new FileNotFoundException(assetUri.ToString());
            }

            GameObject obj = resourceInfo.Object;
            MeshTrianglesDisplay[] resources = obj.GetComponentsInChildren<MeshTrianglesDisplay>();
            HashSet<Mesh> writtenMeshes = new HashSet<Mesh>();

            int meshId = 0;
            foreach (var resource in resources)
            {
                MeshFilter filter = resource.GetComponent<MeshFilter>();
                if (writtenMeshes.Contains(filter.sharedMesh))
                {
                    continue;
                }

                if (filter.sharedMesh.GetIndexCount(0) != 0)
                {
                    Unwrapping.GenerateSecondaryUVSet(filter.sharedMesh);
                }

                writtenMeshes.Add(filter.sharedMesh);

                //string meshPath = AssetDatabase.GenerateUniqueAssetPath(innerPath + "/mesh.mesh");
                //AssetDatabase.CreateAsset(filter.sharedMesh, meshPath);


                StringBuilder sb = new StringBuilder();
                Mesh m = filter.sharedMesh;
                foreach (Vector3 v in m.vertices)
                {
                    sb.AppendFormat(BuiltIns.Culture, "v {0} {1} {2}\n", -v.x, v.y, v.z);
                }

                foreach (Vector3 v in m.normals)
                {
                    sb.AppendFormat(BuiltIns.Culture, "vn {0} {1} {2}\n", -v.x, v.y, v.z);
                }

                sb.AppendLine();
                foreach (Vector2 v in m.uv)
                {
                    sb.AppendFormat(BuiltIns.Culture, "vt {0} {1}\n", v.x, v.y);
                }

                for (int subMesh = 0; subMesh < m.subMeshCount; subMesh++)
                {
                    sb.AppendLine();
                    int[] triangles = m.GetTriangles(subMesh);
                    for (int i = 0; i < triangles.Length; i += 3)
                    {
                        sb.AppendFormat(BuiltIns.Culture, "f {0}/{0}/{0} {1}/{1}/{1} {2}/{2}/{2}\n",
                            triangles[i] + 1, triangles[i + 2] + 1, triangles[i + 1] + 1);
                    }
                }

                File.WriteAllText($"{absolutePath}/mesh-{meshId}.obj", sb.ToString());
                AssetDatabase.Refresh();

                string meshPath = $"{innerPath}/mesh-{meshId}.obj";
                Mesh newMesh = AssetDatabase.LoadAssetAtPath<Mesh>(meshPath);
                filter.sharedMesh = newMesh;

                meshId++;
            }

            Material baseMaterial = UnityEngine.Resources.Load<Material>("Materials/Standard");

            Dictionary<(Color, Color, Texture2D), Material> writtenColors =
                new Dictionary<(Color, Color, Texture2D), Material>();

            int textureId = 0;
            foreach (var resource in resources)
            {
                MeshRenderer renderer = resource.GetComponent<MeshRenderer>();
                Color color = resource.Color;
                Color emissive = resource.EmissiveColor;
                Texture2D texture = resource.DiffuseTexture;

                if (writtenColors.TryGetValue((color, emissive, texture), out Material material))
                {
                    resource.SetMaterialValuesDirect((Texture2D)material.mainTexture, emissive, color, Color.white);
                    renderer.sharedMaterial = material;
                    continue;
                }

                material = Instantiate(baseMaterial);
                material.color = color;
                material.mainTexture = texture;
                renderer.sharedMaterial = material;

                writtenColors.Add((color, emissive, texture), material);
                string materialPath = AssetDatabase.GenerateUniqueAssetPath(innerPath + "/material.mat");
                AssetDatabase.CreateAsset(material, materialPath);

                if (texture == null)
                {
                    resource.SetMaterialValuesDirect(null, emissive, color, Color.white);
                    continue;
                }

                byte[] bytes = texture.EncodeToPNG();
                File.WriteAllBytes($"{absolutePath}/texture-{textureId}.png", bytes);
                AssetDatabase.Refresh();

                string texturePath = $"{innerPath}/texture-{textureId}.png";
                Texture2D newTexture = AssetDatabase.LoadAssetAtPath<Texture2D>(texturePath);
                material.mainTexture = newTexture;

                textureId++;

                resource.SetMaterialValuesDirect(newTexture, emissive, color, Color.white);
            }


            foreach (var resource in resources)
            {
                BoxCollider collider = resource.GetComponent<BoxCollider>();
                collider.SetLocalBounds(resource.Bounds is { } bounds ? bounds : default);
                collider.enabled = false;

                //DestroyImmediate(resource);
                resource.enabled = false;
            }

            foreach (var marker in obj.GetComponentsInChildren<MeshMarkerHolderDisplay>())
            {
                //DestroyImmediate(marker);
                marker.enabled = false;
            }

            Debug.Log("Writing to " + topPath + "/" + filenameWithExtension + ".prefab");
            PrefabUtility.SaveAsPrefabAssetAndConnect(obj, topPath + "/" + filenameWithExtension + ".prefab",
                InteractionMode.UserAction);

            //DestroyImmediate(obj);
        }


        static void CreateWorld(Sdf.World world)
        {
            GameObject worldObject = new GameObject("World:" + world.Name);

            foreach (Sdf.Model model in world.Models)
            {
                CreateModel(model)?.transform.SetParent(worldObject.transform, false);
            }

            foreach (Sdf.Light source in world.Lights)
            {
                GameObject lightObject = new GameObject("Light:" + source.Name);
                lightObject.transform.parent = worldObject.transform;
                Light light = lightObject.AddComponent<Light>();
                light.color = source.Diffuse.ToColor();
                light.lightmapBakeType = LightmapBakeType.Mixed;
                light.shadows = source.CastShadows ? LightShadows.Soft : LightShadows.None;
                lightObject.transform.SetLocalPose(source.Pose.ToPose());
                light.range = 20;
                switch (source.Type)
                {
                    default:
                        light.type = UnityEngine.LightType.Point;
                        break;
                    case LightType.Spot:
                        light.type = UnityEngine.LightType.Spot;
                        light.transform.LookAt(light.transform.position + source.Direction.Ros2Unity());
                        light.spotAngle = (float)source.Spot.OuterAngle * Mathf.Rad2Deg;
                        light.innerSpotAngle = (float)source.Spot.InnerAngle * Mathf.Rad2Deg;
                        break;
                    case LightType.Directional:
                        light.type = UnityEngine.LightType.Directional;
                        light.transform.LookAt(light.transform.position + source.Direction.Ros2Unity());
                        break;
                }
            }
        }

        [NotNull]
        static GameObject CreateModel([NotNull] Sdf.Model model)
        {
            GameObject modelObject = new GameObject("Model:" + model.Name);
            Pose pose = model.Pose?.ToPose() ?? UnityEngine.Pose.identity;
            Pose includePose = model.IncludePose?.ToPose() ?? UnityEngine.Pose.identity;
            modelObject.transform.SetLocalPose(includePose.Multiply(pose));

            foreach (Sdf.Model innerModel in model.Models)
            {
                CreateModel(innerModel).transform.SetParent(modelObject.transform, false);
            }

            foreach (Sdf.Link link in model.Links)
            {
                CreateLink(link)?.transform.SetParent(modelObject.transform, false);
            }

            return modelObject;
        }

        [NotNull]
        static GameObject CreateLink([NotNull] Sdf.Link link)
        {
            GameObject linkObject = new GameObject("Link:" + link.Name);
            linkObject.transform.SetLocalPose(link.Pose?.ToPose() ?? UnityEngine.Pose.identity);

            foreach (Sdf.Visual visual in link.Visuals)
            {
                Sdf.Geometry geometry = visual.Geometry;

                GameObject visualObject = new GameObject
                (
                    name: visual.Name != null ? $"[Visual:{visual.Name}]" : "[Visual]"
                );
                visualObject.transform.SetParent(linkObject.transform, false);

                GameObject resourceObject = null;
                //bool isSynthetic = false;
                if (geometry.Mesh != null)
                {
                    Uri uri = geometry.Mesh.Uri.ToUri();

                    string path =
                        $"Assets/Resources/Package/{uri.Host}{Uri.UnescapeDataString(uri.AbsolutePath)}.prefab";

                    Debug.Log(path);

                    GameObject assetObject = AssetDatabase.LoadAssetAtPath<GameObject>(path);
                    if (assetObject == null)
                    {
                        throw new Exception();
                    }

                    resourceObject = Instantiate(assetObject, visualObject.transform, false);
                    visualObject.transform.localScale = geometry.Mesh.Scale.Ros2Unity().Abs();
                    //isSynthetic = true;
                }
                else if (geometry.Cylinder != null)
                {
                    resourceObject = Instantiate(Resource.Displays.Cylinder.Object, visualObject.transform, false);
                    visualObject.transform.localScale = new Vector3(
                        (float)geometry.Cylinder.Radius * 2,
                        (float)geometry.Cylinder.Length,
                        (float)geometry.Cylinder.Radius * 2);
                }
                else if (geometry.Box != null)
                {
                    resourceObject = Instantiate(Resource.Displays.Cube.Object, visualObject.transform, false);
                    visualObject.transform.localScale = geometry.Box.Scale.Ros2Unity().Abs();
                }
                else if (geometry.Sphere != null)
                {
                    resourceObject = Instantiate(Resource.Displays.Sphere.Object, visualObject.transform, false);
                    visualObject.transform.localScale = (float)geometry.Sphere.Radius * Vector3.one;
                }

                if (resourceObject == null)
                {
                    continue; //?
                }
            }

            return linkObject;
        }
    }
}