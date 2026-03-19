using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using SIGVerse.Common;
using UnityEngine;
using UnityEngine.Animations;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace SIGVerse.Competition.InteractiveCleanup
{
	public class CleanupFreeCaptureMode : MonoBehaviour
	{
		private const float DefaultMoveSpeed = 2.5f;
		private const float FastMoveMultiplier = 3.0f;
		private const float SlowMoveMultiplier = 0.35f;
		private const float LookSensitivity = 0.12f;

		private const string CaptureDirectoryRelativePath = "/../SIGVerseConfig/InteractiveCleanup/Captures/";
		private const string ConfigDirectoryRelativePath = "/../SIGVerseConfig/InteractiveCleanup/";
		private const string SampleDirectoryName = "sample";
		private const string EnvironmentInfoSearchPattern = "EnvironmentInfo*.json";
		private const string GraspingCandidatesRootName = "GraspingCandidates";

		private const string TagRobot = "Robot";
		private const string TagMainMenu = "MainMenu";
		private const string TagGraspingCandidates = "GraspingCandidates";
		private const string TagGraspingCandidatesPosition = "GraspingCandidatesPosition";
		private const string TagDestinationCandidates = "DestinationCandidates";

		private readonly List<GameObject> environments = new List<GameObject>();
		private readonly List<EnvironmentInfo> environmentInfos = new List<EnvironmentInfo>();
		private readonly Dictionary<string, GameObject> environmentLookup = new Dictionary<string, GameObject>(StringComparer.OrdinalIgnoreCase);

		private Transform captureCameraTransform;
		private Camera captureCamera;
		private Vector3 initialCameraPosition;
		private Quaternion initialCameraRotation;
		private GameObject mainMenu;
		private PanelMainController mainPanelController;
		private GameObject robot;
		private GameObject avatar;

		private float yaw;
		private float pitch;
		private int activeMapIndex;
		private bool isLookActive;


		public void Initialize(IEnumerable<GameObject> environmentObjects, PanelMainController panelController = null)
		{
			this.environments.Clear();
			if (environmentObjects != null)
			{
				this.environments.AddRange(environmentObjects.Where(environment => environment != null).Distinct());
			}

			this.environmentLookup.Clear();
			this.BuildEnvironmentLookup();

			this.mainPanelController = panelController;
			this.mainMenu = this.FindFirstObjectWithTag(TagMainMenu);
			this.robot = this.FindFirstObjectWithTag(TagRobot);

			Transform avatarTransform = this.transform.Find("Ethan");
			this.avatar = avatarTransform != null ? avatarTransform.gameObject : null;

			this.LoadEnvironmentInfos();

			this.captureCameraTransform = this.FindCaptureCameraTransform();
			if (this.captureCameraTransform == null)
			{
				SIGVerseLogger.Error("FreeCapture mode failed: primary camera was not found.");
				this.enabled = false;
				return;
			}

			this.captureCamera = this.captureCameraTransform.GetComponent<Camera>();

			this.PrepareCaptureCamera();
			this.DisableSubviewPresentation();
			this.DisableNonCaptureCameras();
			this.CacheInitialCameraPose();

			if (this.environmentInfos.Count > 0)
			{
				this.activeMapIndex = this.ResolveInitialMapIndex();
				this.SetActiveMap(this.activeMapIndex, true);
			}
			else if (this.environments.Count > 0)
			{
				this.activeMapIndex = this.ResolveInitialEnvironmentIndex();
				this.SetActiveEnvironmentOnly(this.activeMapIndex, true);
			}
			else
			{
				SIGVerseLogger.Warn("FreeCapture mode started without configured environments.");
			}

			this.HideOptionalSceneObjects();
			this.UpdatePanelMessage();

			Time.timeScale = 1.0f;
			this.SetLookActive(false);

			SIGVerseLogger.Info(
				"FreeCapture controls: RMB look, WASD move, Q/E vertical, Shift fast, Ctrl slow, [/] or PgUp/PgDn switch map, P screenshot, R reset.");
		}

		private GameObject FindFirstObjectWithTag(string tag)
		{
			GameObject[] taggedObjects = GameObject.FindGameObjectsWithTag(tag);
			return taggedObjects.Length > 0 ? taggedObjects[0] : null;
		}

		private void BuildEnvironmentLookup()
		{
			foreach (GameObject environment in this.environments)
			{
				foreach (string candidate in GetEnvironmentNameCandidates(environment.name))
				{
					if (!this.environmentLookup.ContainsKey(candidate))
					{
						this.environmentLookup.Add(candidate, environment);
					}
				}
			}
		}

		private static IEnumerable<string> GetEnvironmentNameCandidates(string environmentName)
		{
			if (string.IsNullOrEmpty(environmentName)) { yield break; }

			yield return environmentName;

			string suffixDigits = ExtractTrailingDigits(environmentName);
			if (suffixDigits == string.Empty) { yield break; }

			int normalizedIndex = int.Parse(suffixDigits);
			string twoDigits = normalizedIndex.ToString("D2");

			yield return "Environment_" + twoDigits;
			yield return "Layout2019IC" + twoDigits;
		}

		private static string ExtractTrailingDigits(string text)
		{
			Match match = Regex.Match(text ?? string.Empty, "(\\d+)$");
			return match.Success ? match.Groups[1].Value : string.Empty;
		}

		private void LoadEnvironmentInfos()
		{
			this.environmentInfos.Clear();

			string configDirectory = Path.GetFullPath(Application.dataPath + ConfigDirectoryRelativePath);
			string[] infoPaths = this.FindEnvironmentInfoPaths(configDirectory);

			foreach (string infoPath in infoPaths)
			{
				try
				{
					using (StreamReader streamReader = new StreamReader(infoPath, Encoding.UTF8))
					{
						EnvironmentInfo environmentInfo = JsonUtility.FromJson<EnvironmentInfo>(streamReader.ReadToEnd());
						if (environmentInfo != null)
						{
							this.environmentInfos.Add(environmentInfo);
						}
					}
				}
				catch (Exception exception)
				{
					SIGVerseLogger.Warn("FreeCapture skipped EnvironmentInfo. path=" + infoPath + ", reason=" + exception.Message);
				}
			}

			if (this.environmentInfos.Count > 0)
			{
				SIGVerseLogger.Info("FreeCapture loaded EnvironmentInfo files. count=" + this.environmentInfos.Count + ", directory=" + Path.GetDirectoryName(infoPaths[0]));
			}
			else
			{
				SIGVerseLogger.Warn("FreeCapture could not find any EnvironmentInfo files under " + configDirectory + " or its sample directory.");
			}
		}

		private string[] FindEnvironmentInfoPaths(string configDirectory)
		{
			string[] rootInfoPaths = this.GetSortedEnvironmentInfoPaths(configDirectory);
			if (rootInfoPaths.Length > 0)
			{
				return rootInfoPaths;
			}

			string sampleDirectory = Path.Combine(configDirectory, SampleDirectoryName);
			return this.GetSortedEnvironmentInfoPaths(sampleDirectory);
		}

		private string[] GetSortedEnvironmentInfoPaths(string directoryPath)
		{
			if (!Directory.Exists(directoryPath)) { return Array.Empty<string>(); }

			return Directory.GetFiles(directoryPath, EnvironmentInfoSearchPattern, SearchOption.TopDirectoryOnly)
				.OrderBy(path => this.ExtractEnvironmentInfoOrder(path))
				.ThenBy(path => path, StringComparer.OrdinalIgnoreCase)
				.ToArray();
		}

		private int ExtractEnvironmentInfoOrder(string filePath)
		{
			string fileName = Path.GetFileNameWithoutExtension(filePath);
			string suffixDigits = ExtractTrailingDigits(fileName);
			return suffixDigits == string.Empty ? int.MaxValue : int.Parse(suffixDigits);
		}

		private Transform FindCaptureCameraTransform()
		{
			GameObject torsoTrackingCamera = GameObject.Find("TorsoTrackingCamera");
			if (torsoTrackingCamera != null)
			{
				Camera torsoCamera = torsoTrackingCamera.GetComponent<Camera>();
				if (torsoCamera != null) { return torsoCamera.transform; }
			}

			Camera audienceCamera = this.FindSceneCameras()
				.Where(camera => camera.CompareTag("AudienceCamera") && camera.gameObject.name != "HandTrackingCamera")
				.OrderByDescending(camera => camera.depth)
				.FirstOrDefault();
			if (audienceCamera != null) { return audienceCamera.transform; }

			Camera mainCamera = Camera.main;
			if (mainCamera != null && mainCamera.gameObject.name != "AvatarCamera" && mainCamera.gameObject.name != "BirdsEyeViewCamera")
			{
				return mainCamera.transform;
			}

			Camera fallbackCamera = this.FindSceneCameras()
				.Where(camera => camera.gameObject.name != "AvatarCamera" && camera.gameObject.name != "BirdsEyeViewCamera")
				.OrderByDescending(camera => camera.depth)
				.FirstOrDefault();
			if (fallbackCamera != null) { return fallbackCamera.transform; }

			Transform avatarCamera = this.transform.Find("AvatarCamera");
			if (avatarCamera != null) { return avatarCamera; }

			return null;
		}

		private IEnumerable<Camera> FindSceneCameras()
		{
			return Resources.FindObjectsOfTypeAll<Camera>()
				.Where(camera => camera != null)
				.Where(camera => camera.gameObject.scene.IsValid());
		}

		private void PrepareCaptureCamera()
		{
			CleanupLookAtAvatar lookAtAvatar = this.captureCameraTransform.GetComponent<CleanupLookAtAvatar>();
			if (lookAtAvatar != null)
			{
				lookAtAvatar.enabled = false;
			}

			ParentConstraint parentConstraint = this.captureCameraTransform.GetComponent<ParentConstraint>();
			if (parentConstraint != null)
			{
				parentConstraint.constraintActive = false;
				parentConstraint.enabled = false;
			}

			if (this.captureCamera != null)
			{
				this.captureCamera.enabled = true;
				this.captureCamera.depth = 100.0f;
			}

			foreach (AudioListener audioListener in Resources.FindObjectsOfTypeAll<AudioListener>().Where(listener => listener.gameObject.scene.IsValid()))
			{
				audioListener.enabled = audioListener.transform == this.captureCameraTransform;
			}
		}

		private void DisableSubviewPresentation()
		{
			GameObject subviewController = GameObject.Find("SubviewController");
			if (subviewController == null) { return; }

			foreach (MonoBehaviour behaviour in subviewController.GetComponents<MonoBehaviour>())
			{
				behaviour.enabled = false;
			}
		}

		private void DisableNonCaptureCameras()
		{
			foreach (Camera camera in this.FindSceneCameras())
			{
				camera.enabled = camera.transform == this.captureCameraTransform;
			}
		}

		private void CacheInitialCameraPose()
		{
			this.initialCameraPosition = this.captureCameraTransform.position;
			this.initialCameraRotation = this.captureCameraTransform.rotation;

			Vector3 eulerAngles = this.captureCameraTransform.rotation.eulerAngles;
			this.yaw = eulerAngles.y;
			this.pitch = NormalizeAngle(eulerAngles.x);
		}

		private int ResolveInitialMapIndex()
		{
			if (this.environmentInfos.Count == 0) { return 0; }

			GameObject activeEnvironment = this.environments.FirstOrDefault(environment => environment.activeSelf);
			if (activeEnvironment == null) { return 0; }

			int index = this.environmentInfos.FindIndex(environmentInfo => this.ResolveEnvironmentObject(environmentInfo.environmentName) == activeEnvironment);
			return index >= 0 ? index : 0;
		}

		private int ResolveInitialEnvironmentIndex()
		{
			if (this.environments.Count == 0) { return -1; }

			int index = this.environments.FindIndex(environment => environment.activeSelf);
			return index >= 0 ? index : 0;
		}

		private static float NormalizeAngle(float angle)
		{
			return angle > 180.0f ? angle - 360.0f : angle;
		}

		private void HideOptionalSceneObjects()
		{
			if (this.robot != null)
			{
				this.robot.SetActive(false);
			}

			if (this.avatar != null)
			{
				this.avatar.SetActive(false);
			}

			this.HideScorePanel();
		}

		private void HideScorePanel()
		{
			if (this.mainMenu == null) { return; }

			HashSet<GameObject> panelsToHide = new HashSet<GameObject>();

			foreach (Text text in this.mainMenu.GetComponentsInChildren<Text>(true))
			{
				if (IsScorePanelText(text.text))
				{
					panelsToHide.Add(this.ResolveDirectChildOfMainMenu(text.transform));
				}
			}

			Type tmpTextType = Type.GetType("TMPro.TMP_Text, Unity.TextMeshPro");
			if (tmpTextType != null)
			{
				Component[] tmpTexts = this.mainMenu.GetComponentsInChildren(tmpTextType, true);
				foreach (Component tmpText in tmpTexts)
				{
					object textValue = tmpTextType.GetProperty("text") != null ? tmpTextType.GetProperty("text").GetValue(tmpText, null) : null;
					if (textValue is string text && IsScorePanelText(text))
					{
						panelsToHide.Add(this.ResolveDirectChildOfMainMenu(tmpText.transform));
					}
				}
			}

			foreach (GameObject panel in panelsToHide)
			{
				if (panel != null)
				{
					panel.SetActive(false);
				}
			}
		}

		private static bool IsScorePanelText(string text)
		{
			if (string.IsNullOrEmpty(text)) { return false; }

			string normalizedText = text.ToUpperInvariant();
			return normalizedText.Contains("SCORE") || normalizedText.Contains("TOTAL");
		}

		private GameObject ResolveDirectChildOfMainMenu(Transform transform)
		{
			Transform current = transform;
			while (current.parent != null && current.parent != this.mainMenu.transform)
			{
				current = current.parent;
			}

			return current.gameObject;
		}

		private void Update()
		{
			Keyboard keyboard = Keyboard.current;
			Mouse mouse = Mouse.current;
			if (keyboard == null) { return; }

			this.HandleLook(mouse);
			this.HandleMovement(keyboard);
			this.HandleCommands(keyboard);
		}

		private void HandleLook(Mouse mouse)
		{
			if (mouse == null || this.captureCameraTransform == null) { return; }

			if (mouse.rightButton.wasPressedThisFrame)
			{
				this.SetLookActive(true);
			}

			if (mouse.rightButton.wasReleasedThisFrame)
			{
				this.SetLookActive(false);
			}

			if (!this.isLookActive) { return; }

			Vector2 delta = mouse.delta.ReadValue();
			this.yaw += delta.x * LookSensitivity;
			this.pitch = Mathf.Clamp(this.pitch - delta.y * LookSensitivity, -89.0f, 89.0f);
			this.captureCameraTransform.rotation = Quaternion.Euler(this.pitch, this.yaw, 0.0f);
		}

		private void SetLookActive(bool isActive)
		{
			this.isLookActive = isActive;
			Cursor.lockState = isActive ? CursorLockMode.Locked : CursorLockMode.None;
			Cursor.visible = !isActive;
		}

		private void HandleMovement(Keyboard keyboard)
		{
			if (this.captureCameraTransform == null) { return; }

			Vector3 moveDirection = Vector3.zero;

			if (keyboard.wKey.isPressed) { moveDirection += Vector3.forward; }
			if (keyboard.sKey.isPressed) { moveDirection += Vector3.back; }
			if (keyboard.dKey.isPressed) { moveDirection += Vector3.right; }
			if (keyboard.aKey.isPressed) { moveDirection += Vector3.left; }
			if (keyboard.eKey.isPressed) { moveDirection += Vector3.up; }
			if (keyboard.qKey.isPressed) { moveDirection += Vector3.down; }

			if (moveDirection.sqrMagnitude <= 0.0f) { return; }

			moveDirection.Normalize();

			float moveSpeed = DefaultMoveSpeed;
			if (keyboard.leftShiftKey.isPressed || keyboard.rightShiftKey.isPressed)
			{
				moveSpeed *= FastMoveMultiplier;
			}
			if (keyboard.leftCtrlKey.isPressed || keyboard.rightCtrlKey.isPressed)
			{
				moveSpeed *= SlowMoveMultiplier;
			}

			this.captureCameraTransform.position += moveDirection * moveSpeed * Time.unscaledDeltaTime;
		}

		private void HandleCommands(Keyboard keyboard)
		{
			if (keyboard.escapeKey.wasPressedThisFrame)
			{
				this.SetLookActive(false);
			}

			if (keyboard.rKey.wasPressedThisFrame)
			{
				this.ResetCameraPose();
			}

			if (keyboard.pKey.wasPressedThisFrame)
			{
				this.CaptureScreenshot();
			}

			if (keyboard.leftBracketKey.wasPressedThisFrame || keyboard.pageUpKey.wasPressedThisFrame)
			{
				this.StepMap(-1);
			}

			if (keyboard.rightBracketKey.wasPressedThisFrame || keyboard.pageDownKey.wasPressedThisFrame)
			{
				this.StepMap(+1);
			}
		}

		private void StepMap(int delta)
		{
			if (this.environmentInfos.Count > 0)
			{
				this.activeMapIndex = (this.activeMapIndex + delta + this.environmentInfos.Count) % this.environmentInfos.Count;
				this.SetActiveMap(this.activeMapIndex, true);
				return;
			}

			if (this.environments.Count == 0) { return; }

			this.activeMapIndex = (this.activeMapIndex + delta + this.environments.Count) % this.environments.Count;
			this.SetActiveEnvironmentOnly(this.activeMapIndex, true);
		}

		private void SetActiveMap(int index, bool resetCamera)
		{
			if (index < 0 || index >= this.environmentInfos.Count) { return; }

			EnvironmentInfo environmentInfo = this.environmentInfos[index];
			GameObject activeEnvironment = this.ResolveEnvironmentObject(environmentInfo.environmentName);
			this.SetActiveEnvironmentObject(activeEnvironment);
			this.ApplyEnvironmentInfo(environmentInfo);

			this.activeMapIndex = index;
			if (resetCamera)
			{
				this.ResetCameraPose();
			}

			this.UpdatePanelMessage();
			SIGVerseLogger.Info("FreeCapture map switched. index=" + (index + 1) + "/" + this.environmentInfos.Count + ", env=" + (activeEnvironment != null ? activeEnvironment.name : environmentInfo.environmentName));
		}

		private void SetActiveEnvironmentOnly(int index, bool resetCamera)
		{
			if (index < 0 || index >= this.environments.Count) { return; }

			this.SetActiveEnvironmentObject(this.environments[index]);
			this.activeMapIndex = index;

			if (resetCamera)
			{
				this.ResetCameraPose();
			}

			this.UpdatePanelMessage();
			SIGVerseLogger.Info("FreeCapture environment switched. name=" + this.environments[index].name);
		}

		private GameObject ResolveEnvironmentObject(string environmentName)
		{
			foreach (string candidate in GetEnvironmentNameCandidates(environmentName))
			{
				if (this.environmentLookup.TryGetValue(candidate, out GameObject environment))
				{
					return environment;
				}
			}

			GameObject activeEnvironment = this.environments.FirstOrDefault(environment => environment.activeSelf);
			return activeEnvironment ?? this.environments.FirstOrDefault();
		}

		private void SetActiveEnvironmentObject(GameObject activeEnvironment)
		{
			foreach (GameObject environment in this.environments)
			{
				environment.SetActive(environment == activeEnvironment);
			}

			Physics.SyncTransforms();
		}

		private void ApplyEnvironmentInfo(EnvironmentInfo environmentInfo)
		{
			this.DeactivateGraspingCandidatesPositions();
			this.ActivateAllGraspingCandidates();

			Dictionary<string, GameObject> graspables = GameObject.FindGameObjectsWithTag(TagGraspingCandidates)
				.GroupBy(graspable => graspable.name)
				.ToDictionary(group => group.Key, group => group.First(), StringComparer.OrdinalIgnoreCase);

			HashSet<string> activeGraspableNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
			if (environmentInfo.graspablesPositions != null)
			{
				foreach (RelocatableObjectInfo graspableInfo in environmentInfo.graspablesPositions)
				{
					if (!graspables.TryGetValue(graspableInfo.name, out GameObject graspable))
					{
						SIGVerseLogger.Warn("FreeCapture graspable was not found. name=" + graspableInfo.name);
						continue;
					}

					graspable.SetActive(true);
					this.ApplyPose(graspable, graspableInfo);
					activeGraspableNames.Add(graspable.name);
				}
			}

			foreach (KeyValuePair<string, GameObject> pair in graspables)
			{
				if (!activeGraspableNames.Contains(pair.Key))
				{
					pair.Value.SetActive(false);
				}
			}

			Dictionary<string, GameObject> destinations = GameObject.FindGameObjectsWithTag(TagDestinationCandidates)
				.GroupBy(destination => destination.name)
				.ToDictionary(group => group.Key, group => group.First(), StringComparer.OrdinalIgnoreCase);

			if (environmentInfo.destinationsPositions != null)
			{
				foreach (RelocatableObjectInfo destinationInfo in environmentInfo.destinationsPositions)
				{
					if (!destinations.TryGetValue(destinationInfo.name, out GameObject destination))
					{
						SIGVerseLogger.Warn("FreeCapture destination was not found. name=" + destinationInfo.name);
						continue;
					}

					destination.SetActive(true);
					this.ApplyPose(destination, destinationInfo);
				}
			}

			Physics.SyncTransforms();
		}

		private void ActivateAllGraspingCandidates()
		{
			GameObject graspingCandidatesRoot = GameObject.Find(GraspingCandidatesRootName);
			if (graspingCandidatesRoot == null) { return; }

			foreach (Transform child in graspingCandidatesRoot.transform)
			{
				child.gameObject.SetActive(true);
			}
		}

		private void DeactivateGraspingCandidatesPositions()
		{
			foreach (GameObject graspingCandidatePosition in GameObject.FindGameObjectsWithTag(TagGraspingCandidatesPosition))
			{
				graspingCandidatePosition.SetActive(false);
			}
		}

		private void ApplyPose(GameObject targetObject, RelocatableObjectInfo objectInfo)
		{
			Rigidbody rigidbody = targetObject.GetComponent<Rigidbody>();
			if (rigidbody != null)
			{
				rigidbody.velocity = Vector3.zero;
				rigidbody.angularVelocity = Vector3.zero;
				rigidbody.position = objectInfo.position;
				rigidbody.rotation = Quaternion.Euler(objectInfo.eulerAngles);
			}
			else
			{
				targetObject.transform.position = objectInfo.position;
				targetObject.transform.eulerAngles = objectInfo.eulerAngles;
			}
		}

		private void UpdatePanelMessage()
		{
			if (this.mainPanelController == null) { return; }

			if (this.environmentInfos.Count > 0)
			{
				EnvironmentInfo currentInfo = this.environmentInfos[this.activeMapIndex];
				List<string> lines = new List<string>();
				lines.Add(string.Format("FreeCapture [{0}/{1}]", this.activeMapIndex + 1, this.environmentInfos.Count));
				lines.Add(this.BuildTaskMessage(currentInfo));
				lines.Add("RMB look | WASD move | Q/E up/down | [ ] switch map | P screenshot | R reset");
				this.mainPanelController.SetTaskMessageText(string.Join("\n", lines.ToArray()));
				return;
			}

			this.mainPanelController.SetTaskMessageText(
				"FreeCapture\nRMB look | WASD move | Q/E up/down | [ ] switch map | P screenshot | R reset");
		}

		private string BuildTaskMessage(EnvironmentInfo environmentInfo)
		{
			if (!string.IsNullOrEmpty(environmentInfo.taskMessage))
			{
				return environmentInfo.taskMessage;
			}

			if (!string.IsNullOrEmpty(environmentInfo.graspingTargetName) || !string.IsNullOrEmpty(environmentInfo.destinationName))
			{
				return "Target=" + environmentInfo.graspingTargetName + ", Destination=" + environmentInfo.destinationName;
			}

			return environmentInfo.environmentName;
		}

		private void ResetCameraPose()
		{
			if (this.captureCameraTransform == null) { return; }

			this.captureCameraTransform.position = this.initialCameraPosition;
			this.captureCameraTransform.rotation = this.initialCameraRotation;

			Vector3 eulerAngles = this.initialCameraRotation.eulerAngles;
			this.yaw = eulerAngles.y;
			this.pitch = NormalizeAngle(eulerAngles.x);
		}

		private void CaptureScreenshot()
		{
			string captureDirectory = Path.GetFullPath(Application.dataPath + CaptureDirectoryRelativePath);
			Directory.CreateDirectory(captureDirectory);

			string captureName;
			if (this.environmentInfos.Count > 0)
			{
				EnvironmentInfo environmentInfo = this.environmentInfos[this.activeMapIndex];
				captureName = this.BuildScreenshotName(environmentInfo, this.activeMapIndex + 1);
			}
			else if (this.activeMapIndex >= 0 && this.activeMapIndex < this.environments.Count)
			{
				captureName = this.environments[this.activeMapIndex].name;
			}
			else
			{
				captureName = "NoEnvironment";
			}

			string fileName = string.Format(
				"{0}_{1}.png",
				captureName,
				DateTime.Now.ToString("yyyyMMdd_HHmmss_fff"));

			string filePath = Path.Combine(captureDirectory, fileName);
			ScreenCapture.CaptureScreenshot(filePath);

			SIGVerseLogger.Info("FreeCapture screenshot saved. path=" + filePath);
		}

		private string BuildScreenshotName(EnvironmentInfo environmentInfo, int mapNumber)
		{
			string baseName = !string.IsNullOrEmpty(environmentInfo.environmentName) ? environmentInfo.environmentName : "FreeCapture";
			return baseName + "_map" + mapNumber.ToString("D2");
		}
	}
}
