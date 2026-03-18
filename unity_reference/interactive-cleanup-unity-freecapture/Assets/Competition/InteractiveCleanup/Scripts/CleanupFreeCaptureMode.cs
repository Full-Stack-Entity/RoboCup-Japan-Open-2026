using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using SIGVerse.Common;
using UnityEngine;
using UnityEngine.InputSystem;

namespace SIGVerse.Competition.InteractiveCleanup
{
	public class CleanupFreeCaptureMode : MonoBehaviour
	{
		private const float DefaultMoveSpeed = 2.5f;
		private const float FastMoveMultiplier = 3.0f;
		private const float SlowMoveMultiplier = 0.35f;
		private const float LookSensitivity = 0.12f;
		private const string CaptureDirectoryRelativePath = "/../SIGVerseConfig/InteractiveCleanup/Captures/";

		private readonly List<GameObject> environments = new List<GameObject>();

		private Transform captureCameraTransform;
		private Vector3 initialCameraPosition;
		private Quaternion initialCameraRotation;

		private float yaw;
		private float pitch;
		private int activeEnvironmentIndex;
		private bool isLookActive;


		public void Initialize(IEnumerable<GameObject> environmentObjects)
		{
			this.environments.Clear();
			if (environmentObjects != null)
			{
				this.environments.AddRange(environmentObjects.Where(environment => environment != null).Distinct());
			}

			this.captureCameraTransform = this.FindCaptureCameraTransform();
			if (this.captureCameraTransform == null)
			{
				SIGVerseLogger.Error("FreeCapture mode failed: capture camera was not found.");
				this.enabled = false;
				return;
			}

			this.CacheInitialCameraPose();
			this.activeEnvironmentIndex = this.ResolveInitialEnvironmentIndex();

			if (this.environments.Count > 0)
			{
				this.SetActiveEnvironment(this.activeEnvironmentIndex, true);
			}
			else
			{
				SIGVerseLogger.Warn("FreeCapture mode started without configured environments.");
			}

			Time.timeScale = 1.0f;
			this.SetLookActive(false);

			SIGVerseLogger.Info(
				"FreeCapture controls: RMB look, WASD move, Q/E vertical, Shift fast, Ctrl slow, R reset, [/] or PgUp/PgDn switch environment, P screenshot.");
		}

		private Transform FindCaptureCameraTransform()
		{
			Transform avatarCamera = this.transform.Find("AvatarCamera");
			if (avatarCamera != null) { return avatarCamera; }

			Camera[] cameras = this.GetComponentsInChildren<Camera>(true);
			Camera preferredCamera = cameras.FirstOrDefault(camera => camera.gameObject.name == "AvatarCamera");
			if (preferredCamera != null) { return preferredCamera.transform; }

			if (cameras.Length > 0) { return cameras[0].transform; }

			if (Camera.main != null) { return Camera.main.transform; }

			return null;
		}

		private void CacheInitialCameraPose()
		{
			this.initialCameraPosition = this.captureCameraTransform.position;
			this.initialCameraRotation = this.captureCameraTransform.rotation;

			Vector3 eulerAngles = this.captureCameraTransform.rotation.eulerAngles;
			this.yaw = eulerAngles.y;
			this.pitch = NormalizeAngle(eulerAngles.x);
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
			if (mouse == null) { return; }

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

			Vector3 planarForward = this.captureCameraTransform.forward;
			planarForward.y = 0.0f;
			if (planarForward.sqrMagnitude > 0.0001f)
			{
				planarForward.Normalize();
			}
			else
			{
				planarForward = Vector3.forward;
			}

			Vector3 planarRight = this.captureCameraTransform.right;
			planarRight.y = 0.0f;
			if (planarRight.sqrMagnitude > 0.0001f)
			{
				planarRight.Normalize();
			}
			else
			{
				planarRight = Vector3.right;
			}

			Vector3 moveDirection = Vector3.zero;

			if (keyboard.wKey.isPressed) { moveDirection += planarForward; }
			if (keyboard.sKey.isPressed) { moveDirection -= planarForward; }
			if (keyboard.dKey.isPressed) { moveDirection += planarRight; }
			if (keyboard.aKey.isPressed) { moveDirection -= planarRight; }
			if (keyboard.eKey.isPressed) { moveDirection += Vector3.up; }
			if (keyboard.qKey.isPressed) { moveDirection -= Vector3.up; }

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
				this.StepEnvironment(-1);
			}

			if (keyboard.rightBracketKey.wasPressedThisFrame || keyboard.pageDownKey.wasPressedThisFrame)
			{
				this.StepEnvironment(+1);
			}
		}

		private void StepEnvironment(int delta)
		{
			if (this.environments.Count == 0) { return; }

			this.activeEnvironmentIndex = (this.activeEnvironmentIndex + delta + this.environments.Count) % this.environments.Count;
			this.SetActiveEnvironment(this.activeEnvironmentIndex, true);
		}

		private void SetActiveEnvironment(int index, bool resetCamera)
		{
			if (index < 0 || index >= this.environments.Count) { return; }

			for (int i = 0; i < this.environments.Count; i++)
			{
				this.environments[i].SetActive(i == index);
			}
			Physics.SyncTransforms();

			this.activeEnvironmentIndex = index;

			if (resetCamera)
			{
				this.ResetCameraPose();
			}

			SIGVerseLogger.Info("FreeCapture environment switched. name=" + this.environments[index].name);
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

			string environmentName = this.activeEnvironmentIndex >= 0 && this.activeEnvironmentIndex < this.environments.Count
				? this.environments[this.activeEnvironmentIndex].name
				: "NoEnvironment";

			string fileName = string.Format(
				"{0}_{1}.png",
				environmentName,
				DateTime.Now.ToString("yyyyMMdd_HHmmss_fff"));

			string filePath = Path.Combine(captureDirectory, fileName);
			ScreenCapture.CaptureScreenshot(filePath);

			SIGVerseLogger.Info("FreeCapture screenshot saved. path=" + filePath);
		}
	}
}
