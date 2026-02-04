using System;
using UnityEngine;
using TMPro;
using Bhaptics.SDK2;

public class VRRotationHapticsManualController : MonoBehaviour
{
    public enum AxisType { Yaw, Roll, Pitch }
    public enum HapticCondition { GuideDirection, OppositeDirection, RandomMotors, None }

    [Header("References")]
    [Tooltip("XR Origin 또는 OVRCameraRig '루트 Transform' (Main Camera X)")]
    public Transform rigRoot;

    // 네 프로젝트 타입에 맞춰서 연결 (클래스명이 다르면 Inspector에서 레퍼런스 타입도 바꿔야 함)
    public YawGaussian yawController;
    public RollGaussian rollController;
    public PitchGaussian pitchController;

    [Header("UI")]
    [Tooltip("카운트다운/상태를 표시할 TextMeshProUGUI")]
    public TMP_Text statusText;

    [Header("Current Setting (Inspector에서 바꾸고 Play/Stop)")]
    public AxisType axis = AxisType.Yaw;
    public HapticCondition condition = HapticCondition.GuideDirection;

    [Header("Speeds (Visual vs Haptics)")]
    [Tooltip("deg/sec. 화면(카메라/rig) 회전 속도")]
    public float visualAngularSpeedDegPerSec = 45f;

    [Tooltip("deg/sec. 수트(햅틱) 진행 속도. 시각 속도와 분리 가능")]
    public float hapticAngularSpeedDegPerSec = 90f;

    [Tooltip("minutes. 0이면 자동 종료 없음")]
    public float playDurationMinutes = 0f;

    [Tooltip("Time.timeScale 영향 안 받게")]
    public bool useUnscaledTime = true;

    [Header("Haptics - Intensity per Axis")]
    [Range(0f, 1f)] public float yawMaxIntensity01 = 0.30f;
    [Range(0f, 1f)] public float rollMaxIntensity01 = 0.30f;
    [Range(0f, 1f)] public float pitchMaxIntensity01 = 0.30f;

    [Header("Normalization (optional)")]
    public bool useNormalization = false;
    public YawGaussian.NormalizationMode yawNormalizationMode = YawGaussian.NormalizationMode.None;
    public PitchGaussian.NormalizationMode pitchNormalizationMode = PitchGaussian.NormalizationMode.None;
    public float energyTargetMotors = 2.5f;

    [Header("Haptics - Yaw Shaping")]
    [Range(0f, 0.4f)] public float yawCutoff01 = 0.05f;
    [Range(0f, 0.5f)] public float yawPerceptualThreshold01 = 0.05f;
    [Range(0f, 0.2f)] public float yawMinOn01 = 0.00f;
    public float yawSmoothingTau = 0.08f;
    public int yawDurationMillis = 50;

    [Header("Haptics - Yaw Sigmas")]
    public float yawSigmaFrontBack = 0.70f;
    public float yawSigmaSeam = 0.90f;

    [Header("Haptics - Pitch Shaping")]
    [Range(0f, 0.4f)] public float pitchCutoff01 = 0.05f;
    [Range(0f, 0.5f)] public float pitchPerceptualThreshold01 = 0.05f;
    [Range(0f, 0.2f)] public float pitchMinOn01 = 0.00f;
    public float pitchSmoothingTau = 0.08f;
    public int pitchDurationMillis = 50;

    [Header("Haptics - Pitch Sigmas")]
    public float pitchSigmaMain = 0.70f;
    public float pitchSigmaSeam = 0.90f;

    [Header("Haptics - Roll Shape")]
    public float rollSigmaStepsMain = 0.90f;
    public float rollSigmaStepsSub = 0.75f;
    public int rollNeighborCountMain = 2;
    public int rollNeighborCountSub = 1;
    [Range(0f, 0.4f)] public float rollCutoff01 = 0.02f;
    [Range(0f, 1f)] public float rollMainScale = 1.0f;
    [Range(0f, 1f)] public float rollSubScale = 0.9f;
    public float rollSmoothingTau = 0.08f;
    public int rollDurationMillis = 50;

    [Header("Random Motors Condition")]
    public float randomUpdateInterval = 0.10f;
    [Range(1, 10)] public int randomMotorCount = 3;
    [Range(0f, 1f)] public float randomIntensity01 = 0.25f;
    public int randomDurationMs = 60;

    [Header("FMS Reminder (Beep)")]
    [Tooltip("0이면 비활성화. 예: 30이면 30초마다 삐 소리")]
    public float fmsReminderIntervalSeconds = 30f;

    [Tooltip("삐 소리를 재생할 AudioSource (UI용 추천)")]
    public AudioSource uiAudioSource;

    [Tooltip("삐 소리 AudioClip (짧은 wav/mp3)")]
    public AudioClip beepClip;

    [Range(0f, 1f)] public float beepVolume = 0.8f;

    [Tooltip("삐 소리 울릴 때 화면에도 잠깐 'FMS' 표시할지")]
    public bool showFmsTextOnBeep = true;

    public float fmsTextSeconds = 0.5f;

    // ===== runtime =====
    [SerializeField, Tooltip("현재 실행 중 여부(읽기용)")]
    private bool isRunning = false;

    private Coroutine _playRoutine;
    private Coroutine _autoStopRoutine;
    private Coroutine _fmsBeepRoutine;

    private float _randomElapsed = 0f;
    private readonly int[] _randomMotors = new int[32];

    private void OnDisable()
    {
        StopNow();
    }

    private void Update()
    {
        if (!isRunning) return;

        float dt = useUnscaledTime ? Time.unscaledDeltaTime : Time.deltaTime;
        if (dt <= 0f) return;

        RotateRig(dt);

        if (condition == HapticCondition.RandomMotors)
        {
            _randomElapsed += dt;
            if (_randomElapsed >= Mathf.Max(0.01f, randomUpdateInterval))
            {
                _randomElapsed = 0f;
                SendRandomMotorsOnce();
            }
        }
    }

    // ======================
    // Inspector 버튼으로 누를 API
    // ======================
    public void PlayNow()
    {
        if (rigRoot == null)
        {
            Debug.LogError("[Manual] rigRoot가 비어있음. XR Origin/OVRCameraRig 루트를 넣어.");
            return;
        }

        if (_playRoutine != null) StopCoroutine(_playRoutine);
        if (_autoStopRoutine != null) StopCoroutine(_autoStopRoutine);
        if (_fmsBeepRoutine != null) StopCoroutine(_fmsBeepRoutine);

        _playRoutine = StartCoroutine(PlaySequence());
    }

    public void StopNow()
    {
        isRunning = false;
        _randomElapsed = 0f;

        if (_playRoutine != null) { StopCoroutine(_playRoutine); _playRoutine = null; }
        if (_autoStopRoutine != null) { StopCoroutine(_autoStopRoutine); _autoStopRoutine = null; }
        if (_fmsBeepRoutine != null) { StopCoroutine(_fmsBeepRoutine); _fmsBeepRoutine = null; }

        StopAllHapticsHard();
        SetStatus("");
    }

    private System.Collections.IEnumerator PlaySequence()
    {
        StopAllHapticsHard();
        ApplyCommonHapticParams();

        SetStatus("3");
        yield return WaitSeconds(1f);
        SetStatus("2");
        yield return WaitSeconds(1f);
        SetStatus("1");
        yield return WaitSeconds(1f);
        SetStatus("GO");

        isRunning = true;
        _randomElapsed = 0f;

        // FMS 알림 시작
        StartFmsReminderIfNeeded();

        float baseHaptic = hapticAngularSpeedDegPerSec;

        // 조건에 따라 부호 부여
        float hapticSpeed =
            (condition == HapticCondition.OppositeDirection) ? -baseHaptic :
            (condition == HapticCondition.GuideDirection) ? baseHaptic :
            0f;

        if (condition == HapticCondition.GuideDirection || condition == HapticCondition.OppositeDirection)
        {
            StartAxisHaptics(hapticSpeed);
        }
        else if (condition == HapticCondition.RandomMotors)
        {
            Array.Clear(_randomMotors, 0, _randomMotors.Length);
        }

        if (playDurationMinutes > 0f)
        {
            _autoStopRoutine = StartCoroutine(AutoStopAfter(playDurationMinutes * 60f));
        }

        yield return WaitSeconds(0.4f);
        SetStatus("");
    }

    private System.Collections.IEnumerator AutoStopAfter(float seconds)
    {
        yield return WaitSeconds(Mathf.Max(0.01f, seconds));

        if (!isRunning) yield break;

        isRunning = false;
        _randomElapsed = 0f;

        if (_fmsBeepRoutine != null) { StopCoroutine(_fmsBeepRoutine); _fmsBeepRoutine = null; }

        StopAllHapticsHard();

        SetStatus("Finished");
        yield return WaitSeconds(1.2f);
        SetStatus("");
    }

    private System.Collections.IEnumerator WaitSeconds(float seconds)
    {
        if (useUnscaledTime) yield return new WaitForSecondsRealtime(seconds);
        else yield return new WaitForSeconds(seconds);
    }

    private void SetStatus(string msg)
    {
        if (statusText != null) statusText.text = msg;
    }

    // ======================
    // 내부 동작
    // ======================
    private void RotateRig(float dt)
    {
        Vector3 axisVec = axis switch
        {
            AxisType.Yaw => Vector3.up,
            AxisType.Roll => Vector3.forward,
            AxisType.Pitch => Vector3.right,
            _ => Vector3.up
        };

        float angle = visualAngularSpeedDegPerSec * dt;
        rigRoot.Rotate(axisVec, angle, Space.Self);
    }

    private void ApplyCommonHapticParams()
    {
        // axis별 intensity
        if (yawController != null) yawController.SetMaxIntensity01(yawMaxIntensity01);
        if (pitchController != null) pitchController.SetMaxIntensity01(pitchMaxIntensity01);

        if (rollController != null)
        {
            rollController.SetMaxIntensity01(rollMaxIntensity01);
            rollController.SetSigmaSteps(rollSigmaStepsMain, rollSigmaStepsSub);
            rollController.SetNeighborCounts(rollNeighborCountMain, rollNeighborCountSub);
            rollController.SetCutoff01(rollCutoff01);
            rollController.SetGroupScales(rollMainScale, rollSubScale);
            rollController.SetSmoothingTau(rollSmoothingTau);
            rollController.SetDurationMillis(rollDurationMillis);
        }

        // Yaw 상세 파라미터
        if (yawController != null)
        {
            yawController.SetSigmaForStage(YawGaussian.YawStage.Front, yawSigmaFrontBack);
            yawController.SetSigmaForStage(YawGaussian.YawStage.Side, yawSigmaSeam);

            yawController.SetCutoff01(yawCutoff01);
            yawController.SetPerceptualThreshold01(yawPerceptualThreshold01);
            yawController.SetMinOn01(yawMinOn01);
            yawController.SetSmoothingTau(yawSmoothingTau);
            yawController.SetDurationMillis(yawDurationMillis);

            if (useNormalization)
                yawController.SetNormalizationMode(yawNormalizationMode, energyTargetMotors);
            else
                yawController.SetNormalizationMode(YawGaussian.NormalizationMode.None, energyTargetMotors);
        }

        // Pitch 상세 파라미터
        if (pitchController != null)
        {
            pitchController.SetSigmaForStage(PitchGaussian.PitchStage.Front, pitchSigmaMain);
            pitchController.SetSigmaForStage(PitchGaussian.PitchStage.Top, pitchSigmaSeam);

            pitchController.SetCutoff01(pitchCutoff01);
            pitchController.SetPerceptualThreshold01(pitchPerceptualThreshold01);
            pitchController.SetMinOn01(pitchMinOn01);
            pitchController.SetSmoothingTau(pitchSmoothingTau);
            pitchController.SetDurationMillis(pitchDurationMillis);

            if (useNormalization)
                pitchController.SetNormalizationMode(pitchNormalizationMode, energyTargetMotors);
            else
                pitchController.SetNormalizationMode(PitchGaussian.NormalizationMode.None, energyTargetMotors);
        }
    }

    private void StartAxisHaptics(float hapticSpeedDegPerSecSigned)
    {
        switch (axis)
        {
            case AxisType.Yaw:
                if (yawController == null) { Debug.LogError("[Manual] yawController null"); return; }
                yawController.StopAll();
                yawController.StartStage(YawGaussian.YawStage.Front, hapticSpeedDegPerSecSigned);
                break;

            case AxisType.Roll:
                if (rollController == null) { Debug.LogError("[Manual] rollController null"); return; }
                rollController.StopHaptics();
                // ✅ 핵심: 이제 Roll도 signed speed를 그대로 넘김 (롤 컨트롤러가 부호를 반영하도록 수정 필요)
                rollController.StartStage(hapticSpeedDegPerSecSigned);
                break;

            case AxisType.Pitch:
                if (pitchController == null) { Debug.LogError("[Manual] pitchController null"); return; }
                pitchController.StopAll();
                pitchController.StartStage(PitchGaussian.PitchStage.Front, hapticSpeedDegPerSecSigned);
                break;
        }
    }

    // ===== Random Motors =====
    private void SendRandomMotorsOnce()
    {
        Array.Clear(_randomMotors, 0, _randomMotors.Length);

        int k = Mathf.Clamp(randomMotorCount, 1, 10);
        int intensity = Mathf.Clamp(Mathf.RoundToInt(Mathf.Clamp01(randomIntensity01) * 100f), 0, 100);

        for (int i = 0; i < k; i++)
        {
            int tries = 0;
            while (tries++ < 50)
            {
                int m = UnityEngine.Random.Range(0, 32);
                if (_randomMotors[m] == 0)
                {
                    _randomMotors[m] = intensity;
                    break;
                }
            }
        }

        BhapticsLibrary.PlayMotors((int)PositionType.Vest, _randomMotors, Mathf.Max(10, randomDurationMs));
    }

    // ===== Stop Hard =====
    private void StopAllHapticsHard()
    {
        if (yawController != null) yawController.StopAll();
        if (rollController != null) rollController.StopHaptics();
        if (pitchController != null) pitchController.StopAll();

        Array.Clear(_randomMotors, 0, _randomMotors.Length);
        BhapticsLibrary.PlayMotors((int)PositionType.Vest, _randomMotors, 60);
    }

    // ===== FMS Reminder =====
    private void StartFmsReminderIfNeeded()
    {
        if (_fmsBeepRoutine != null)
        {
            StopCoroutine(_fmsBeepRoutine);
            _fmsBeepRoutine = null;
        }

        if (fmsReminderIntervalSeconds <= 0f) return;
        _fmsBeepRoutine = StartCoroutine(FmsBeepLoop());
    }

    private System.Collections.IEnumerator FmsBeepLoop()
    {
        float interval = Mathf.Max(0.1f, fmsReminderIntervalSeconds);

        while (isRunning)
        {
            yield return WaitSeconds(interval);
            if (!isRunning) yield break;

            PlayBeep();

            if (showFmsTextOnBeep)
            {
                SetStatus("FMS");
                yield return WaitSeconds(Mathf.Max(0.05f, fmsTextSeconds));
                SetStatus("");
            }
        }
    }

    private void PlayBeep()
    {
        if (uiAudioSource == null || beepClip == null)
        {
            Debug.Log("[Manual] FMS Beep: uiAudioSource 또는 beepClip이 비어있어서 소리를 못 냄");
            return;
        }

        uiAudioSource.PlayOneShot(beepClip, beepVolume);
    }
}
