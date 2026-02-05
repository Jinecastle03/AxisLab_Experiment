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

    public YawGaussian yawController;
    public RollGaussian rollController;    // yaw만 쓸 거면 비워도 됨
    public PitchGaussian pitchController;  // yaw만 쓸 거면 비워도 됨

    [Header("UI (optional)")]
    public TMP_Text statusText;

    [Header("Current Setting (Inspector에서 바꾸고 Play/Stop)")]
    public AxisType axis = AxisType.Yaw;
    public HapticCondition condition = HapticCondition.GuideDirection;

    [Header("Speeds (Visual vs Haptics)")]
    [Tooltip("deg/sec. 화면(카메라/rig) 회전 속도")]
    public float visualAngularSpeedDegPerSec = 45f;

    [Tooltip("deg/sec. 수트(햅틱) 진행 속도. Random의 step 계산에도 이 값 사용")]
    public float hapticAngularSpeedDegPerSec = 90f;

    [Tooltip("minutes. 0이면 자동 종료 없음")]
    public float playDurationMinutes = 0f;

    [Tooltip("Time.timeScale 영향 안 받게")]
    public bool useUnscaledTime = true;

    [Header("Runtime (Read Only)")]
    [SerializeField] private float elapsedSeconds = 0f;
    [SerializeField] private int elapsedWholeSeconds = 0;

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

    [Header("Random Motors (Yaw path 내부 랜덤)")]
    [Tooltip("각속도 기반으로 '다음 모터로 넘어가는 시간'을 자동 계산할지")]
    public bool randomUseStepTimeFromAngularSpeed = true;

    [Tooltip("randomUseStepTimeFromAngularSpeed=false 일 때만 사용. 초 단위")]
    public float randomUpdateInterval = 0.10f;

    [Range(0f, 1f)] public float randomIntensity01 = 0.25f;

    [Tooltip("randomUseStepTimeFromAngularSpeed=false 일 때만 사용. ms")]
    public int randomDurationMs = 60;

    public bool randomAvoidImmediateRepeat = true;

    [Tooltip("Yaw 랜덤 풀을 강제로 지정(비워두면 yawController.loopPath 사용)")]
    public int[] randomYawPoolOverride;

    [Header("FMS Reminder (Beep)")]
    [Tooltip("0이면 비활성화. 예: 30이면 30초마다 삐 소리")]
    public float fmsReminderIntervalSeconds = 30f;

    public AudioSource uiAudioSource;
    public AudioClip beepClip;
    [Range(0f, 1f)] public float beepVolume = 0.8f;

    public bool showFmsTextOnBeep = true;
    public float fmsTextSeconds = 0.5f;

    // ===== runtime =====
    [SerializeField, Tooltip("현재 실행 중 여부(읽기용)")]
    private bool isRunning = false;

    private Coroutine _playRoutine;
    private Coroutine _autoStopRoutine;
    private Coroutine _fmsBeepRoutine;

    // Random runtime
    private float _randomElapsed = 0f;
    private int _prevRandomMotor = -1;

    private readonly int[] _randomMotors = new int[32];
    private readonly int[] _randomPool = new int[32];
    private int _randomPoolCount = 0;

    private void OnDisable()
    {
        StopNow();
    }

    private void Update()
    {
        if (!isRunning) return;

        float dt = useUnscaledTime ? Time.unscaledDeltaTime : Time.deltaTime;
        if (dt <= 0f) return;

        // ✅ Inspector용 경과 시간
        elapsedSeconds += dt;
        elapsedWholeSeconds = Mathf.FloorToInt(elapsedSeconds);

        RotateRig(dt);

        // ✅ RandomMotors: yaw path 내부에서만 랜덤 + step마다 1개 모터
        if (condition == HapticCondition.RandomMotors && axis == AxisType.Yaw)
        {
            RefreshYawRandomPool();

            _randomElapsed += dt;
            float stepSec = GetRandomStepIntervalSeconds();
            if (_randomElapsed >= stepSec)
            {
                _randomElapsed = 0f;
                int durationMs = GetRandomDurationMs(stepSec);
                SendRandomMotorOnce(durationMs);
            }
        }
    }

    // ======================
    // Inspector 버튼 API
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
        _prevRandomMotor = -1;

        // 경과시간 리셋(원하면 유지해도 되는데 보통 Stop하면 0으로 두는 게 편함)
        elapsedSeconds = 0f;
        elapsedWholeSeconds = 0;

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

        SetStatus("3"); yield return WaitSeconds(1f);
        SetStatus("2"); yield return WaitSeconds(1f);
        SetStatus("1"); yield return WaitSeconds(1f);
        SetStatus("GO");

        isRunning = true;

        // ✅ 경과시간 시작점 초기화
        elapsedSeconds = 0f;
        elapsedWholeSeconds = 0;

        _randomElapsed = 0f;
        _prevRandomMotor = -1;

        StartFmsReminderIfNeeded();

        float baseHaptic = hapticAngularSpeedDegPerSec;
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
        _prevRandomMotor = -1;

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
                rollController.StartStage(hapticSpeedDegPerSecSigned);
                break;

            case AxisType.Pitch:
                if (pitchController == null) { Debug.LogError("[Manual] pitchController null"); return; }
                pitchController.StopAll();
                pitchController.StartStage(PitchGaussian.PitchStage.Front, hapticSpeedDegPerSecSigned);
                break;
        }
    }

    // ======================
    // Random (Yaw path 내부 랜덤)
    // ======================
    private void RefreshYawRandomPool()
    {
        _randomPoolCount = 0;

        int[] src = null;
        if (randomYawPoolOverride != null && randomYawPoolOverride.Length > 0)
            src = randomYawPoolOverride;
        else if (yawController != null)
            src = yawController.GetLoopPathReadOnly(); // ✅ YawGaussian에 getter 필요

        if (src != null && src.Length > 0)
        {
            for (int i = 0; i < src.Length && _randomPoolCount < _randomPool.Length; i++)
            {
                int idx = src[i];
                if (idx < 0 || idx >= 32) continue;

                bool exists = false;
                for (int j = 0; j < _randomPoolCount; j++)
                    if (_randomPool[j] == idx) { exists = true; break; }

                if (!exists) _randomPool[_randomPoolCount++] = idx;
            }
        }

        // fallback (예상치 못하게 path가 비면 전체 32로)
        if (_randomPoolCount <= 0)
        {
            for (int i = 0; i < 32; i++) _randomPool[i] = i;
            _randomPoolCount = 32;
        }
    }

    private float GetRandomStepIntervalSeconds()
    {
        if (!randomUseStepTimeFromAngularSpeed)
            return Mathf.Max(0.01f, randomUpdateInterval);

        float omega = Mathf.Abs(hapticAngularSpeedDegPerSec);
        if (omega < 1e-3f) return Mathf.Max(0.01f, randomUpdateInterval);

        // ✅ stepSec = 360 / (omega * N), N = yaw path 모터 개수
        int n = Mathf.Max(1, _randomPoolCount);
        return 360f / (omega * n);
    }

    private int GetRandomDurationMs(float stepSec)
    {
        if (!randomUseStepTimeFromAngularSpeed)
            return Mathf.Max(10, randomDurationMs);

        int ms = Mathf.RoundToInt(stepSec * 1000f);
        return Mathf.Clamp(ms, 20, 5000);
    }

    private void SendRandomMotorOnce(int durationMs)
    {
        Array.Clear(_randomMotors, 0, _randomMotors.Length);

        int intensity = Mathf.Clamp(Mathf.RoundToInt(Mathf.Clamp01(randomIntensity01) * 100f), 0, 100);

        int pick = _randomPool[UnityEngine.Random.Range(0, Mathf.Max(1, _randomPoolCount))];

        if (randomAvoidImmediateRepeat && _randomPoolCount > 1)
        {
            int guard = 0;
            while (pick == _prevRandomMotor && guard++ < 20)
                pick = _randomPool[UnityEngine.Random.Range(0, _randomPoolCount)];
        }

        _prevRandomMotor = pick;
        _randomMotors[pick] = intensity;

        BhapticsLibrary.PlayMotors((int)PositionType.Vest, _randomMotors, Mathf.Max(10, durationMs));
    }

    // ======================
    // Stop / FMS
    // ======================
    private void StopAllHapticsHard()
    {
        if (yawController != null) yawController.StopAll();
        if (rollController != null) rollController.StopHaptics();
        if (pitchController != null) pitchController.StopAll();

        Array.Clear(_randomMotors, 0, _randomMotors.Length);
        BhapticsLibrary.PlayMotors((int)PositionType.Vest, _randomMotors, 60);
    }

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

    void Start()
    {
        var avail = OVRPlugin.systemDisplayFrequenciesAvailable;
        Debug.Log("[HMD] Available Hz: " + string.Join(", ", avail));
        Debug.Log("[HMD] Current Hz (before): " + OVRPlugin.systemDisplayFrequency);

        OVRPlugin.systemDisplayFrequency = 120f;
        Debug.Log("[HMD] Current Hz (after set): " + OVRPlugin.systemDisplayFrequency);
    }
}
