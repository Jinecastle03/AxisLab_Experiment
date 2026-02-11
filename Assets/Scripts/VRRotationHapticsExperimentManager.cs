using System;
using UnityEngine;
using TMPro;
using Bhaptics.SDK2;

public class VRRotationHapticsManualController : MonoBehaviour
{
    public enum AxisType { Yaw, Roll, Pitch }
    public enum HapticCondition { Match, Mismatch, RandomMotors, None }

    [Header("References (IMPORTANT)")]
    [Tooltip("HMD 카메라 Transform (예: CenterEyeAnchor / Main Camera). 회전 피벗(중심)으로 사용.")]
    public Transform headTransform;

    [Tooltip("회전시킬 맵/자극 루트(StimulusRoot). 카메라/rig는 절대 돌리지 않음.")]
    public Transform stimulusRoot;

    public YawGaussian yawController;
    public RollGaussian rollController;    // yaw만 쓸 거면 비워도 됨
    public PitchGaussian pitchController;  // yaw만 쓸 거면 비워도 됨

    [Header("UI (optional)")]
    public TMP_Text statusText;

    [Header("Current Setting (Inspector에서 바꾸고 Play/Stop)")]
    public AxisType axis = AxisType.Yaw;
    public HapticCondition condition = HapticCondition.Match;

    [Header("Speeds (Visual vs Haptics)")]
    [Tooltip("deg/sec. 시각(맵/자극) 회전 속도")]
    public float visualAngularSpeedDegPerSec = 45f;

    [Tooltip("deg/sec. 수트(햅틱) 진행 속도. Random의 step 계산에도 이 값 사용")]
    public float hapticAngularSpeedDegPerSec = 90f;

    [Tooltip("minutes. 0이면 자동 종료 없음")]
    public float playDurationMinutes = 0f;

    [Tooltip("Time.timeScale 영향 안 받게")]
    public bool useUnscaledTime = true;

    [Header("Stimulus Rotation")]
    [Tooltip("true면 맵(StimulusRoot) 회전 방향을 반대로 뒤집음")]
    public bool invertStimulusRotation = true;

    [Header("Skybox Rotation")]
    [Tooltip("true면 RenderSettings.skybox 머티리얼의 _Rotation(지원하는 셰이더일 때)을 함께 회전시킵니다.")]
    public bool rotateSkybox = true;

    [Tooltip("Skybox 회전 배수. 보통 1.0")]
    public float skyboxRotationMultiplier = 1f;

    [Tooltip("Play 시작 시 skybox rotation을 0으로 리셋합니다.")]
    public bool resetSkyboxRotationOnPlay = true;

    private Material _skyboxMat;
    private float _skyboxRotationDeg = 0f;

    // ============================================================
    // ✅ Speed Schedule (Yaw only): 40→60→50… 반복 + 간격 인스펙터 제어
    // ============================================================
    [Header("Speed Schedule (Yaw only)")]
    [Tooltip("켜면 Yaw에서 시각/햅틱 속도가 speedSegmentSeconds마다 시퀀스대로 반복됨")]
    public bool useYawSpeedSchedule = false;

    [Tooltip("속도 변경 간격(초). 예: 10=10초마다, 30=30초마다")]
    public float speedSegmentSeconds = 10f;

    [Tooltip("시각 회전 속도 시퀀스(deg/sec). 예: 40,60,50 (계속 반복)")]
    public float[] yawVisualSpeedSequenceDegPerSec = new float[] { 40f, 60f, 50f };

    [Tooltip("햅틱 속도 = (시각 속도 * multiplier). 예: 2면 45->90 비율")]
    public float yawHapticSpeedMultiplierFromVisual = 2.0f;

    

    [Header("Yaw Speed Linking & Smoothing")]
    [Tooltip("true면 (Yaw에서) 햅틱 속도를 시각 속도에 multiplier로 동기화합니다. false면 hapticAngularSpeedDegPerSec 값을 그대로 사용합니다.")]
    public bool yawSyncHapticSpeedToVisual = true;

    [Tooltip("true면 (Yaw에서) 속도 변경이 Lerp로 부드럽게 전환됩니다. false면 즉시(Snap) 바뀝니다.")]
    public bool yawSmoothSpeedTransitions = true;

    [Tooltip("Yaw 속도 전환에 걸리는 시간(초). 0이면 즉시 전환.")]
    public float yawSpeedLerpSeconds = 1.2f;

    [Header("Runtime Speed (Read Only)")]
    [SerializeField] private float _targetVisualSpeedDegPerSec = 0f;
    [SerializeField] private float _currentVisualSpeedDegPerSec = 0f;
    [SerializeField] private float _targetHapticSpeedAbsDegPerSec = 0f;
    [SerializeField] private float _currentHapticSpeedAbsDegPerSec = 0f;

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

    
    public enum RandomOmegaSource { HapticSpeed, VisualSpeed }

    [Tooltip("randomUseStepTimeFromAngularSpeed=true일 때, step 계산에 사용할 각속도 소스")]
    public RandomOmegaSource randomOmegaSource = RandomOmegaSource.HapticSpeed;

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

    // ✅ Speed Schedule runtime
    private Coroutine _yawSpeedScheduleRoutine;
    private int _yawSpeedSeqIndex = 0;

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

        // 경과 시간
        elapsedSeconds += dt;
        elapsedWholeSeconds = Mathf.FloorToInt(elapsedSeconds);

        // ✅ Yaw: 시각속도/햅틱속도 동기화 + Lerp(옵션) 업데이트
        UpdateYawSpeedTargetsAndCurrents(dt);

        // ✅ 핵심: 카메라/rig가 아니라 StimulusRoot(맵)만 회전
        RotateStimulus(dt);

        // ✅ Yaw (Match/Mismatch): Lerp 중에도 햅틱 각속도가 같이 천천히 변하도록 매 프레임 반영
        if (axis == AxisType.Yaw && yawController != null && (condition == HapticCondition.Match || condition == HapticCondition.Mismatch))
        {
            float signed = (condition == HapticCondition.Mismatch) ? -_currentHapticSpeedAbsDegPerSec : _currentHapticSpeedAbsDegPerSec;
            yawController.SetAngularSpeedDegPerSec(signed);
        }
// RandomMotors: yaw path 내부에서만 랜덤 + step마다 1개 모터
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
        isRunning = false;

        if (headTransform == null)
        {
            Debug.LogError("[Manual] headTransform이 비어있음. HMD 카메라(예: CenterEyeAnchor/Main Camera)를 넣어.");
            return;
        }
        if (stimulusRoot == null)
        {
            Debug.LogError("[Manual] stimulusRoot가 비어있음. 맵/자극 루트(StimulusRoot)를 넣어.");
            return;
        }

        if (_playRoutine != null) StopCoroutine(_playRoutine);
        if (_autoStopRoutine != null) StopCoroutine(_autoStopRoutine);
        if (_fmsBeepRoutine != null) StopCoroutine(_fmsBeepRoutine);
        if (_yawSpeedScheduleRoutine != null) StopCoroutine(_yawSpeedScheduleRoutine);

        _playRoutine = StartCoroutine(PlaySequence());
    }

    public void StopNow()
    {
        isRunning = false;

        _randomElapsed = 0f;
        _prevRandomMotor = -1;

        elapsedSeconds = 0f;
        elapsedWholeSeconds = 0;

        if (_playRoutine != null) { StopCoroutine(_playRoutine); _playRoutine = null; }
        if (_autoStopRoutine != null) { StopCoroutine(_autoStopRoutine); _autoStopRoutine = null; }
        if (_fmsBeepRoutine != null) { StopCoroutine(_fmsBeepRoutine); _fmsBeepRoutine = null; }
        if (_yawSpeedScheduleRoutine != null) { StopCoroutine(_yawSpeedScheduleRoutine); _yawSpeedScheduleRoutine = null; }

        StopAllHapticsHard();
        SetStatus("");
    }

    private System.Collections.IEnumerator PlaySequence()
    {
        StopAllHapticsHard();
        ApplyCommonHapticParams();


        CacheSkyboxMaterial();
        SetStatus("3"); yield return WaitSeconds(1f);
        SetStatus("2"); yield return WaitSeconds(1f);
        SetStatus("1"); yield return WaitSeconds(1f);
        SetStatus("GO");

        isRunning = true;

        elapsedSeconds = 0f;
        elapsedWholeSeconds = 0;

        _randomElapsed = 0f;
        _prevRandomMotor = -1;

        StartFmsReminderIfNeeded();

        // (1) Yaw Speed Schedule 초기 적용 (0번째 속도부터)
        InitializeYawSpeedScheduleForRun();


        // Yaw 속도 동기화/스무딩 런타임 초기화 (시작 시점은 즉시 목표값으로 세팅)
        UpdateYawSpeedTargetsAndCurrents(0.016f);
        _currentVisualSpeedDegPerSec = _targetVisualSpeedDegPerSec;
        _currentHapticSpeedAbsDegPerSec = _targetHapticSpeedAbsDegPerSec;

        // (2) 현재 hapticAngularSpeedDegPerSec 기준으로 방향 결정
        float baseHaptic = (axis == AxisType.Yaw) ? _currentHapticSpeedAbsDegPerSec : hapticAngularSpeedDegPerSec;
        float hapticSpeed =
            (condition == HapticCondition.Mismatch) ? -baseHaptic :
            (condition == HapticCondition.Match) ? baseHaptic :
            0f;

        if (condition == HapticCondition.Match || condition == HapticCondition.Mismatch)
        {
            StartAxisHaptics(hapticSpeed);
        }
        else if (condition == HapticCondition.RandomMotors)
        {
            Array.Clear(_randomMotors, 0, _randomMotors.Length);
        }

        // (3) Yaw Speed Schedule 루프 시작
        StartYawSpeedScheduleLoopIfNeeded();

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
        if (_yawSpeedScheduleRoutine != null) { StopCoroutine(_yawSpeedScheduleRoutine); _yawSpeedScheduleRoutine = null; }

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

    private void CacheSkyboxMaterial()
    {
        _skyboxMat = RenderSettings.skybox;
        if (_skyboxMat != null && _skyboxMat.HasProperty("_Rotation"))
        {
            if (resetSkyboxRotationOnPlay)
            {
                _skyboxRotationDeg = 0f;
                _skyboxMat.SetFloat("_Rotation", _skyboxRotationDeg);
            }
            else
            {
                _skyboxRotationDeg = _skyboxMat.GetFloat("_Rotation");
            }
        }
    }


    // ======================
    // Yaw speed sync & smoothing (NO latency attack)
    // ======================
    private void UpdateYawSpeedTargetsAndCurrents(float dt)
    {
        // 기본 target은 Inspector 값
        float targetVisual = Mathf.Max(0f, visualAngularSpeedDegPerSec);
        float targetHapticAbs = Mathf.Max(0f, hapticAngularSpeedDegPerSec);

        // Speed Schedule이 켜져있고 Yaw라면, ApplyYawScheduleSpeed()가 visualAngularSpeedDegPerSec를 갱신해줌
        // 여기서는 "시각->햅틱 동기화" 옵션만 반영
        if (axis == AxisType.Yaw && yawSyncHapticSpeedToVisual)
        {
            float mul = Mathf.Max(0f, yawHapticSpeedMultiplierFromVisual);
            targetHapticAbs = Mathf.Max(0f, targetVisual * mul);

            // 인스펙터에서도 값 확인 가능하도록
            hapticAngularSpeedDegPerSec = targetHapticAbs;
        }

        _targetVisualSpeedDegPerSec = targetVisual;
        _targetHapticSpeedAbsDegPerSec = targetHapticAbs;

        // 초기값 보정 (처음 한번)
        if (_currentVisualSpeedDegPerSec <= 0f && _currentHapticSpeedAbsDegPerSec <= 0f && elapsedSeconds <= 0.0001f)
        {
            _currentVisualSpeedDegPerSec = _targetVisualSpeedDegPerSec;
            _currentHapticSpeedAbsDegPerSec = _targetHapticSpeedAbsDegPerSec;
        }

        bool doLerp = yawSmoothSpeedTransitions && yawSpeedLerpSeconds > 0f;

        // Yaw에서만 스무딩 적용. (Roll/Pitch는 필요하면 동일 패턴으로 확장)
        if (axis != AxisType.Yaw) doLerp = false;

        if (!doLerp)
        {
            _currentVisualSpeedDegPerSec = _targetVisualSpeedDegPerSec;
            _currentHapticSpeedAbsDegPerSec = _targetHapticSpeedAbsDegPerSec;
            return;
        }

        float t = Mathf.Clamp01(dt / Mathf.Max(1e-5f, yawSpeedLerpSeconds));

        _currentVisualSpeedDegPerSec = Mathf.Lerp(_currentVisualSpeedDegPerSec, _targetVisualSpeedDegPerSec, t);
        _currentHapticSpeedAbsDegPerSec = Mathf.Lerp(_currentHapticSpeedAbsDegPerSec, _targetHapticSpeedAbsDegPerSec, t);
    }



    // ======================
    // ✅ 핵심: StimulusRoot 회전 (맵 회전)
    // ======================
    private void RotateStimulus(float dt)
    {
        if (stimulusRoot == null || headTransform == null) return;

        Vector3 axisVec = axis switch
        {
            AxisType.Yaw => Vector3.up,
            AxisType.Roll => Vector3.forward,
            AxisType.Pitch => Vector3.right,
            _ => Vector3.up
        };

        float visualSpeed = (axis == AxisType.Yaw) ? _currentVisualSpeedDegPerSec : visualAngularSpeedDegPerSec;
        float angle = visualSpeed * dt;
        if (invertStimulusRotation) angle = -angle;

        Vector3 pivot = headTransform.position;
        stimulusRoot.RotateAround(pivot, axisVec, angle);

        // Skybox 회전 (Skybox/Panoramic 등 _Rotation 지원 셰이더일 때)
        if (rotateSkybox && _skyboxMat != null && _skyboxMat.HasProperty("_Rotation"))
        {
            _skyboxRotationDeg = (_skyboxRotationDeg + angle * skyboxRotationMultiplier) % 360f;
            if (_skyboxRotationDeg < 0f) _skyboxRotationDeg += 360f;
            _skyboxMat.SetFloat("_Rotation", _skyboxRotationDeg);
        }
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

    // ============================================================
    // Speed Schedule (Yaw only)
    // ============================================================
    private void InitializeYawSpeedScheduleForRun()
    {
        if (!useYawSpeedSchedule) return;
        if (axis != AxisType.Yaw) return;
        if (yawVisualSpeedSequenceDegPerSec == null || yawVisualSpeedSequenceDegPerSec.Length <= 0) return;

        _yawSpeedSeqIndex = 0;
        ApplyYawScheduleSpeed(_yawSpeedSeqIndex);
    }

    private void StartYawSpeedScheduleLoopIfNeeded()
    {
        if (_yawSpeedScheduleRoutine != null)
        {
            StopCoroutine(_yawSpeedScheduleRoutine);
            _yawSpeedScheduleRoutine = null;
        }

        if (!useYawSpeedSchedule) return;
        if (axis != AxisType.Yaw) return;
        if (yawVisualSpeedSequenceDegPerSec == null || yawVisualSpeedSequenceDegPerSec.Length <= 0) return;

        float seg = Mathf.Max(0.01f, speedSegmentSeconds);
        _yawSpeedScheduleRoutine = StartCoroutine(YawSpeedScheduleLoop(seg));
    }

    private System.Collections.IEnumerator YawSpeedScheduleLoop(float segmentSeconds)
    {
        int n = Mathf.Max(1, yawVisualSpeedSequenceDegPerSec.Length);

        while (isRunning)
        {
            yield return WaitSeconds(segmentSeconds);
            if (!isRunning) yield break;

            _yawSpeedSeqIndex = (_yawSpeedSeqIndex + 1) % n;
            ApplyYawScheduleSpeed(_yawSpeedSeqIndex);
        }
    }

    private void ApplyYawScheduleSpeed(int seqIndex)
    {
        int i = Mathf.Clamp(seqIndex, 0, yawVisualSpeedSequenceDegPerSec.Length - 1);

        float v = Mathf.Max(0f, yawVisualSpeedSequenceDegPerSec[i]); // visual deg/s
        visualAngularSpeedDegPerSec = v;

        float hAbs = Mathf.Max(0f, hapticAngularSpeedDegPerSec);

        if (yawSyncHapticSpeedToVisual)
        {
            float mul = Mathf.Max(0f, yawHapticSpeedMultiplierFromVisual);
            hAbs = Mathf.Max(0f, v * mul); // haptic magnitude deg/s
            hapticAngularSpeedDegPerSec = hAbs; // 인스펙터 표시용
        }
        // NOTE:
        // - 여기서는 목표 속도(visualAngularSpeedDegPerSec / hapticAngularSpeedDegPerSec)만 갱신합니다.
        // - 실제 적용(스냅/lerp 포함)은 Update()에서 _current* 값으로 매 프레임 반영합니다.
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
            src = yawController.GetLoopPathReadOnly();

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

        // fallback
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

        float omega;
        if (randomOmegaSource == RandomOmegaSource.VisualSpeed)
            omega = (axis == AxisType.Yaw) ? Mathf.Abs(_currentVisualSpeedDegPerSec) : Mathf.Abs(visualAngularSpeedDegPerSec);
        else
            omega = (axis == AxisType.Yaw) ? Mathf.Abs(_currentHapticSpeedAbsDegPerSec) : Mathf.Abs(hapticAngularSpeedDegPerSec);
        if (omega < 1e-3f) return Mathf.Max(0.01f, randomUpdateInterval);

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
}
