using System;
using UnityEngine;
using Bhaptics.SDK2;

public class RollGaussian : MonoBehaviour
{
    public enum NormalizationMode { None, Peak, Energy }
    public enum SpeedMode { DegreesPerSecond, RadiansPerSecond, CyclesPerSecond }

    [Header("Speed")]
    [SerializeField] private SpeedMode speedMode = SpeedMode.DegreesPerSecond;

    // 내부에서는 "크기"로 저장하고, 방향은 _dirSign으로 관리
    [SerializeField] private float angularSpeedDegPerSec = 60f;
    [SerializeField] private float angularSpeedRadPerSec = 1.0f;
    [SerializeField] private float cyclesPerSecond = 0.25f;

    [Header("Intensity")]
    [Range(0f, 1f)] [SerializeField] private float maxIntensity01 = 0.85f;

    [Header("Normalization (optional)")]
    [SerializeField] private NormalizationMode normalizationMode = NormalizationMode.None;
    [Tooltip("Energy mode only. 'number of motors at full intensity' 느낌. 1.5~3.5 추천")]
    [SerializeField] private float energyTarget = 2.5f;

    [Header("Gaussian Shape")]
    [SerializeField] private float sigmaStepsMain = 0.9f;
    [SerializeField] private float sigmaStepsSub = 0.75f;

    [SerializeField] private int neighborCountMain = 2;
    [SerializeField] private int neighborCountSub = 1;
    [SerializeField] private float cutoff01 = 0.02f;

    [Header("Weights per group")]
    [Range(0f, 1f)] [SerializeField] private float mainScale = 1.0f;
    [Range(0f, 1f)] [SerializeField] private float subScale = 0.9f;

    [Header("Smoothing")]
    [SerializeField] private float smoothingTau = 0.08f;
    [SerializeField] private bool useUnscaledTime = true;

    [Header("bHaptics Call")]
    [SerializeField] private int durationMillis = 50;

    private const int VestMotorCount = 32;

    // ---------------------------
    // Paths (Inspector에서 수정 가능)
    // ---------------------------

    [Header("Roll Paths - FRONT")]
    public bool enableFront = true;

    // 너가 쓰던 기본 FRONT 경로
    [SerializeField] private int[] mainFront = { 12, 13, 14, 15, 11, 7, 3, 2, 1, 0, 4, 8 };
    [SerializeField] private int[] subFront  = { 9, 10, 6, 5 };

    [Header("Roll Paths - BACK (fill indices in Inspector)")]
    public bool enableBack = true;
    [Tooltip("비어있으면 Back은 자동으로 스킵됨")]
    [SerializeField] private int[] mainBack = Array.Empty<int>();
    [Tooltip("비어있으면 Back sub는 스킵됨")]
    [SerializeField] private int[] subBack = Array.Empty<int>();

    [Header("Roll Paths - SIDE (fill indices in Inspector)")]
    public bool enableSide = true;
    [Tooltip("비어있으면 Side는 자동으로 스킵됨")]
    [SerializeField] private int[] mainSide = Array.Empty<int>();
    [Tooltip("비어있으면 Side sub는 스킵됨")]
    [SerializeField] private int[] subSide = Array.Empty<int>();

    [Header("Path Phase Offsets (in main-step units)")]
    [Tooltip("Back이 Front 대비 얼마나 뒤로(phase) 따라올지. 0이면 동일 center, 6이면 반바퀴(12 steps 기준)")]
    public float backMainOffsetSteps = 6f;

    [Tooltip("Side가 Front 대비 얼마나 위상차를 가질지. 보통 3(quarter-turn) 근처부터 튜닝")]
    public float sideMainOffsetSteps = 3f;

    [Tooltip("Back sub offset (sub-step units). 보통 main offset을 sub 스케일로 맞춰줌")]
    public float backSubOffsetSteps = 2f;

    [Tooltip("Side sub offset (sub-step units). 보통 1 정도부터 튜닝")]
    public float sideSubOffsetSteps = 1f;

    [Header("Extra Path Scales")]
    [Range(0f, 1f)] public float backMainScale = 1.0f;
    [Range(0f, 1f)] public float backSubScale = 1.0f;
    [Range(0f, 1f)] public float sideMainScale = 1.0f;
    [Range(0f, 1f)] public float sideSubScale = 1.0f;

    // ---------------------------
    // runtime
    // ---------------------------
    private float[] _raw01 = new float[VestMotorCount];
    private float[] _smoothed01 = new float[VestMotorCount];

    private bool _running;
    private int _step;     // 0..(front main steps-1)
    private float _phase;  // 0..1
    private int _dirSign = 1; // +1 / -1

    private int MainStepsFront => (mainFront != null) ? mainFront.Length : 0;
    private int SubStepsFront  => (subFront != null) ? subFront.Length : 0;

    private void OnDisable() => StopHaptics();

    // =========================
    // Main update
    // =========================
    private void Update()
    {
        if (!_running) return;

        float dt = useUnscaledTime ? Time.unscaledDeltaTime : Time.deltaTime;
        if (dt <= 0f) return;

        int mainSteps = Mathf.Max(1, MainStepsFront);
        float cpsAbs = GetCyclesPerSecondAbs();
        float stepsPerSecSigned = _dirSign * cpsAbs * mainSteps;

        // signed phase advance
        _phase += stepsPerSecSigned * dt;

        // wrap for both directions
        while (_phase >= 1f)
        {
            _phase -= 1f;
            _step = (_step + 1) % mainSteps;
        }
        while (_phase < 0f)
        {
            _phase += 1f;
            _step = (_step - 1 + mainSteps) % mainSteps;
        }

        Array.Clear(_raw01, 0, _raw01.Length);

        // ---- base center (Front 기준) ----
        float centerMain = _step + _phase; // in "front main step" space
        float centerSub = GetSubCenterFromMain(centerMain, mainSteps, Mathf.Max(1, SubStepsFront));

        // FRONT
        if (enableFront)
        {
            ApplyGaussianOnPath(mainFront, centerMain, sigmaStepsMain, neighborCountMain, mainScale);
            ApplyGaussianOnPath(subFront,  centerSub,  sigmaStepsSub,  neighborCountSub,  subScale);
        }

        // BACK (same motion, offset)
        if (enableBack && mainBack != null && mainBack.Length > 0)
        {
            float cMainB = centerMain + backMainOffsetSteps;
            ApplyGaussianOnPath(mainBack, cMainB, sigmaStepsMain, neighborCountMain, mainScale * backMainScale);

            if (subBack != null && subBack.Length > 0)
            {
                float cSubB = centerSub + backSubOffsetSteps;
                ApplyGaussianOnPath(subBack, cSubB, sigmaStepsSub, neighborCountSub, subScale * backSubScale);
            }
        }

        // SIDE (same motion, offset)
        if (enableSide && mainSide != null && mainSide.Length > 0)
        {
            float cMainS = centerMain + sideMainOffsetSteps;
            ApplyGaussianOnPath(mainSide, cMainS, sigmaStepsMain, neighborCountMain, mainScale * sideMainScale);

            if (subSide != null && subSide.Length > 0)
            {
                float cSubS = centerSub + sideSubOffsetSteps;
                ApplyGaussianOnPath(subSide, cSubS, sigmaStepsSub, neighborCountSub, subScale * sideSubScale);
            }
        }

        NormalizeRawIfNeeded();

        // smoothing
        float a = 1f - Mathf.Exp(-dt / Mathf.Max(0.0001f, smoothingTau));
        for (int i = 0; i < VestMotorCount; i++)
            _smoothed01[i] = Mathf.Lerp(_smoothed01[i], _raw01[i], a);

        // send
        int[] motorValues = new int[VestMotorCount];
        for (int i = 0; i < VestMotorCount; i++)
            motorValues[i] = Mathf.RoundToInt(Mathf.Clamp01(_smoothed01[i]) * 100f);

        BhapticsLibrary.PlayMotors((int)PositionType.Vest, motorValues, durationMillis);
    }

    // =========================
    // Public API (Manager hooks)
    // =========================

    // ✅ signed speed 지원: 음수면 반대 방향으로 진행
    public void StartStage(float speedDegPerSecSigned)
    {
        _dirSign = (speedDegPerSecSigned >= 0f) ? 1 : -1;
        SetSpeedDegPerSec(Mathf.Abs(speedDegPerSecSigned));
        StartHaptics();
    }

    public void SetSpeedDegPerSec(float degPerSecAbs)
    {
        speedMode = SpeedMode.DegreesPerSecond;
        angularSpeedDegPerSec = Mathf.Max(0f, degPerSecAbs);
    }

    public void StartHaptics()
    {
        _step = 0;
        _phase = 0f;

        Array.Clear(_raw01, 0, _raw01.Length);
        Array.Clear(_smoothed01, 0, _smoothed01.Length);

        _running = true;
    }

    public void StopHaptics()
    {
        _running = false;

        Array.Clear(_raw01, 0, _raw01.Length);
        Array.Clear(_smoothed01, 0, _smoothed01.Length);

        int[] zeros = new int[VestMotorCount];
        BhapticsLibrary.PlayMotors((int)PositionType.Vest, zeros, 100);
    }

    // =========================
    // Core math
    // =========================

    private void ApplyGaussianOnPath(int[] path, float center, float sigmaSteps, int neighborCount, float scale)
    {
        if (path == null) return;
        int n = path.Length;
        if (n < 2) return;

        center = Repeat(center, n);

        float inv2sig2 = 1f / (2f * sigmaSteps * sigmaSteps);
        int centerIdx = Mathf.FloorToInt(center);

        for (int k = -neighborCount; k <= neighborCount; k++)
        {
            int idx = Mod(centerIdx + k, n);
            int motorId = path[idx];
            if (motorId < 0 || motorId >= VestMotorCount) continue;

            float d = ShortestCyclicDistance(center, idx, n);
            float val = maxIntensity01 * Mathf.Exp(-(d * d) * inv2sig2) * scale;
            if (val < cutoff01) continue;

            if (val > _raw01[motorId]) _raw01[motorId] = val;
        }
    }

    private float GetSubCenterFromMain(float centerMain, int mainSteps, int subSteps)
    {
        if (subSteps <= 0) return 0f;

        // subSteps가 4, mainSteps가 12라면 groupSize=3
        float groupSize = (float)mainSteps / subSteps;
        return centerMain / Mathf.Max(1e-4f, groupSize);
    }

    private float GetCyclesPerSecondAbs()
    {
        switch (speedMode)
        {
            case SpeedMode.DegreesPerSecond: return angularSpeedDegPerSec / 360f;
            case SpeedMode.RadiansPerSecond: return angularSpeedRadPerSec / (2f * Mathf.PI);
            case SpeedMode.CyclesPerSecond:
            default: return Mathf.Abs(cyclesPerSecond);
        }
    }

    private void NormalizeRawIfNeeded()
    {
        if (normalizationMode == NormalizationMode.None) return;

        float peak = Mathf.Max(1e-4f, maxIntensity01);
        float scale = 1f;

        if (normalizationMode == NormalizationMode.Peak)
        {
            float maxRaw = 0f;
            for (int i = 0; i < VestMotorCount; i++) maxRaw = Mathf.Max(maxRaw, _raw01[i]);
            if (maxRaw > 1e-6f) scale = peak / maxRaw;
        }
        else if (normalizationMode == NormalizationMode.Energy)
        {
            float sumNorm = 0f;
            for (int i = 0; i < VestMotorCount; i++) sumNorm += (_raw01[i] / peak);
            float desired = Mathf.Max(1e-3f, energyTarget);
            if (sumNorm > 1e-6f) scale = desired / sumNorm;
        }

        if (Mathf.Abs(scale - 1f) < 1e-4f) return;

        for (int i = 0; i < VestMotorCount; i++)
            _raw01[i] = Mathf.Clamp(_raw01[i] * scale, 0f, peak);
    }

    // =========================
    // Helpers
    // =========================
    private static int Mod(int x, int m)
    {
        int r = x % m;
        return r < 0 ? r + m : r;
    }

    private static float Repeat(float x, float len)
    {
        x %= len;
        if (x < 0f) x += len;
        return x;
    }

    private static float ShortestCyclicDistance(float a, float b, float len)
    {
        float d = a - b;
        d = (d + len * 0.5f) % len;
        if (d < 0f) d += len;
        d -= len * 0.5f;
        return d;
    }

    // =========================
    // Setters called by Manager
    // =========================
    public void SetNormalizationMode(NormalizationMode mode, float energyTargetMotors = 2.5f)
    {
        normalizationMode = mode;
        energyTarget = Mathf.Max(0.01f, energyTargetMotors);
    }

    public void SetMaxIntensity01(float v) => maxIntensity01 = Mathf.Clamp01(v);

    public void SetSigmaSteps(float main, float sub)
    {
        sigmaStepsMain = Mathf.Max(0.01f, main);
        sigmaStepsSub = Mathf.Max(0.01f, sub);
    }

    public void SetNeighborCounts(int main, int sub)
    {
        neighborCountMain = Mathf.Clamp(main, 0, 10);
        neighborCountSub = Mathf.Clamp(sub, 0, 10);
    }

    public void SetCutoff01(float v) => cutoff01 = Mathf.Clamp01(v);

    public void SetGroupScales(float main, float sub)
    {
        mainScale = Mathf.Clamp01(main);
        subScale = Mathf.Clamp01(sub);
    }

    public void SetSmoothingTau(float seconds) => smoothingTau = Mathf.Max(0f, seconds);
    public void SetDurationMillis(int ms) => durationMillis = Mathf.Clamp(ms, 10, 2000);
}
