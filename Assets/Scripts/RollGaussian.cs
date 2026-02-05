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

    [Header("Gaussian Shape (Main/Sub)")]
    [SerializeField] private float sigmaStepsMain = 1.2f;   // Front/Back 쪽 sigma
    [SerializeField] private float sigmaStepsSub  = 1.2f;   // Left/Right 쪽 sigma
    [SerializeField] private int neighborCountMain = 4;     // Front/Back 쪽 neighbor
    [SerializeField] private int neighborCountSub  = 4;     // Left/Right 쪽 neighbor

    [Range(0f, 1f)] [SerializeField] private float cutoff01 = 0.02f;

    [Header("Grouping / Scaling (Main/Sub)")]
    [Range(0f, 1f)] [SerializeField] private float mainScale01 = 1f; // Front/Back
    [Range(0f, 1f)] [SerializeField] private float subScale01  = 1f; // Left/Right

    [Header("Smoothing")]
    [SerializeField] private float smoothingTau = 0.06f; // seconds, 0=off

    [Header("bHaptics Output")]
    [SerializeField] private int durationMillis = 30; // 짧게 갱신 반복

    // =========================
    // Motor layout (Vest 32)
    // =========================
    private const int VestMotorCount = 32;

    // Roll은 "논리 경로"를 따라 center가 도는 느낌으로 구성
    // NOTE: 프로젝트에서 원래 쓰던 mapping이 따로 있다면 PATH/GROUP들을 거기에 맞게 수정해야 함.
    private static readonly int[] PATH = new int[]
    {
        // left -> front -> right -> back (순환)
        0, 1, 2, 3,     4, 5, 6, 7,     8, 9, 10, 11,     12, 13, 14, 15
    };

    // 그룹(예시): 실제 프로젝트 mapping과 다르면 여기만 맞춰주면 됨
    private static readonly int[] GROUP_LEFT  = new int[] { 0, 1, 2, 3 };
    private static readonly int[] GROUP_FRONT = new int[] { 4, 5, 6, 7 };
    private static readonly int[] GROUP_RIGHT = new int[] { 8, 9, 10, 11 };
    private static readonly int[] GROUP_BACK  = new int[] { 12, 13, 14, 15 };

    // =========================
    // Runtime state
    // =========================
    private float _degPhase;         // degrees accumulator [0..360)
    private int _dirSign = +1;       // +1 or -1
    private float _speedAbsDeg;      // positive, if set by manager

    private readonly float[] _raw01 = new float[VestMotorCount];
    private readonly float[] _smoothed01 = new float[VestMotorCount];

    // Reusable buffers (avoid per-frame GC)
    private int[] _motorValues;      // ✅ 재사용 (GC 제거)
    private int[] _zeros;

    private bool _running = false;

    private void Awake()
    {
        _motorValues = new int[VestMotorCount];
        _zeros = new int[VestMotorCount];
    }

    private void OnDisable()
    {
        StopHaptics();
    }

    private void Update()
    {
        if (!_running) return;

        float dt = Time.deltaTime;
        if (dt <= 0f) return;

        float degPerSecAbs = GetSpeedDegPerSecAbs();

        // roll phase update (degrees)
        _degPhase += _dirSign * degPerSecAbs * dt;
        _degPhase = Repeat(_degPhase, 360f);

        // Map degrees [0..360) to center index on PATH [0..n)
        int n = PATH.Length;
        float center = (_degPhase / 360f) * n;

        Array.Clear(_raw01, 0, _raw01.Length);

        // gaussian on path
        ApplyGaussianOnPath(PATH, center);

        // apply main/sub scales:
        // main = Front/Back, sub = Left/Right
        ApplyGroupScale(GROUP_FRONT, mainScale01);
        ApplyGroupScale(GROUP_BACK,  mainScale01);
        ApplyGroupScale(GROUP_LEFT,  subScale01);
        ApplyGroupScale(GROUP_RIGHT, subScale01);

        // normalization
        NormalizeRawIfNeeded();

        // smoothing
        if (smoothingTau <= 0f)
        {
            Array.Copy(_raw01, _smoothed01, _raw01.Length);
        }
        else
        {
            float a = 1f - Mathf.Exp(-dt / Mathf.Max(1e-6f, smoothingTau));
            for (int i = 0; i < VestMotorCount; i++)
                _smoothed01[i] = Mathf.Lerp(_smoothed01[i], _raw01[i], a);
        }

        // send (NO per-frame allocation)
        for (int i = 0; i < VestMotorCount; i++)
            _motorValues[i] = Mathf.RoundToInt(Mathf.Clamp01(_smoothed01[i]) * 100f);

        BhapticsLibrary.PlayMotors((int)PositionType.Vest, _motorValues, durationMillis);
    }

    // =========================
    // Public API (Manager hooks)
    // =========================

    // ✅ signed speed 지원: 음수면 반대 방향
    public void StartStage(float speedDegPerSecSigned)
    {
        _dirSign = (speedDegPerSecSigned >= 0f) ? +1 : -1;
        _speedAbsDeg = Mathf.Abs(speedDegPerSecSigned);
        StartHaptics();
    }

    public void SetSpeedDegPerSec(float speedDegPerSecSigned)
    {
        _dirSign = (speedDegPerSecSigned >= 0f) ? +1 : -1;
        _speedAbsDeg = Mathf.Abs(speedDegPerSecSigned);
    }

    public void StartHaptics()
    {
        _running = true;
        // 필요하면 여기서 리셋:
        // _degPhase = 0f;
    }

    public void StopHaptics()
    {
        _running = false;

        Array.Clear(_raw01, 0, _raw01.Length);
        Array.Clear(_smoothed01, 0, _smoothed01.Length);

        Array.Clear(_motorValues, 0, _motorValues.Length);
        Array.Clear(_zeros, 0, _zeros.Length);
        BhapticsLibrary.PlayMotors((int)PositionType.Vest, _zeros, 100);
    }

    // =========================
    // ✅ Manager 호환 오버로드 (컴파일 에러 해결용)
    // =========================

    // 매니저가 (main, sub) 2개 인자를 넣는 버전
    public void SetSigmaSteps(float main, float sub)
    {
        sigmaStepsMain = Mathf.Max(1e-3f, main);
        sigmaStepsSub  = Mathf.Max(1e-3f, sub);
    }

    public void SetNeighborCounts(int main, int sub)
    {
        neighborCountMain = Mathf.Max(0, main);
        neighborCountSub  = Mathf.Max(0, sub);
    }

    // 매니저가 (mainScale, subScale) 2개 인자를 넣는 버전
    public void SetGroupScales(float main, float sub)
    {
        mainScale01 = Mathf.Clamp01(main);
        subScale01  = Mathf.Clamp01(sub);
    }

    // =========================
    // Optional setters (단일 인자/세부 설정용)
    // =========================
    public void SetNormalizationMode(NormalizationMode mode) => normalizationMode = mode;
    public void SetMaxIntensity01(float v) => maxIntensity01 = Mathf.Clamp01(v);
    public void SetCutoff01(float v) => cutoff01 = Mathf.Clamp01(v);
    public void SetSmoothingTau(float seconds) => smoothingTau = Mathf.Max(0f, seconds);
    public void SetDurationMillis(int ms) => durationMillis = Mathf.Max(1, ms);

    // =========================
    // Core math
    // =========================

    private void ApplyGaussianOnPath(int[] path, float center)
    {
        if (path == null) return;
        int n = path.Length;
        if (n < 2) return;

        center = Repeat(center, n);

        // 샘플링은 main/sub 중 더 큰 neighbor 범위를 돌고,
        // 각 모터가 속한 그룹에 따라 sigma/neighbor를 달리 적용
        int maxNeighbor = Mathf.Max(neighborCountMain, neighborCountSub);
        int c0 = Mathf.FloorToInt(center);

        for (int k = -maxNeighbor; k <= maxNeighbor; k++)
        {
            int idx = Mod(c0 + k, n);
            float dx = ShortestCircularDelta(center, idx, n); // distance in steps
            float absDx = Mathf.Abs(dx);

            int motor = path[idx];
            if (motor < 0 || motor >= VestMotorCount) continue;

            bool isMain = IsMotorInMainGroup(motor); // Front/Back이면 main
            int neighborForThis = isMain ? neighborCountMain : neighborCountSub;
            if (absDx > neighborForThis) continue;

            float sigma = isMain ? sigmaStepsMain : sigmaStepsSub;
            float inv2sig2 = 1f / (2f * sigma * sigma);

            float g = Mathf.Exp(-(dx * dx) * inv2sig2);
            if (g < cutoff01) continue;

            float v = g * maxIntensity01;
            if (v > _raw01[motor]) _raw01[motor] = v; // max-combine
        }
    }

    private bool IsMotorInMainGroup(int motor)
    {
        // main = Front/Back
        return InGroup(motor, GROUP_FRONT) || InGroup(motor, GROUP_BACK);
    }

    private static bool InGroup(int motor, int[] group)
    {
        if (group == null) return false;
        for (int i = 0; i < group.Length; i++)
            if (group[i] == motor) return true;
        return false;
    }

    private void ApplyGroupScale(int[] group, float s)
    {
        if (group == null) return;
        s = Mathf.Clamp01(s);

        for (int i = 0; i < group.Length; i++)
        {
            int motor = group[i];
            if (motor < 0 || motor >= VestMotorCount) continue;
            _raw01[motor] *= s;
        }
    }

    private void NormalizeRawIfNeeded()
    {
        if (normalizationMode == NormalizationMode.None) return;

        if (normalizationMode == NormalizationMode.Peak)
        {
            float peak = 0f;
            for (int i = 0; i < VestMotorCount; i++) peak = Mathf.Max(peak, _raw01[i]);
            if (peak <= 1e-6f) return;

            float inv = 1f / peak;
            for (int i = 0; i < VestMotorCount; i++) _raw01[i] *= inv;
        }
        else if (normalizationMode == NormalizationMode.Energy)
        {
            float sum = 0f;
            for (int i = 0; i < VestMotorCount; i++) sum += _raw01[i] * _raw01[i];
            if (sum <= 1e-8f) return;

            float inv = 1f / Mathf.Sqrt(sum);
            for (int i = 0; i < VestMotorCount; i++) _raw01[i] *= inv;
        }
    }

    private float GetSpeedDegPerSecAbs()
    {
        float absDeg = Mathf.Max(0f, _speedAbsDeg);

        // manager가 안 셋했으면 inspector 값 사용
        if (absDeg <= 1e-6f)
        {
            switch (speedMode)
            {
                case SpeedMode.DegreesPerSecond:
                    absDeg = Mathf.Abs(angularSpeedDegPerSec);
                    break;
                case SpeedMode.RadiansPerSecond:
                    absDeg = Mathf.Abs(angularSpeedRadPerSec) * Mathf.Rad2Deg;
                    break;
                case SpeedMode.CyclesPerSecond:
                    absDeg = Mathf.Abs(cyclesPerSecond) * 360f;
                    break;
            }
        }

        return absDeg;
    }

    // =========================
    // Utility
    // =========================
    private static float Repeat(float v, float len)
    {
        if (len <= 0f) return 0f;
        return v - Mathf.Floor(v / len) * len;
    }

    private static int Mod(int x, int m)
    {
        int r = x % m;
        return (r < 0) ? r + m : r;
    }

    private static float ShortestCircularDelta(float a, float b, int n)
    {
        float d = a - b;
        d = d - Mathf.Round(d / n) * n;
        return d;
    }
}
