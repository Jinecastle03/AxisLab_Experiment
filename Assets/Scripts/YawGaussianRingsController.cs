using System;
using UnityEngine;
using Bhaptics.SDK2;

public class YawGaussianRingsController : MonoBehaviour
{
    public enum SpeedMode { DegreesPerSecond, RadiansPerSecond, CyclesPerSecond }

    [Header("Enable Rings")]
    [SerializeField] private bool useTopRing = true;
    [SerializeField] private bool useMidRing = false;
    [SerializeField] private bool useBottomRing = false;

    [Header("Rings (Yaw Order: front -> right -> back -> left)")]
    [SerializeField] private int[] topRing    = { 12, 13, 14, 15, 31, 30, 29, 28 };
    [SerializeField] private int[] midRing    = { 8,  9,  10, 11, 27, 26, 25, 24 };
    [SerializeField] private int[] bottomRing = { 4,  5,  6,  7,  23, 22, 21, 20 };

    [Header("Ring Intensity Scales")]
    [Range(0f, 1f)] [SerializeField] private float topScale = 1.0f;
    [Range(0f, 1f)] [SerializeField] private float midScale = 0.85f;
    [Range(0f, 1f)] [SerializeField] private float bottomScale = 0.75f;

    [Header("Speed Input")]
    [SerializeField] private SpeedMode speedMode = SpeedMode.DegreesPerSecond;
    [SerializeField] private float angularSpeedDegPerSec = 60f;
    [SerializeField] private float angularSpeedRadPerSec = 1.0f;
    [SerializeField] private float cyclesPerSecond = 0.25f;

    [Header("Intensity")]
    [Range(0f, 1f)] [SerializeField] private float maxIntensity01 = 0.85f;

    [Header("Gaussian (Front/Back vs Side)")]
    [SerializeField] private float sigmaFrontBack = 0.85f;
    [SerializeField] private float sigmaSide = 1.10f;
    [Tooltip("Arc window around each ring's side-edge midpoint where sigmaSide applies")]
    [SerializeField] private float sideSigmaWindow = 0.9f;

    [Header("Side Edge Fix (edge length shortening)")]
    [Tooltip("<1 makes the problematic edge shorter => center passes faster => less 'staying'")]
    [SerializeField] private float sideEdgeLengthMultiplier = 0.55f;

    [Header("Smoothing")]
    [SerializeField] private float smoothingTau = 0.08f;
    [SerializeField] private bool useUnscaledTime = true;

    [Header("bHaptics Call")]
    [SerializeField] private int durationMillis = 50;

    private const int VestMotorCount = 40;

    // ring-specific precomputed geometry
    private RingGeom _topGeom, _midGeom, _botGeom;

    private float[] _raw01;
    private float[] _smoothed01;

    private float _sTop, _sMid, _sBot; // separate progress per ring (keeps motion consistent when toggling rings)
    private bool _running;

    [Serializable]
    private struct EdgePair
    {
        public int from;
        public int to;
        public EdgePair(int f, int t) { from = f; to = t; }
    }

    private struct RingGeom
    {
        public int[] motors;      // length N
        public float[] arcPos;    // length N
        public float[] edgeLen;   // length N
        public float totalLen;
        public EdgePair sideA;
        public EdgePair sideB;
        public int N;
        public bool valid;
    }

    private void Awake()
    {
        _raw01 = new float[VestMotorCount];
        _smoothed01 = new float[VestMotorCount];

        // Build once for play mode, but Start() will rebuild too (so inspector edits apply)
        RebuildAll();
    }

    private void OnDisable()
    {
        StopHaptics();
    }

    private void Update()
    {
        if (!_running) return;

        float dt = useUnscaledTime ? Time.unscaledDeltaTime : Time.deltaTime;
        if (dt <= 0f) return;

        Array.Clear(_raw01, 0, _raw01.Length);

        // advance + apply per enabled ring
        if (useTopRing && _topGeom.valid)
        {
            _sTop = AdvanceS(_sTop, _topGeom.totalLen, dt);
            ApplyRing(_topGeom, _sTop, topScale);
        }
        if (useMidRing && _midGeom.valid)
        {
            _sMid = AdvanceS(_sMid, _midGeom.totalLen, dt);
            ApplyRing(_midGeom, _sMid, midScale);
        }
        if (useBottomRing && _botGeom.valid)
        {
            _sBot = AdvanceS(_sBot, _botGeom.totalLen, dt);
            ApplyRing(_botGeom, _sBot, bottomScale);
        }

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
    // UI hooks
    // =========================

    /// <summary>UI Button에 연결: Start</summary>
    public void StartHaptics()
    {
        // Stop 상태에서 inspector 값을 바꾼 뒤 Start하면 반영되도록 항상 rebuild
        RebuildAll();

        // (재)시작 시 진행도 초기화: "항상 같은 위치에서 시작"
        _sTop = 0f;
        _sMid = 0f;
        _sBot = 0f;

        Array.Clear(_raw01, 0, _raw01.Length);
        Array.Clear(_smoothed01, 0, _smoothed01.Length);

        _running = true;
    }

    /// <summary>UI Button에 연결: Stop</summary>
    public void StopHaptics()
    {
        _running = false;

        Array.Clear(_raw01, 0, _raw01.Length);
        Array.Clear(_smoothed01, 0, _smoothed01.Length);

        // force stop vibration
        int[] zeros = new int[VestMotorCount];
        BhapticsLibrary.PlayMotors((int)PositionType.Vest, zeros, 100);
    }

    // =========================
    // Core per ring
    // =========================

    private float AdvanceS(float s, float totalLen, float dt)
    {
        float cps = GetCyclesPerSecond();
        float speedArcPerSec = cps * totalLen;
        return Wrap(s + speedArcPerSec * dt, totalLen);
    }

    private void ApplyRing(RingGeom g, float s, float ringScale)
    {
        float sideBlend = GetSideBlend(g, s); // 0~1
        float localSigma = sigmaFrontBack * Mathf.Pow(sigmaSide / sigmaFrontBack, sideBlend);
        ApplyRingGaussian(g, s, localSigma, maxIntensity01, ringScale);
    }


    private void ApplyRingGaussian(RingGeom g, float s, float sigmaLocal, float maxA, float scale)
    {
        float inv2sig2 = 1f / (2f * sigmaLocal * sigmaLocal);

        for (int idx = 0; idx < g.N; idx++)
        {
            int motorId = g.motors[idx];
            if (motorId < 0 || motorId >= VestMotorCount) continue;

            float p = g.arcPos[idx];
            float d = ShortestSignedDistanceOnRing(s, p, g.totalLen);

            float val = maxA * Mathf.Exp(-(d * d) * inv2sig2) * scale;

            // 여러 링이 같은 모터를 때릴 가능성은 낮지만, 겹치면 max로
            if (val > _raw01[motorId]) _raw01[motorId] = val;
        }
    }

    private bool IsNearSideEdge(RingGeom g, float s)
    {
        // two midpoints of side edges
        if (TryGetEdgeMidpoint(g, g.sideA, out float midA))
            if (Mathf.Abs(ShortestSignedDistanceOnRing(s, midA, g.totalLen)) < sideSigmaWindow) return true;

        if (TryGetEdgeMidpoint(g, g.sideB, out float midB))
            if (Mathf.Abs(ShortestSignedDistanceOnRing(s, midB, g.totalLen)) < sideSigmaWindow) return true;

        return false;
    }

    private bool TryGetEdgeMidpoint(RingGeom g, EdgePair edge, out float midpoint)
    {
        midpoint = 0f;

        for (int i = 0; i < g.N; i++)
        {
            int a = g.motors[i];
            int b = g.motors[(i + 1) % g.N];

            if ((a == edge.from && b == edge.to) || (a == edge.to && b == edge.from))
            {
                float aPos = g.arcPos[i];
                float eLen = g.edgeLen[i];
                midpoint = Wrap(aPos + eLen * 0.5f, g.totalLen);
                return true;
            }
        }
        return false;
    }

    // =========================
    // Build ring geometry
    // =========================

    private void RebuildAll()
    {
        // side edges per ring (너가 준 값)
        _topGeom = BuildRingGeom(topRing,
            new EdgePair(15, 31),
            new EdgePair(28, 12));

        _midGeom = BuildRingGeom(midRing,
            new EdgePair(11, 27),
            new EdgePair(24, 8));

        _botGeom = BuildRingGeom(bottomRing,
            new EdgePair(7, 23),
            new EdgePair(20, 4));
    }

    private RingGeom BuildRingGeom(int[] motors, EdgePair sideA, EdgePair sideB)
    {
        RingGeom g = new RingGeom
        {
            motors = motors,
            sideA = sideA,
            sideB = sideB,
            valid = false
        };

        if (motors == null || motors.Length < 3)
        {
            Debug.LogError("Ring motors invalid.");
            return g;
        }

        g.N = motors.Length;
        g.arcPos = new float[g.N];
        g.edgeLen = new float[g.N];

        // default edge length = 1
        for (int i = 0; i < g.N; i++) g.edgeLen[i] = 1.0f;

        // shorten side edges if they exist as adjacent pairs
        ApplyEdgeLenMultiplierIfAdjacent(ref g, sideA, sideEdgeLengthMultiplier);
        ApplyEdgeLenMultiplierIfAdjacent(ref g, sideB, sideEdgeLengthMultiplier);

        // cumulative arc positions
        g.arcPos[0] = 0f;
        for (int i = 1; i < g.N; i++)
            g.arcPos[i] = g.arcPos[i - 1] + g.edgeLen[i - 1];

        g.totalLen = g.arcPos[g.N - 1] + g.edgeLen[g.N - 1];
        g.valid = true;
        return g;
    }

    private void ApplyEdgeLenMultiplierIfAdjacent(ref RingGeom g, EdgePair edge, float mult)
    {
        for (int i = 0; i < g.N; i++)
        {
            int a = g.motors[i];
            int b = g.motors[(i + 1) % g.N];
            if ((a == edge.from && b == edge.to) || (a == edge.to && b == edge.from))
            {
                g.edgeLen[i] *= mult;
                return;
            }
        }
        // 인접하지 않으면 그냥 스킵 (근데 너가 준 pair들은 각 ring에서 인접으로 들어있어야 정상)
    }

    // =========================
    // Speed + Helpers
    // =========================

    private float GetCyclesPerSecond()
    {
        switch (speedMode)
        {
            case SpeedMode.DegreesPerSecond:
                return angularSpeedDegPerSec / 360f;
            case SpeedMode.RadiansPerSecond:
                return angularSpeedRadPerSec / (2f * Mathf.PI);
            case SpeedMode.CyclesPerSecond:
            default:
                return cyclesPerSecond;
        }
    }

    private static float Wrap(float x, float mod)
    {
        x %= mod;
        if (x < 0f) x += mod;
        return x;
    }

    private static float ShortestSignedDistanceOnRing(float s, float p, float totalLen)
    {
        float d = s - p;
        d = (d + totalLen * 0.5f) % totalLen;
        if (d < 0f) d += totalLen;
        d -= totalLen * 0.5f;
        return d;
    }
    // side edge에 얼마나 가까운지 [0,1]로 반환
    private float GetSideBlend(RingGeom g, float s)
    {
        float best = 0f;

        if (TryGetEdgeMidpoint(g, g.sideA, out float midA))
        {
            float d = Mathf.Abs(ShortestSignedDistanceOnRing(s, midA, g.totalLen));
            best = Mathf.Max(best, 1f - d / sideSigmaWindow);
        }

        if (TryGetEdgeMidpoint(g, g.sideB, out float midB))
        {
            float d = Mathf.Abs(ShortestSignedDistanceOnRing(s, midB, g.totalLen));
            best = Mathf.Max(best, 1f - d / sideSigmaWindow);
        }

        // clamp + smoothstep (중요)
        best = Mathf.Clamp01(best);
        return best * best * (3f - 2f * best); // smoothstep
    }

}
