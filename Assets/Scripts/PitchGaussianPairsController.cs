using System;
using UnityEngine;
using Bhaptics.SDK2;

public class PitchGaussianPairsController : MonoBehaviour
{
    public enum SpeedMode { DegreesPerSecond, RadiansPerSecond, CyclesPerSecond }

    [Header("Enable Rings")]
    [SerializeField] private bool useMidRing = true;
    [SerializeField] private bool useEdgeRing = true;

    [Header("Speed Input")]
    [SerializeField] private SpeedMode speedMode = SpeedMode.DegreesPerSecond;
    [SerializeField] private float angularSpeedDegPerSec = 60f;
    [SerializeField] private float angularSpeedRadPerSec = 1.0f;
    [SerializeField] private float cyclesPerSecond = 0.25f;

    [Header("Intensity")]
    [Range(0f, 1f)] [SerializeField] private float maxIntensity01 = 0.85f;

    [Header("Ring Scales")]
    [Range(0f, 1f)] [SerializeField] private float midScale = 1.0f;
    [Range(0f, 1f)] [SerializeField] private float edgeScale = 0.95f;

    [Header("Gaussian Sigmas (Normal vs Top/Bottom)")]
    [Tooltip("Default sigma used for most of the ring.")]
    [SerializeField] private float sigmaNormal = 0.85f;

    [Tooltip("Sigma near Top/Bottom boundary segments (can be slightly larger to smooth).")]
    [SerializeField] private float sigmaTopBottom = 1.00f;

    [Tooltip("How wide around each boundary-midpoint we blend sigmaTopBottom in (in arc units).")]
    [SerializeField] private float topBottomWindow = 0.9f;

    [Header("Smoothing")]
    [SerializeField] private float smoothingTau = 0.08f;
    [SerializeField] private bool useUnscaledTime = true;

    [Header("bHaptics Call")]
    [SerializeField] private int durationMillis = 50;

    [Header("Gaussian Window (optional but recommended)")]
    [Tooltip("Only apply gaussian to neighbor pairs around the center (prevents far tail).")]
    [SerializeField] private int neighborPairs = 2; // ±2 => 5 pairs worth
    [SerializeField] private float cutoff01 = 0.02f;

    private const int VestMotorCount = 40;

    // ===== Pair definition =====
    [Serializable]
    public struct MotorPair
    {
        public int a;
        public int b;
        public MotorPair(int a, int b) { this.a = a; this.b = b; }
    }

    [Serializable]
    private struct PairEdge
    {
        public MotorPair from;
        public MotorPair to;
        public PairEdge(MotorPair f, MotorPair t) { from = f; to = t; }
    }

    private struct PairRingGeom
    {
        public MotorPair[] pairs; // length N
        public float[] arcPos;    // length N
        public float totalLen;
        public int N;
        public PairEdge topBottomA;
        public PairEdge topBottomB;
        public bool valid;
    }

    // ===== Your pitch rings =====
    // Mid Ring: (13,14)->(9,10)->(5,6)->(1,2)->(17,18)->(21,22)->(25,26)->(29,30)->back
    [SerializeField] private MotorPair[] midPairs = new MotorPair[]
    {
        new MotorPair(13,14),
        new MotorPair(9,10),
        new MotorPair(5,6),
        new MotorPair(1,2),
        new MotorPair(17,18),
        new MotorPair(21,22),
        new MotorPair(25,26),
        new MotorPair(29,30),
    };

    // Edge Ring: (12,15)->(8,11)->(4,7)->(0,3)->(16,19)->(20,23)->(24,27)->(28,31)->back
    [SerializeField] private MotorPair[] edgePairs = new MotorPair[]
    {
        new MotorPair(12,15),
        new MotorPair(8,11),
        new MotorPair(4,7),
        new MotorPair(0,3),
        new MotorPair(16,19),
        new MotorPair(20,23),
        new MotorPair(24,27),
        new MotorPair(28,31),
    };

    // Top/Bottom boundaries you specified:
    // Mid: (1,2)->(17,18) and (29,30)->(13,14)
    // Edge: (0,3)->(16,19) and (28,31)->(12,15)
    private PairRingGeom _midGeom, _edgeGeom;

    private float[] _raw01;
    private float[] _smoothed01;

    private float _sMid, _sEdge;
    private bool _running;

    private void Awake()
    {
        _raw01 = new float[VestMotorCount];
        _smoothed01 = new float[VestMotorCount];
        RebuildAll();
    }

    private void OnDisable() => StopHaptics();

    private void Update()
    {
        if (!_running) return;

        float dt = useUnscaledTime ? Time.unscaledDeltaTime : Time.deltaTime;
        if (dt <= 0f) return;

        Array.Clear(_raw01, 0, _raw01.Length);

        if (useMidRing && _midGeom.valid)
        {
            _sMid = AdvanceS(_sMid, _midGeom.totalLen, dt);
            ApplyPairRing(_midGeom, _sMid, midScale);
        }

        if (useEdgeRing && _edgeGeom.valid)
        {
            _sEdge = AdvanceS(_sEdge, _edgeGeom.totalLen, dt);
            ApplyPairRing(_edgeGeom, _sEdge, edgeScale);
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

    // ===== UI =====
    public void StartHaptics()
    {
        RebuildAll();

        _sMid = 0f;
        _sEdge = 0f;

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

    // ===== Core =====
    private float AdvanceS(float s, float totalLen, float dt)
    {
        float cps = GetCyclesPerSecond();
        float speedArcPerSec = cps * totalLen;
        return Wrap(s + speedArcPerSec * dt, totalLen);
    }

    private void ApplyPairRing(PairRingGeom g, float s, float scale)
    {
        // sigma blend near top/bottom boundaries (smoothstep)
        float blend = GetTopBottomBlend(g, s); // 0..1
        float sigma = Mathf.Lerp(sigmaNormal, sigmaTopBottom, blend);

        ApplyPairRingGaussian(g, s, sigma, maxIntensity01, scale);
    }

    private void ApplyPairRingGaussian(PairRingGeom g, float s, float sigma, float maxA, float scale)
    {
        float inv2sig2 = 1f / (2f * sigma * sigma);

        // optional neighbor window: only compute around nearest pair index to keep "point" clean
        int centerIdx = FindNearestPairIndex(g, s);

        for (int k = -neighborPairs; k <= neighborPairs; k++)
        {
            int idx = Mod(centerIdx + k, g.N);

            float p = g.arcPos[idx];
            float d = ShortestSignedDistanceOnRing(s, p, g.totalLen);

            float val = maxA * Mathf.Exp(-(d * d) * inv2sig2) * scale;
            if (val < cutoff01) continue;

            MotorPair mp = g.pairs[idx];
            AddMotor(mp.a, val);
            AddMotor(mp.b, val);
        }
    }

    private int FindNearestPairIndex(PairRingGeom g, float s)
    {
        int bestIdx = 0;
        float bestAbs = float.MaxValue;

        for (int i = 0; i < g.N; i++)
        {
            float d = Mathf.Abs(ShortestSignedDistanceOnRing(s, g.arcPos[i], g.totalLen));
            if (d < bestAbs)
            {
                bestAbs = d;
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    private void AddMotor(int motorId, float val01)
    {
        if (motorId < 0 || motorId >= VestMotorCount) return;
        if (val01 > _raw01[motorId]) _raw01[motorId] = val01;
    }

    // ===== Top/Bottom sigma blending =====
    private float GetTopBottomBlend(PairRingGeom g, float s)
    {
        float best = 0f;

        if (TryGetEdgeMidpoint(g, g.topBottomA, out float midA))
        {
            float d = Mathf.Abs(ShortestSignedDistanceOnRing(s, midA, g.totalLen));
            best = Mathf.Max(best, 1f - d / topBottomWindow);
        }

        if (TryGetEdgeMidpoint(g, g.topBottomB, out float midB))
        {
            float d = Mathf.Abs(ShortestSignedDistanceOnRing(s, midB, g.totalLen));
            best = Mathf.Max(best, 1f - d / topBottomWindow);
        }

        best = Mathf.Clamp01(best);
        return best * best * (3f - 2f * best); // smoothstep
    }

    private bool TryGetEdgeMidpoint(PairRingGeom g, PairEdge edge, out float midpoint)
    {
        midpoint = 0f;

        for (int i = 0; i < g.N; i++)
        {
            MotorPair a = g.pairs[i];
            MotorPair b = g.pairs[(i + 1) % g.N];

            if (SamePair(a, edge.from) && SamePair(b, edge.to) ||
                SamePair(a, edge.to) && SamePair(b, edge.from))
            {
                float aPos = g.arcPos[i];
                float eLen = GetEdgeLen(g, i);
                midpoint = Wrap(aPos + eLen * 0.5f, g.totalLen);
                return true;
            }
        }
        return false;
    }

    // Here we use uniform edge length=1 between pair nodes (same as yaw default)
    private float GetEdgeLen(PairRingGeom g, int edgeIndex) => 1.0f;

    private static bool SamePair(MotorPair x, MotorPair y)
    {
        // order-insensitive match
        return (x.a == y.a && x.b == y.b) || (x.a == y.b && x.b == y.a);
    }

    // ===== Build =====
    private void RebuildAll()
    {
        _midGeom = BuildPairRingGeom(
            midPairs,
            new PairEdge(new MotorPair(1, 2), new MotorPair(17, 18)),
            new PairEdge(new MotorPair(29, 30), new MotorPair(13, 14))
        );

        _edgeGeom = BuildPairRingGeom(
            edgePairs,
            new PairEdge(new MotorPair(0, 3), new MotorPair(16, 19)),
            new PairEdge(new MotorPair(28, 31), new MotorPair(12, 15))
        );
    }

    private PairRingGeom BuildPairRingGeom(MotorPair[] pairs, PairEdge topBottomA, PairEdge topBottomB)
    {
        PairRingGeom g = new PairRingGeom
        {
            pairs = pairs,
            topBottomA = topBottomA,
            topBottomB = topBottomB,
            valid = false
        };

        if (pairs == null || pairs.Length < 3)
        {
            Debug.LogError("Pair ring invalid.");
            return g;
        }

        g.N = pairs.Length;
        g.arcPos = new float[g.N];

        // uniform arc spacing = 1 between pair nodes
        g.arcPos[0] = 0f;
        for (int i = 1; i < g.N; i++)
            g.arcPos[i] = g.arcPos[i - 1] + 1.0f;

        g.totalLen = g.N * 1.0f;
        g.valid = true;
        return g;
    }

    // ===== Speed + helpers =====
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

    private static int Mod(int x, int m)
    {
        int r = x % m;
        return r < 0 ? r + m : r;
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
}
