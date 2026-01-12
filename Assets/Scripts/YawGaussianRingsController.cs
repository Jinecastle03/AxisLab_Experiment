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

    [Header("Start Index Shift")]
    [Tooltip("Top: 1 => start at motor 13 (avoid initial wrap neighbor issue)")]
    [Range(0, 7)] [SerializeField] private int startIndexTop = 1;
    [Range(0, 7)] [SerializeField] private int startIndexMid = 0;
    [Range(0, 7)] [SerializeField] private int startIndexBottom = 0;

    [Header("Local Cutoff (4-motor crossfade)")]
    [Tooltip("Gaussian cutoff on normalized [0..1] (tails).")]
    [Range(0f, 0.2f)] [SerializeField] private float cutoff01 = 0.04f;

    [Header("Perceptual Threshold (Soft)")]
    [Tooltip("Below this normalized intensity, treat as 0 (but smoothly remap above threshold).")]
    [Range(0f, 0.3f)] [SerializeField] private float perceptualThresholdFrontBack01 = 0.08f;

    [Tooltip("Side often needs a slightly higher threshold or different value.")]
    [Range(0f, 0.3f)] [SerializeField] private float perceptualThresholdSide01 = 0.10f;

    [Header("Smoothing")]
    [Tooltip("Smooth motor outputs to reduce 'staccato' feel.")]
    [SerializeField] private float smoothingTau = 0.12f;
    [SerializeField] private bool useUnscaledTime = true;

    [Header("bHaptics Call")]
    [SerializeField] private int durationMillis = 50;

    [Header("Output Shaping (Gamma + Soft Floor)")]
    [Tooltip("Gamma < 1 boosts low intensities (reduces 'quantized' feel). 0.55~0.85 recommended.")]
    [SerializeField] private float outputGamma = 0.65f;

    [Tooltip("Minimum perceptible intensity once a motor is 'on'. 0.08~0.15 recommended.")]
    [SerializeField] private float minOn01 = 0.10f;

    [Tooltip("How softly we enter the floor (bigger = smoother, smaller = more aggressive). 0.15~0.35 recommended.")]
    [SerializeField] private float minOnBlend01 = 0.25f;


    [Header("Side Edge Fix (optional)")]
    [Tooltip("Keep 1.0 for uniform speed while debugging choppiness.")]
    [SerializeField] private float sideEdgeLengthMultiplier = 1.0f;

    private const int VestMotorCount = 40;

    private RingGeom _topGeom, _midGeom, _botGeom;
    private float[] _raw01;
    private float[] _smoothed01;

    private float _sTop, _sMid, _sBot;
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
        public int[] motors;
        public float[] arcPos;
        public float[] edgeLen;
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
        RebuildAll();
    }

    private void OnDisable() => StopHaptics();

    private void Update()
    {
        if (!_running) return;

        float dt = useUnscaledTime ? Time.unscaledDeltaTime : Time.deltaTime;
        if (dt <= 0f) return;

        Array.Clear(_raw01, 0, _raw01.Length);

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

        // Smooth outputs
        float a = 1f - Mathf.Exp(-dt / Mathf.Max(0.0001f, smoothingTau));
        for (int i = 0; i < VestMotorCount; i++)
            _smoothed01[i] = Mathf.Lerp(_smoothed01[i], _raw01[i], a);

        int[] motorValues = new int[VestMotorCount];
        for (int i = 0; i < VestMotorCount; i++)
        {
            float v = Mathf.Clamp01(_smoothed01[i]);

            // (A) Gamma shaping: low intensity boost to avoid "steppy" feel near threshold
            // gamma < 1 => boosts low values
            v = Mathf.Pow(v, Mathf.Max(0.01f, outputGamma));

            // (B) Soft floor: if motor is on (>0), ensure it reaches at least minOn01,
            // but enter smoothly to avoid popping.
            if (v > 0f && minOn01 > 0f)
            {
                float blend = Mathf.Max(1e-4f, minOnBlend01);
                float t = Mathf.Clamp01(v / blend);                 // 0..1
                float smooth = t * t * (3f - 2f * t);               // smoothstep
                float floor = minOn01 * smooth;                     // 0..minOn01
                v = Mathf.Max(v, floor);
            }

            motorValues[i] = Mathf.RoundToInt(Mathf.Clamp01(v) * 100f);
        }


        BhapticsLibrary.PlayMotors((int)PositionType.Vest, motorValues, durationMillis);
        // --- Debug (optional) ---
        if (Time.frameCount % 1 == 0) // 10프레임마다
        {
            float sum = 0f;
            int dom = -1;
            float domVal = 0f;
            for (int i = 0; i < VestMotorCount; i++)
            {
                float v = _smoothed01[i];
                sum += v;
                if (v > domVal) { domVal = v; dom = i; }
            }
            Debug.Log($"[HAPT] sum={sum:F3}, dom={dom}, domVal={domVal:F3}");
        }

    }

    // ========= UI hooks =========
    public void StartHaptics()
    {
        RebuildAll();

        _sTop = GetStartS(_topGeom, startIndexTop);      // ✅ 13부터 시작
        _sMid = GetStartS(_midGeom, startIndexMid);
        _sBot = GetStartS(_botGeom, startIndexBottom);

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

    private float GetStartS(RingGeom g, int startIdx)
    {
        if (!g.valid || g.N == 0) return 0f;
        int idx = Mathf.Clamp(startIdx, 0, g.N - 1);
        return Wrap(g.arcPos[idx], g.totalLen);
    }

    // ========= Core =========
    private float AdvanceS(float s, float totalLen, float dt)
    {
        float cps = GetCyclesPerSecond();
        float speedArcPerSec = cps * totalLen;
        return Wrap(s + speedArcPerSec * dt, totalLen);
    }

    private void ApplyRing(RingGeom g, float s, float ringScale)
    {
        float sideBlend = GetSideBlend(g, s); // 0..1

        // sigma geometric interpolation (log-domain blend) -> natural for scale params
        float localSigma = sigmaFrontBack * Mathf.Pow(sigmaSide / sigmaFrontBack, sideBlend);

        // perceptual threshold by region
        float thr = Mathf.Lerp(perceptualThresholdFrontBack01, perceptualThresholdSide01, sideBlend);

        ApplyGaussian_Local4Crossfade(g, s, localSigma, maxIntensity01, ringScale, thr);
    }

    /// <summary>
    /// 핵심: "centerIdx를 nearest로 툭 바꾸는 방식" 대신
    /// s가 속한 구간 i->i+1 을 찾고, 4개 (i-1,i,i+1,i+2)만 사용.
    /// i->i+1 가중치는 SmoothStep으로 연속 변화 => front 끊김(스냅) 크게 감소.
    /// </summary>
    private void ApplyGaussian_Local4Crossfade(RingGeom g, float s, float sigmaLocal, float maxA, float scale, float perceptualThreshold01)
    {
        float peak = Mathf.Max(0.0001f, maxA * scale);
        float inv2sig2 = 1f / (2f * sigmaLocal * sigmaLocal);

        // 1) Find segment i such that s in [arcPos[i], arcPos[i]+edgeLen[i]) (wrapped)
        int iSeg = FindSegmentIndex(g, s);
        int i0 = Mod(iSeg - 1, g.N);
        int i1 = Mod(iSeg,     g.N);
        int i2 = Mod(iSeg + 1, g.N);
        int i3 = Mod(iSeg + 2, g.N);

        // 2) local t in [0,1] within segment i1 -> i2
        float segStart = g.arcPos[i1];
        float segLen = Mathf.Max(0.0001f, g.edgeLen[i1]);
        float sUnwrapped = UnwrapNear(s, segStart, g.totalLen);
        float t = Mathf.Clamp01((sUnwrapped - segStart) / segLen);

        // 3) Crossfade weights (continuous)
        // w1 decreases, w2 increases smoothly
        float w2 = SmoothStep01(t);
        float w1 = 1f - w2;

        // Give shoulders a small continuous support to avoid harshness
        // (keeps it "local" but reduces staccato)
        float shoulder = 0.35f; // 0.25~0.45
        float w0 = shoulder * w1;
        float w3 = shoulder * w2;

        // Normalize weights so total energy stays stable
        float sum = w0 + w1 + w2 + w3;
        w0 /= sum; w1 /= sum; w2 /= sum; w3 /= sum;

        // 4) Evaluate gaussian at those 4 motor positions, then multiply by weights
        AccumulateMotor(g, i0, s, peak, inv2sig2, w0, perceptualThreshold01);
        AccumulateMotor(g, i1, s, peak, inv2sig2, w1, perceptualThreshold01);
        AccumulateMotor(g, i2, s, peak, inv2sig2, w2, perceptualThreshold01);
        AccumulateMotor(g, i3, s, peak, inv2sig2, w3, perceptualThreshold01);
    }

    private void AccumulateMotor(RingGeom g, int idx, float s, float peak, float inv2sig2, float weight, float perceptualThreshold01)
    {
        int motorId = g.motors[idx];
        if (motorId < 0 || motorId >= VestMotorCount) return;

        float p = g.arcPos[idx];
        float d = ShortestSignedDistanceOnRing(s, p, g.totalLen);
        float dAbs = Mathf.Abs(d);

        float val = peak * Mathf.Exp(-(dAbs * dAbs) * inv2sig2);
        float norm = Mathf.Clamp01(val / peak);

        // 1) gaussian tail cutoff
        if (norm < cutoff01) return;

        // 2) perceptual soft threshold (NO popping)
        // below thr => 0, above => remap to 0..1
        norm = SoftThreshold(norm, perceptualThreshold01);

        // 3) apply crossfade weight
        norm *= weight;

        val = norm * peak;
        if (val > _raw01[motorId]) _raw01[motorId] = val;
    }

    // ========= Geometry =========
    private void RebuildAll()
    {
        _topGeom = BuildRingGeom(topRing,    new EdgePair(15, 31), new EdgePair(28, 12));
        _midGeom = BuildRingGeom(midRing,    new EdgePair(11, 27), new EdgePair(24, 8));
        _botGeom = BuildRingGeom(bottomRing, new EdgePair(7,  23), new EdgePair(20, 4));
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

        for (int i = 0; i < g.N; i++) g.edgeLen[i] = 1.0f;

        ApplyEdgeLenMultiplierIfAdjacent(ref g, sideA, sideEdgeLengthMultiplier);
        ApplyEdgeLenMultiplierIfAdjacent(ref g, sideB, sideEdgeLengthMultiplier);

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
    }

    // ========= Side blend =========
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

        best = Mathf.Clamp01(best);
        return best * best * (3f - 2f * best); // smoothstep
    }

    // ========= Helpers =========
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
        if (r < 0) r += m;
        return r;
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

    // Find segment index i such that s is between arcPos[i] and arcPos[i]+edgeLen[i] (wrapped)
    private static int FindSegmentIndex(RingGeom g, float s)
    {
        // s를 0..totalLen
        s = Wrap(s, g.totalLen);

        for (int i = 0; i < g.N; i++)
        {
            float start = g.arcPos[i];
            float end = start + g.edgeLen[i];

            // wrap 고려: end가 totalLen 넘을 수 있음
            if (end <= g.totalLen)
            {
                if (s >= start && s < end) return i;
            }
            else
            {
                // 구간이 wrap되는 경우: [start,totalLen) U [0,end-totalLen)
                if (s >= start || s < (end - g.totalLen)) return i;
            }
        }
        return 0;
    }


    // unwrap s near refPos so (s - refPos) is continuous (no wrap jump)
    private static float UnwrapNear(float s, float refPos, float mod)
    {
        float half = mod * 0.5f;
        float delta = s - refPos;
        if (delta > half) s -= mod;
        else if (delta < -half) s += mod;
        return s;
    }

    private static float SmoothStep01(float t)
    {
        t = Mathf.Clamp01(t);
        return t * t * (3f - 2f * t);
    }

    private static float SoftThreshold(float x01, float thr01)
    {
    x01 = Mathf.Clamp01(x01);
    thr01 = Mathf.Clamp01(thr01);
    if (thr01 <= 0f) return x01;

    // thr 이하도 완전 0으로 자르지 않고, 부드럽게 눌러줌
    // (thr에서 기울기 연속)
    float y = (x01 - thr01) / (1f - thr01);
    y = Mathf.Clamp01(y);
    return y * y * (3f - 2f * y); // smoothstep
    }
}
