using System;
using UnityEngine;
using Bhaptics.SDK2;

public class YawGaussianRingPlayer : MonoBehaviour
{
    public enum SpeedMode { DegreesPerSecond, RadiansPerSecond, CyclesPerSecond }

    [Header("Rings (Yaw Order: front -> right -> back -> left)")]
    [SerializeField] private bool useTopRing = true;
    [SerializeField] private bool useMidRing = false;
    [SerializeField] private bool useBottomRing = false;

    // Top (existing)
    [SerializeField] private int[] topRing = new int[] { 12, 13, 14, 15, 31, 30, 29, 28 };

    // Mid
    [SerializeField] private int[] midRing = new int[] { 8, 9, 10, 11, 27, 26, 25, 24 };

    // Bottom
    [SerializeField] private int[] bottomRing = new int[] { 4, 5, 6, 7, 23, 22, 21, 20 };

    [Header("Ring Intensity Scales (when multiple rings enabled)")]
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

    [Header("Gaussian Parameters (separate for Front/Back vs Side)")]
    [Tooltip("Sigma used on normal segments (front/back and most of ring).")]
    [SerializeField] private float sigmaFrontBack = 0.85f;

    [Tooltip("Sigma used near side edges (problem edges). Larger => more overlap => smoother.")]
    [SerializeField] private float sigmaSide = 1.10f;

    [Tooltip("Arc window around side edge midpoint where sigmaSide applies (in arc units).")]
    [SerializeField] private float sideSigmaWindow = 0.9f;

    [Header("Side Edge Fix (edge length shortening)")]
    [Tooltip("Edge length multiplier for problematic edges (<1 means pass faster).")]
    [SerializeField] private float sideEdgeLengthMultiplier = 0.55f;

    // 너가 말한 문제 구간
    [SerializeField] private int sideEdgeA_from = 15;
    [SerializeField] private int sideEdgeA_to   = 28;
    [SerializeField] private int sideEdgeB_from = 31;
    [SerializeField] private int sideEdgeB_to   = 12;

    [Header("Smoothing")]
    [SerializeField] private float smoothingTau = 0.08f;
    [SerializeField] private bool useUnscaledTime = true;

    [Header("bHaptics Call")]
    [SerializeField] private int durationMillis = 50;

    private const int VestMotorCount = 40;

    private float[] _smoothed01;
    private float[] _raw01;

    private float _s;             // arc position
    private float[] _arcPos;      // arc pos per index (0..N-1) based on reference ring
    private float[] _edgeLen;     // edge lengths per index
    private int _N;               // ring length (must be 8 here)

    // We build arc from a "reference ring" because all rings have same length/order concept.
    // We'll use topRing as reference (length must match enabled rings).
    private int[] _refRing;

    private void Awake()
    {
        // pick a reference ring that is enabled (or top by default)
        _refRing = topRing;

        if (_refRing == null || _refRing.Length < 3)
        {
            Debug.LogError("Reference ring is invalid.");
            enabled = false;
            return;
        }

        // Validate lengths: enabled rings must have same length as ref
        _N = _refRing.Length;

        ValidateRingLength(useTopRing, topRing, "Top");
        ValidateRingLength(useMidRing, midRing, "Mid");
        ValidateRingLength(useBottomRing, bottomRing, "Bottom");

        BuildArcFromReferenceRing();

        _smoothed01 = new float[VestMotorCount];
        _raw01 = new float[VestMotorCount];
    }

    private void OnEnable()
    {
        _s = 0f;
        Array.Clear(_smoothed01, 0, _smoothed01.Length);
        Array.Clear(_raw01, 0, _raw01.Length);
    }

    private void OnDisable()
    {
        StopAllMotorsOnce();
    }

    private void Update()
    {
        float dt = useUnscaledTime ? Time.unscaledDeltaTime : Time.deltaTime;
        if (dt <= 0f) return;

        float totalLen = TotalLen();
        float cps = GetCyclesPerSecond();
        float speedArcPerSec = cps * totalLen;

        _s = Wrap(_s + speedArcPerSec * dt, totalLen);

        Array.Clear(_raw01, 0, _raw01.Length);

        // apply enabled rings
        if (useTopRing)    ApplyRing(topRing, topScale, totalLen);
        if (useMidRing)    ApplyRing(midRing, midScale, totalLen);
        if (useBottomRing) ApplyRing(bottomRing, bottomScale, totalLen);

        // smoothing
        float a = 1f - Mathf.Exp(-dt / Mathf.Max(0.0001f, smoothingTau));
        for (int i = 0; i < VestMotorCount; i++)
            _smoothed01[i] = Mathf.Lerp(_smoothed01[i], _raw01[i], a);

        // send
        int[] motorValues = new int[VestMotorCount];
        for (int i = 0; i < VestMotorCount; i++)
            motorValues[i] = Mathf.RoundToInt(Mathf.Clamp01(_smoothed01[i]) * 100f);

        // ✅ your SDK expects int position
        BhapticsLibrary.PlayMotors((int)PositionType.Vest, motorValues, durationMillis);
    }

    private void ApplyRing(int[] ringMotorIds, float ringScale, float totalLen)
    {
        // choose sigma based on whether center is near side edges
        float localSigma = IsNearSideEdge(_s, totalLen) ? sigmaSide : sigmaFrontBack;
        ApplyRingGaussian(ringMotorIds, totalLen, _s, localSigma, maxIntensity01, _raw01, ringScale);
    }

    // ----------------- Speed -----------------

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

    // ----------------- Gaussian on ring -----------------

    private void ApplyRingGaussian(int[] motorIds, float totalLen, float s, float sigmaLocal, float maxA, float[] out01, float scale)
    {
        float inv2sig2 = 1f / (2f * sigmaLocal * sigmaLocal);

        for (int idx = 0; idx < motorIds.Length; idx++)
        {
            int motorId = motorIds[idx];
            if (motorId < 0 || motorId >= VestMotorCount) continue;

            float p = _arcPos[idx]; // note: idx aligned by ring order
            float d = ShortestSignedDistanceOnRing(s, p, totalLen);

            float g = Mathf.Exp(-(d * d) * inv2sig2);
            float val = maxA * g * scale;

            if (val > out01[motorId]) out01[motorId] = val;
        }
    }

    // ----------------- Side edge detection -----------------

    private bool IsNearSideEdge(float s, float totalLen)
    {
        if (TryGetEdgeMidpoint(sideEdgeA_from, sideEdgeA_to, totalLen, out float midA))
            if (Mathf.Abs(ShortestSignedDistanceOnRing(s, midA, totalLen)) < sideSigmaWindow) return true;

        if (TryGetEdgeMidpoint(sideEdgeB_from, sideEdgeB_to, totalLen, out float midB))
            if (Mathf.Abs(ShortestSignedDistanceOnRing(s, midB, totalLen)) < sideSigmaWindow) return true;

        return false;
    }

    private bool TryGetEdgeMidpoint(int fromId, int toId, float totalLen, out float midpoint)
    {
        midpoint = 0f;

        for (int i = 0; i < _N; i++)
        {
            int a = _refRing[i];
            int b = _refRing[(i + 1) % _N];

            if ((a == fromId && b == toId) || (a == toId && b == fromId))
            {
                float aPos = _arcPos[i];
                float edge = _edgeLen[i];
                midpoint = Wrap(aPos + edge * 0.5f, totalLen);
                return true;
            }
        }
        return false;
    }

    // ----------------- Arc build (edge shortening) -----------------

    private void BuildArcFromReferenceRing()
    {
        _edgeLen = new float[_N];
        _arcPos = new float[_N];

        for (int i = 0; i < _N; i++) _edgeLen[i] = 1.0f;

        // shorten problematic edges (only if adjacent in reference ring)
        ApplyEdgeLengthMultiplierIfAdjacent(sideEdgeA_from, sideEdgeA_to, sideEdgeLengthMultiplier);
        ApplyEdgeLengthMultiplierIfAdjacent(sideEdgeB_from, sideEdgeB_to, sideEdgeLengthMultiplier);

        _arcPos[0] = 0f;
        for (int i = 1; i < _N; i++)
            _arcPos[i] = _arcPos[i - 1] + _edgeLen[i - 1];
    }

    private void ApplyEdgeLengthMultiplierIfAdjacent(int fromId, int toId, float mult)
    {
        for (int i = 0; i < _N; i++)
        {
            int a = _refRing[i];
            int b = _refRing[(i + 1) % _N];

            if ((a == fromId && b == toId) || (a == toId && b == fromId))
            {
                _edgeLen[i] *= mult;
                return;
            }
        }
    }

    private float TotalLen()
    {
        return _arcPos[^1] + _edgeLen[^1];
    }

    // ----------------- Validation -----------------

    private void ValidateRingLength(bool enabledRing, int[] ring, string name)
    {
        if (!enabledRing) return;
        if (ring == null || ring.Length != _N)
        {
            Debug.LogError($"{name} ring length must match reference ring length ({_N}).");
            enabled = false;
        }
    }

    // ----------------- Helpers -----------------

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

    private void StopAllMotorsOnce()
    {
        int[] zeros = new int[VestMotorCount];
        BhapticsLibrary.PlayMotors((int)PositionType.Vest, zeros, 100);
    }
}
