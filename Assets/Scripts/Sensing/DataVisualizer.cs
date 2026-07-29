using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Robocon.Course;

namespace Robocon.Sensing
{
    /// <summary>
    /// カメラ頂部水平加速度の時系列グラフ（1.00 m/s^2の閾値線つき）と、
    /// 速度・角速度・ジャーク・経過時間などの数値をUnity実行中にオーバーレイ表示する
    /// データ可視化コンポーネント。実行時にCanvas/UI一式を自己構築するため、
    /// 空のGameObjectに乗せるだけで動作する。
    /// </summary>
    public class DataVisualizer : MonoBehaviour
    {
        [SerializeField] private float graphTimeWindowSeconds = 15f;
        [SerializeField] private float graphMaxAccel = 1.5f;
        [SerializeField] private float sampleInterval = 0.05f;
        [SerializeField] private int graphWidth = 480;
        [SerializeField] private int graphHeight = 170;
        [SerializeField] private float accelThreshold = 1.0f;

        private AccelerationLogger accelLogger;
        private Rigidbody robotRb;
        private RunTimer runTimer;

        private Texture2D graphTexture;
        private Text readoutText;
        private readonly List<float> accelSamples = new List<float>();
        private float lastSampleTime = -999f;

        private float maxAccelSeen;
        private float maxSpeedSeen;
        private float maxAngularSpeedSeen;
        private float maxJerkSeen;

        private void Awake()
        {
            BuildUi();
        }

        private void Start()
        {
            var robotGo = GameObject.Find("Robot");
            if (robotGo != null)
            {
                accelLogger = robotGo.GetComponent<AccelerationLogger>();
                robotRb = robotGo.GetComponent<Rigidbody>();
            }
            runTimer = FindFirstObjectByType<RunTimer>();

            int maxSamples = Mathf.CeilToInt(graphTimeWindowSeconds / sampleInterval);
            accelSamples.Capacity = maxSamples + 1;
        }

        private void Update()
        {
            if (accelLogger == null) return;
            if (Time.time - lastSampleTime < sampleInterval) return;
            lastSampleTime = Time.time;

            float accel = accelLogger.LatestHorizontalAccel;
            float jerk = accelLogger.LatestHorizontalJerk;
            float speed = robotRb != null ? robotRb.velocity.magnitude : 0f;
            float angularSpeed = robotRb != null ? Mathf.Abs(robotRb.angularVelocity.y) : 0f;

            maxAccelSeen = Mathf.Max(maxAccelSeen, accel);
            maxSpeedSeen = Mathf.Max(maxSpeedSeen, speed);
            maxAngularSpeedSeen = Mathf.Max(maxAngularSpeedSeen, angularSpeed);
            maxJerkSeen = Mathf.Max(maxJerkSeen, jerk);

            int maxSamples = Mathf.CeilToInt(graphTimeWindowSeconds / sampleInterval);
            accelSamples.Add(accel);
            while (accelSamples.Count > maxSamples) accelSamples.RemoveAt(0);

            RedrawGraph();
            UpdateReadout(accel, jerk, speed, angularSpeed);
        }

        private void BuildUi()
        {
            var canvasGo = new GameObject("DataVisualizerCanvas");
            canvasGo.transform.SetParent(transform, false);
            var canvas = canvasGo.AddComponent<Canvas>();
            // ScreenSpaceOverlayはカメラのレンダーパイプラインの外で画面に直接合成されるため、
            // RenderTexture経由のスクリーンショットや録画キャプチャに映らないことがある。
            // ScreenSpaceCameraにしてメインカメラの描画に含めることで確実に記録されるようにする。
            // ScreenSpaceOverlay: どのカメラにも依存せず画面へ直接合成されるため、
            // Sceneビューでの視点操作やGameビュー内カメラの向き・位置に関係なく、
            // 常に画面左上に固定表示される（Gameビューでのみ描画される点に注意）。
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            var scaler = canvasGo.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1280, 720);
            canvasGo.AddComponent<GraphicRaycaster>();

            var panelGo = new GameObject("Panel");
            panelGo.transform.SetParent(canvasGo.transform, false);
            var panelImg = panelGo.AddComponent<Image>();
            panelImg.color = new Color(0f, 0f, 0f, 0.55f);
            var panelRt = panelGo.GetComponent<RectTransform>();
            panelRt.anchorMin = new Vector2(0f, 1f);
            panelRt.anchorMax = new Vector2(0f, 1f);
            panelRt.pivot = new Vector2(0f, 1f);
            panelRt.anchoredPosition = new Vector2(10f, -10f);
            panelRt.sizeDelta = new Vector2(graphWidth + 20f, graphHeight + 240f);

            var graphGo = new GameObject("Graph");
            graphGo.transform.SetParent(panelGo.transform, false);
            var rawImage = graphGo.AddComponent<RawImage>();
            var graphRt = graphGo.GetComponent<RectTransform>();
            graphRt.anchorMin = new Vector2(0f, 1f);
            graphRt.anchorMax = new Vector2(0f, 1f);
            graphRt.pivot = new Vector2(0f, 1f);
            graphRt.anchoredPosition = new Vector2(10f, -10f);
            graphRt.sizeDelta = new Vector2(graphWidth, graphHeight);

            graphTexture = new Texture2D(graphWidth, graphHeight, TextureFormat.RGBA32, false)
            {
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Clamp
            };
            rawImage.texture = graphTexture;

            var labelGo = new GameObject("GraphLabel");
            labelGo.transform.SetParent(panelGo.transform, false);
            var label = labelGo.AddComponent<Text>();
            label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            label.fontSize = 18;
            label.fontStyle = FontStyle.Bold;
            label.color = new Color(1f, 0.45f, 0.45f);
            label.text = $"水平加速度 [m/s^2]  赤線=上限{accelThreshold:F2}";
            label.alignment = TextAnchor.UpperLeft;
            var labelRt = labelGo.GetComponent<RectTransform>();
            labelRt.anchorMin = new Vector2(0f, 1f);
            labelRt.anchorMax = new Vector2(0f, 1f);
            labelRt.pivot = new Vector2(0f, 1f);
            labelRt.anchoredPosition = new Vector2(10f, -6f);
            labelRt.sizeDelta = new Vector2(graphWidth, 26f);

            var textGo = new GameObject("Readout");
            textGo.transform.SetParent(panelGo.transform, false);
            readoutText = textGo.AddComponent<Text>();
            readoutText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            readoutText.fontSize = 24;
            readoutText.color = Color.white;
            readoutText.alignment = TextAnchor.UpperLeft;
            readoutText.lineSpacing = 1.15f;
            readoutText.horizontalOverflow = HorizontalWrapMode.Overflow;
            readoutText.verticalOverflow = VerticalWrapMode.Overflow;
            var textRt = textGo.GetComponent<RectTransform>();
            textRt.anchorMin = new Vector2(0f, 1f);
            textRt.anchorMax = new Vector2(0f, 1f);
            textRt.pivot = new Vector2(0f, 1f);
            textRt.anchoredPosition = new Vector2(10f, -(graphHeight + 40f));
            textRt.sizeDelta = new Vector2(graphWidth, 220f);
        }

        private void RedrawGraph()
        {
            var pixels = new Color32[graphWidth * graphHeight];
            Color32 bg = new Color32(15, 15, 15, 235);
            for (int i = 0; i < pixels.Length; i++) pixels[i] = bg;

            int thresholdY = Mathf.Clamp(Mathf.RoundToInt(accelThreshold / graphMaxAccel * graphHeight), 0, graphHeight - 1);
            Color32 thresholdColor = new Color32(230, 60, 60, 255);
            for (int x = 0; x < graphWidth; x++) pixels[thresholdY * graphWidth + x] = thresholdColor;

            int n = accelSamples.Count;
            if (n >= 2)
            {
                Color32 lineColor = new Color32(90, 200, 255, 255);
                int maxSamples = Mathf.CeilToInt(graphTimeWindowSeconds / sampleInterval);
                for (int i = 1; i < n; i++)
                {
                    int x0 = Mathf.RoundToInt((float)(i - 1) / (maxSamples - 1) * (graphWidth - 1));
                    int x1 = Mathf.RoundToInt((float)i / (maxSamples - 1) * (graphWidth - 1));
                    int y0 = Mathf.Clamp(Mathf.RoundToInt(accelSamples[i - 1] / graphMaxAccel * graphHeight), 0, graphHeight - 1);
                    int y1 = Mathf.Clamp(Mathf.RoundToInt(accelSamples[i] / graphMaxAccel * graphHeight), 0, graphHeight - 1);
                    DrawLine(pixels, x0, y0, x1, y1, lineColor);
                }
            }

            graphTexture.SetPixels32(pixels);
            graphTexture.Apply(false);
        }

        private void DrawLine(Color32[] pixels, int x0, int y0, int x1, int y1, Color32 color)
        {
            int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
            int dy = -Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
            int err = dx + dy;
            while (true)
            {
                SetPixelSafe(pixels, x0, y0, color);
                if (x0 == x1 && y0 == y1) break;
                int e2 = 2 * err;
                if (e2 >= dy) { err += dy; x0 += sx; }
                if (e2 <= dx) { err += dx; y0 += sy; }
            }
        }

        private void SetPixelSafe(Color32[] pixels, int x, int y, Color32 color)
        {
            if (x < 0 || x >= graphWidth || y < 0 || y >= graphHeight) return;
            pixels[y * graphWidth + x] = color;
        }

        private void UpdateReadout(float accel, float jerk, float speed, float angularSpeed)
        {
            string state = runTimer != null ? runTimer.State.ToString() : "N/A";
            float elapsed = runTimer != null ? (runTimer.ResultSeconds >= 0f ? runTimer.ResultSeconds : runTimer.ElapsedSeconds) : Time.time;

            readoutText.text =
                $"走行状態: {state}\n" +
                $"経過時間: {elapsed:F2} s\n" +
                $"速度: {speed:F3} m/s (max {maxSpeedSeen:F3})\n" +
                $"水平加速度: {accel:F3} m/s^2 (max {maxAccelSeen:F3})\n" +
                $"ジャーク: {jerk:F2} m/s^3 (max {maxJerkSeen:F2})\n" +
                $"角速度: {angularSpeed:F3} rad/s (max {maxAngularSpeedSeen:F3})";
        }
    }
}
