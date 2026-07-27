using System.IO;
using UnityEngine;

namespace Robocon.Sensing
{
    /// <summary>
    /// 毎FixedUpdateで時刻・位置・速度・角速度・カメラ頭頂部の水平加速度・ジャークを記録しCSVへ出力する。
    /// カメラ頂部の加速度は剛体上の点の一般公式 a_P = a_com + α×r + ω×(ω×r) から求める
    /// （水平ヨー旋回のみならr∥ωとなりα×r=0,ω×(ω×r)=0で a_P=a_com に一致するが、
    /// 万一の転倒等でr∦ωになっても正しく評価できるよう一般式のまま計算する）。
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class AccelerationLogger : MonoBehaviour
    {
        [SerializeField] private string outputFileName = "accel_log.csv";
        [SerializeField] private float horizontalAccelWarningThreshold = 1.0f;
        [SerializeField] private int flushIntervalSteps = 100;

        private Rigidbody rb;
        private Transform cameraTop;
        private StreamWriter writer;
        private int stepsSinceFlush;

        private Vector3 prevVelocity;
        private Vector3 prevAngularVelocity;
        private Vector3 prevPointAccel;
        private bool hasPrevSample;

        public float LatestHorizontalAccel { get; private set; }
        public float LatestHorizontalJerk { get; private set; }

        public void Init(Rigidbody body, Transform cameraTopTransform)
        {
            rb = body;
            cameraTop = cameraTopTransform;
        }

        private void Start()
        {
            string path = Path.Combine(Application.persistentDataPath, outputFileName);
            writer = new StreamWriter(path, false);
            writer.WriteLine("time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,angvel_x,angvel_y,angvel_z,accel_h,jerk_h,accel_x,accel_z,over_limit");
            Debug.Log($"[AccelerationLogger] 出力先: {path}");

            prevVelocity = rb.velocity;
            prevAngularVelocity = rb.angularVelocity;
            prevPointAccel = Vector3.zero;
            hasPrevSample = false;
        }

        private void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;
            Vector3 velocity = rb.velocity;
            Vector3 angularVelocity = rb.angularVelocity;

            Vector3 comAccel = (velocity - prevVelocity) / dt;
            Vector3 angularAccel = (angularVelocity - prevAngularVelocity) / dt;

            Vector3 r = cameraTop.position - rb.worldCenterOfMass;
            Vector3 pointAccel = comAccel
                                  + Vector3.Cross(angularAccel, r)
                                  + Vector3.Cross(angularVelocity, Vector3.Cross(angularVelocity, r));

            float horizontalAccel = new Vector2(pointAccel.x, pointAccel.z).magnitude;
            Vector3 jerkVec = hasPrevSample ? (pointAccel - prevPointAccel) / dt : Vector3.zero;
            float horizontalJerk = new Vector2(jerkVec.x, jerkVec.z).magnitude;

            LatestHorizontalAccel = horizontalAccel;
            LatestHorizontalJerk = horizontalJerk;

            bool overLimit = horizontalAccel > horizontalAccelWarningThreshold;
            if (overLimit)
            {
                Debug.LogWarning($"[AccelerationLogger] カメラ水平加速度が制約超過: {horizontalAccel:F3} m/s^2 (t={Time.time:F2}s)");
            }

            writer.WriteLine(string.Join(",",
                Time.time.ToString("F4"),
                rb.position.x.ToString("F5"), rb.position.y.ToString("F5"), rb.position.z.ToString("F5"),
                velocity.x.ToString("F5"), velocity.y.ToString("F5"), velocity.z.ToString("F5"),
                angularVelocity.x.ToString("F5"), angularVelocity.y.ToString("F5"), angularVelocity.z.ToString("F5"),
                horizontalAccel.ToString("F5"), horizontalJerk.ToString("F5"),
                pointAccel.x.ToString("F5"), pointAccel.z.ToString("F5"),
                overLimit ? "1" : "0"));

            stepsSinceFlush++;
            if (stepsSinceFlush >= flushIntervalSteps)
            {
                writer.Flush();
                stepsSinceFlush = 0;
            }

            prevVelocity = velocity;
            prevAngularVelocity = angularVelocity;
            prevPointAccel = pointAccel;
            hasPrevSample = true;
        }

        private void OnDestroy()
        {
            writer?.Flush();
            writer?.Dispose();
        }

        private void OnApplicationQuit()
        {
            writer?.Flush();
        }
    }
}
