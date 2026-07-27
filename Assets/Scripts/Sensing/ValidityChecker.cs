using System;
using System.IO;
using UnityEngine;
using Robocon.Course;

namespace Robocon.Sensing
{
    /// <summary>
    /// 壁接触（Wallタグとの衝突）・コース外逸脱・転倒（Up軸とワールドUp軸のなす角が
    /// 閾値超過）を検知し、無効フラグを立てて記録・停止する。
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class ValidityChecker : MonoBehaviour
    {
        [SerializeField] private float tiltThresholdDeg = 15f;
        [SerializeField] private string wallTag = "Wall";
        [SerializeField] private string eventLogFileName = "invalid_events.csv";

        private Rigidbody rb;
        private Robot.RobotController controller;
        private StreamWriter eventWriter;

        public bool IsValid { get; private set; } = true;
        public string InvalidReason { get; private set; } = string.Empty;

        /// <summary>無効化された瞬間に理由付きで発火する。RunTimer等が購読する。</summary>
        public event Action<string> Invalidated;

        public void Init(Rigidbody body, Robot.RobotController robotController)
        {
            rb = body;
            controller = robotController;
        }

        private void Start()
        {
            string path = Path.Combine(Application.persistentDataPath, eventLogFileName);
            eventWriter = new StreamWriter(path, true);
        }

        private void FixedUpdate()
        {
            if (!IsValid) return;

            float tilt = Vector3.Angle(transform.up, Vector3.up);
            if (tilt > tiltThresholdDeg)
            {
                Invalidate($"転倒検知 (傾き={tilt:F1}deg)");
                return;
            }

            var course = CourseBuilder.Instance;
            if (course != null)
            {
                Vector2 xz = new Vector2(rb.position.x, rb.position.z);
                if (!course.IsInsideCourse(xz))
                {
                    Invalidate($"コース外逸脱 (x={xz.x:F3}, z={xz.y:F3})");
                }
            }
        }

        private void OnCollisionEnter(Collision collision)
        {
            if (!IsValid) return;
            if (collision.gameObject.CompareTag(wallTag))
            {
                Invalidate($"壁接触 ({collision.gameObject.name})");
            }
        }

        private void Invalidate(string reason)
        {
            IsValid = false;
            InvalidReason = reason;
            Debug.LogError($"[ValidityChecker] 無効判定: {reason} (t={Time.time:F2}s, pos={rb.position})");

            eventWriter?.WriteLine($"{Time.time:F4},{reason},{rb.position.x:F4},{rb.position.y:F4},{rb.position.z:F4}");
            eventWriter?.Flush();

            controller?.Stop();
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.isKinematic = true;

            Invalidated?.Invoke(reason);
        }

        private void OnDestroy()
        {
            eventWriter?.Flush();
            eventWriter?.Dispose();
        }
    }
}
