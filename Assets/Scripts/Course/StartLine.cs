using System;
using UnityEngine;
using Robocon.Robot;

namespace Robocon.Course
{
    /// <summary>スタートラインのトリガー。ロボット本体（RobotController保持体）の進入を通知する。</summary>
    [RequireComponent(typeof(Collider))]
    public class StartLine : MonoBehaviour
    {
        public event Action<RobotController> RobotEntered;

        private bool armed = true;

        private void Reset()
        {
            var col = GetComponent<Collider>();
            col.isTrigger = true;
        }

        /// <summary>次の1回の進入を再び検知できるようにする（新しい走行の開始時に呼ぶ）。</summary>
        public void Arm() => armed = true;

        private void OnTriggerEnter(Collider other)
        {
            if (!armed) return;
            var robot = other.GetComponentInParent<RobotController>();
            if (robot == null) return;

            armed = false;
            RobotEntered?.Invoke(robot);
        }
    }
}
