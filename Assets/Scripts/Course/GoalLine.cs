using System;
using UnityEngine;
using Robocon.Robot;

namespace Robocon.Course
{
    /// <summary>
    /// ゴールラインのトリガー。課題仕様「ゴールのラインを超えて完全に離れた時点で計測終了」
    /// に合わせ、進入(OnTriggerEnter)ではなく完全通過(OnTriggerExit)を検知して通知する。
    /// </summary>
    [RequireComponent(typeof(Collider))]
    public class GoalLine : MonoBehaviour
    {
        public event Action<RobotController> RobotCleared;

        private bool armed = true;
        private bool hasEntered;

        private void Reset()
        {
            var col = GetComponent<Collider>();
            col.isTrigger = true;
        }

        /// <summary>次の1回の到達を再び検知できるようにする（新しい走行の開始時に呼ぶ）。</summary>
        public void Arm()
        {
            armed = true;
            hasEntered = false;
        }

        private void OnTriggerEnter(Collider other)
        {
            if (!armed) return;
            if (other.GetComponentInParent<RobotController>() == null) return;
            hasEntered = true;
        }

        private void OnTriggerExit(Collider other)
        {
            if (!armed || !hasEntered) return;
            var robot = other.GetComponentInParent<RobotController>();
            if (robot == null) return;

            armed = false;
            RobotCleared?.Invoke(robot);
        }
    }
}
