using UnityEngine;
using Robocon.Robot;
using Robocon.Sensing;

namespace Robocon.Course
{
    /// <summary>StartLine/GoalLineのイベントを購読し、走行タイムを計測・記録する。</summary>
    public class RunTimer : MonoBehaviour
    {
        public enum RunState { Idle, Running, Finished, Invalid }

        [SerializeField] private StartLine startLine;
        [SerializeField] private GoalLine goalLine;

        public RunState State { get; private set; } = RunState.Idle;
        public float ElapsedSeconds { get; private set; }
        public float ResultSeconds { get; private set; } = -1f;

        private float startTime;
        private ValidityChecker activeValidityChecker;

        private void Start()
        {
            if (startLine == null) startLine = FindFirstObjectByType<StartLine>();
            if (goalLine == null) goalLine = FindFirstObjectByType<GoalLine>();

            if (startLine != null) startLine.RobotEntered += OnRobotStarted;
            if (goalLine != null) goalLine.RobotEntered += OnRobotGoaled;
        }

        private void OnDestroy()
        {
            if (startLine != null) startLine.RobotEntered -= OnRobotStarted;
            if (goalLine != null) goalLine.RobotEntered -= OnRobotGoaled;
            if (activeValidityChecker != null) activeValidityChecker.Invalidated -= OnRunInvalidated;
        }

        private void Update()
        {
            if (State == RunState.Running)
            {
                ElapsedSeconds = Time.time - startTime;
            }
        }

        private void OnRobotStarted(RobotController robot)
        {
            State = RunState.Running;
            startTime = Time.time;
            ElapsedSeconds = 0f;
            ResultSeconds = -1f;

            activeValidityChecker = robot.GetComponent<ValidityChecker>();
            if (activeValidityChecker != null) activeValidityChecker.Invalidated += OnRunInvalidated;

            Debug.Log($"[RunTimer] 計測開始 t={startTime:F2}s");
        }

        private void OnRobotGoaled(RobotController robot)
        {
            if (State != RunState.Running) return;

            ResultSeconds = Time.time - startTime;
            State = RunState.Finished;
            Debug.Log($"[RunTimer] ゴール！ タイム={ResultSeconds:F3}s");

            if (activeValidityChecker != null)
            {
                activeValidityChecker.Invalidated -= OnRunInvalidated;
                activeValidityChecker = null;
            }
        }

        private void OnRunInvalidated(string reason)
        {
            if (State != RunState.Running) return;

            State = RunState.Invalid;
            Debug.LogWarning($"[RunTimer] 走行無効: {reason} (経過={ElapsedSeconds:F3}s)");
        }

        /// <summary>新しい走行を開始できるようStart/Goalラインを再アームする。</summary>
        public void ResetRun()
        {
            State = RunState.Idle;
            ElapsedSeconds = 0f;
            ResultSeconds = -1f;
            if (activeValidityChecker != null)
            {
                activeValidityChecker.Invalidated -= OnRunInvalidated;
                activeValidityChecker = null;
            }
            startLine?.Arm();
            goalLine?.Arm();
        }
    }
}
