using UnityEngine;

namespace Robocon.Robot
{
    /// <summary>
    /// MotionProfilerが生成した目標速度・角速度の時系列に対し、PID＋フィードフォワードで
    /// 追従する低レベル制御ループ。直進・信地旋回・超信地旋回はいずれも
    /// 「目標COM前進速度v」と「目標ヨー角速度ω」の組として統一的に扱う。
    /// 信地旋回では支点半径rによりv=ω・rの拘束が成り立つ（v_L=v-ωr=0となり片輪固定と等価）。
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class TrajectoryFollower : MonoBehaviour
    {
        private enum Mode { Idle, Straight, Pivot, Spin }

        [Header("前進速度PIDゲイン")]
        [SerializeField] private float linearKp = 40f;
        [SerializeField] private float linearKi = 15f;
        [SerializeField] private float linearKd = 3f;
        [SerializeField] private float linearIntegralClamp = 2f;

        [Header("ヨー角速度PIDゲイン")]
        [SerializeField] private float angularKp = 10f;
        [SerializeField] private float angularKi = 4f;
        [SerializeField] private float angularKd = 0.4f;
        [SerializeField] private float angularIntegralClamp = 3f;

        [Header("直進区間の姿勢保持（外側ループ）")]
        [SerializeField] private float headingHoldKp = 6f;
        [SerializeField] private float maxHeadingCorrectionOmega = 1.5f;

        private Rigidbody rb;
        private WheelDrive wheelDrive;
        private float mass;
        private float yawInertia;

        private Mode mode = Mode.Idle;
        private TrapezoidalProfile profile;
        private float segmentTime;
        private float pivotRadius;
        private float targetHeadingDeg;

        private float vIntegral, vPrevError;
        private float wIntegral, wPrevError;

        public bool IsIdle => mode == Mode.Idle;
        public bool IsSegmentFinished { get; private set; } = true;

        public float LastForwardForce { get; private set; }
        public float LastYawTorque { get; private set; }

        public void Init(Rigidbody body, WheelDrive drive, float mass, float yawInertia)
        {
            rb = body;
            wheelDrive = drive;
            this.mass = mass;
            this.yawInertia = yawInertia;
        }

        public void FollowStraight(TrapezoidalProfile p)
        {
            profile = p;
            mode = Mode.Straight;
            segmentTime = 0f;
            pivotRadius = 0f;
            targetHeadingDeg = transform.eulerAngles.y;
            IsSegmentFinished = false;
            ResetPid();
        }

        public void FollowPivotTurn(TrapezoidalProfile p, float pivotRadiusMeters)
        {
            profile = p;
            mode = Mode.Pivot;
            segmentTime = 0f;
            pivotRadius = pivotRadiusMeters;
            IsSegmentFinished = false;
            ResetPid();
        }

        public void FollowSpinTurn(TrapezoidalProfile p)
        {
            profile = p;
            mode = Mode.Spin;
            segmentTime = 0f;
            pivotRadius = 0f;
            IsSegmentFinished = false;
            ResetPid();
        }

        public void Stop()
        {
            mode = Mode.Idle;
            profile = null;
            IsSegmentFinished = true;
            ResetPid();
        }

        private void ResetPid()
        {
            vIntegral = 0f; vPrevError = 0f;
            wIntegral = 0f; wPrevError = 0f;
        }

        private void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;
            float targetV = 0f, targetVAccel = 0f, targetOmega = 0f, targetOmegaAccel = 0f;

            if (mode != Mode.Idle)
            {
                MotionState s = profile.Sample(segmentTime);
                segmentTime += dt;
                IsSegmentFinished = s.Finished;

                switch (mode)
                {
                    case Mode.Straight:
                        targetV = s.Velocity;
                        targetVAccel = s.Acceleration;
                        float headingErrorRad = Mathf.DeltaAngle(transform.eulerAngles.y, targetHeadingDeg) * Mathf.Deg2Rad;
                        targetOmega = Mathf.Clamp(headingHoldKp * headingErrorRad, -maxHeadingCorrectionOmega, maxHeadingCorrectionOmega);
                        targetOmegaAccel = 0f;
                        break;
                    case Mode.Pivot:
                        targetOmega = s.Velocity;
                        targetOmegaAccel = s.Acceleration;
                        targetV = targetOmega * pivotRadius;
                        targetVAccel = targetOmegaAccel * pivotRadius;
                        break;
                    case Mode.Spin:
                        targetOmega = s.Velocity;
                        targetOmegaAccel = s.Acceleration;
                        targetV = 0f;
                        targetVAccel = 0f;
                        break;
                }
            }
            else
            {
                IsSegmentFinished = true;
            }

            float actualV = Vector3.Dot(rb.velocity, transform.forward);
            float actualOmega = rb.angularVelocity.y;

            float vError = targetV - actualV;
            vIntegral = Mathf.Clamp(vIntegral + vError * dt, -linearIntegralClamp, linearIntegralClamp);
            float vDeriv = dt > 1e-9f ? (vError - vPrevError) / dt : 0f;
            vPrevError = vError;
            float correctiveAccelV = linearKp * vError + linearKi * vIntegral + linearKd * vDeriv;
            float forwardForce = mass * (targetVAccel + correctiveAccelV);

            float wError = targetOmega - actualOmega;
            wIntegral = Mathf.Clamp(wIntegral + wError * dt, -angularIntegralClamp, angularIntegralClamp);
            float wDeriv = dt > 1e-9f ? (wError - wPrevError) / dt : 0f;
            wPrevError = wError;
            float correctiveAccelW = angularKp * wError + angularKi * wIntegral + angularKd * wDeriv;
            float yawTorque = yawInertia * (targetOmegaAccel + correctiveAccelW);

            LastForwardForce = forwardForce;
            LastYawTorque = yawTorque;
            wheelDrive.ApplyDrive(forwardForce, yawTorque);
        }
    }
}
