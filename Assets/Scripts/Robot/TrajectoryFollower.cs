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
        [SerializeField] private float linearKp = 15f;
        [SerializeField] private float linearKi = 5f;
        [SerializeField] private float linearKd = 0.5f;
        [SerializeField] private float linearIntegralClamp = 1f;
        [SerializeField] private float maxCorrectiveLinearAccel = 3f;

        [Header("ヨー角速度PIDゲイン")]
        [SerializeField] private float angularKp = 4f;
        [SerializeField] private float angularKi = 1f;
        [SerializeField] private float angularKd = 0.05f;
        [SerializeField] private float angularIntegralClamp = 1f;
        [SerializeField] private float maxCorrectiveAngularAccel = 10f;

        [Header("直進区間の経路追従（状態フィードバック）")]
        [Tooltip("状態ベクトル x=[横方向位置誤差, ヘディング誤差] に対する線形状態フィードバック " +
                 "ω = -(K_lat*x1 + K_head*x2) で目標ヨー角速度を決める。単純な1状態Pのヘディング保持より" +
                 "堅牢で、蓄積した横ドリフトも直接補正する。内側のヨーPIDより十分遅い帯域になるようゲインを選ぶ。")]
        [SerializeField] private float stateFeedbackKLateral = 1.2f;
        [SerializeField] private float stateFeedbackKHeading = 1.5f;
        [SerializeField] private float maxHeadingCorrectionOmega = 1.5f;

        [Header("転倒防止（ピッチ・ロールの自己復元、ヨー制御には影響しない）")]
        [SerializeField] private float levelingKp = 40f;
        [SerializeField] private float levelingKd = 8f;

        [Header("カメラ水平加速度の絶対上限（フィードフォワード+PID補正の合計をここでハードクランプする）")]
        [SerializeField] private float hardLinearAccelLimit = 0.98f;

        private Rigidbody rb;
        private WheelDrive wheelDrive;
        private float mass;
        private float yawInertia;

        private Mode mode = Mode.Idle;
        private TrapezoidalProfile profile;
        private float segmentTime;
        private float pivotRadius;
        private float targetHeadingDeg;
        private Vector3 segmentStartPos;
        private Vector3 segmentRightAxis;

        public float LastLateralError { get; private set; }
        public float LastHeadingErrorDeg { get; private set; }

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
            segmentStartPos = transform.position;
            segmentRightAxis = transform.right;
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

                        // 状態フィードバック: x = [横方向位置誤差 y(m), ヘディング誤差 θ(rad)]
                        // （yは基準直線からの符号付き垂直距離、θ=DeltaAngle(現在, 目標)=目標-現在）。
                        // 運動学 dy/dt=v・(現在ヘディング-目標ヘディング)=-v・θ, dθ/dt=-ω を
                        // u=ω=-K_lat・y+K_head・θ で閉ループすると
                        // y''+K_head・y'+v・K_lat・y=0 という安定な2次系になる（K_lat,K_head>0で減衰）。
                        float lateralError = Vector3.Dot(transform.position - segmentStartPos, segmentRightAxis);
                        float headingErrorRad = Mathf.DeltaAngle(transform.eulerAngles.y, targetHeadingDeg) * Mathf.Deg2Rad;
                        LastLateralError = lateralError;
                        LastHeadingErrorDeg = headingErrorRad * Mathf.Rad2Deg;

                        float stateFeedbackOmega = -stateFeedbackKLateral * lateralError + stateFeedbackKHeading * headingErrorRad;
                        targetOmega = Mathf.Clamp(stateFeedbackOmega, -maxHeadingCorrectionOmega, maxHeadingCorrectionOmega);
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
            float correctiveAccelV = Mathf.Clamp(
                linearKp * vError + linearKi * vIntegral + linearKd * vDeriv,
                -maxCorrectiveLinearAccel, maxCorrectiveLinearAccel);
            // 台形プロファイルは巡航→減速の境界で目標加速度が不連続にジャンプする（ジャーク不連続）。
            // PID追従精度だけに頼ると、この境界やその他の過渡応答で必須条件の1.00 m/s^2を
            // 一時的に超えうるため、水平加速度＝重心並進加速度そのもの（この設計の前提）を
            // 直接ハードクランプし、常に上限内に収まることを物理的に保証する。
            float totalLinearAccel = Mathf.Clamp(targetVAccel + correctiveAccelV, -hardLinearAccelLimit, hardLinearAccelLimit);
            float forwardForce = mass * totalLinearAccel;

            float wError = targetOmega - actualOmega;
            wIntegral = Mathf.Clamp(wIntegral + wError * dt, -angularIntegralClamp, angularIntegralClamp);
            float wDeriv = dt > 1e-9f ? (wError - wPrevError) / dt : 0f;
            wPrevError = wError;
            float correctiveAccelW = Mathf.Clamp(
                angularKp * wError + angularKi * wIntegral + angularKd * wDeriv,
                -maxCorrectiveAngularAccel, maxCorrectiveAngularAccel);
            float yawTorque = yawInertia * (targetOmegaAccel + correctiveAccelW);

            LastForwardForce = forwardForce;
            LastYawTorque = yawTorque;
            wheelDrive.ApplyDrive(forwardForce, yawTorque);

            ApplyTiltStabilization();
            ConstrainLateralSlip();
        }

        /// <summary>
        /// 実車の車輪は前後には転がるが横方向には滑りにくい（非ホロノミック拘束）。
        /// 床の摩擦を0にした副作用でこの拘束が失われ、旋回中のわずかな横成分が
        /// 一切補正されず蓄積してドリフトする問題が生じたため、横方向速度成分を
        /// 毎ステップ明示的にゼロへ落として車輪のグリップを表現する。
        /// </summary>
        private void ConstrainLateralSlip()
        {
            Vector3 lateral = Vector3.Dot(rb.velocity, transform.right) * transform.right;
            rb.velocity -= lateral;
        }

        /// <summary>ロボットは水平面内のヨー回転のみを行う設計（ピッチ・ロールしない）という
        /// 前提を成り立たせるため、車体の自己復元力（低いキャスター相当の機械的安定性）を
        /// ピッチ・ロール軸にのみ作用する復元トルクとしてモデル化する。ヨー角速度は一切変更しない。</summary>
        private void ApplyTiltStabilization()
        {
            Vector3 tiltAxis = Vector3.Cross(transform.up, Vector3.up);
            float tiltAngle = Vector3.Angle(transform.up, Vector3.up) * Mathf.Deg2Rad;

            if (tiltAngle > 1e-4f)
            {
                rb.AddTorque(tiltAxis.normalized * (tiltAngle * levelingKp), ForceMode.Force);
            }

            Vector3 yawAngularVelocity = Vector3.Dot(rb.angularVelocity, transform.up) * transform.up;
            Vector3 tiltAngularVelocity = rb.angularVelocity - yawAngularVelocity;
            rb.AddTorque(-tiltAngularVelocity * levelingKd, ForceMode.Force);
        }
    }
}
