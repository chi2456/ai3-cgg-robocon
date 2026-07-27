using System.Collections;
using UnityEngine;
using Robocon.Course;
using Robocon.Sensing;

namespace Robocon.Robot
{
    /// <summary>
    /// 前進/後退/停止/信地旋回/超信地旋回の高レベルAPIを持つロボット司令クラス。
    /// Awakeで剛体・コライダー・カメラ台座を自己構築するため、空のGameObjectに
    /// 本コンポーネントを1つ載せるだけで物理シミュレーション可能なロボットになる。
    ///
    /// ヨー角の符号規約: Unityの左手系Y回転に合わせ、正の角度＝ロボット正面から見て
    /// 右回り（真上から見て時計回り）。信地旋回は左輪固定＋右輪駆動が正の角度に対応する。
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(WheelDrive))]
    [RequireComponent(typeof(TrajectoryFollower))]
    [RequireComponent(typeof(AccelerationLogger))]
    [RequireComponent(typeof(ValidityChecker))]
    public class RobotController : MonoBehaviour
    {
        [Header("ロボット仕様（課題定義値）")]
        [SerializeField] private float robotMass = 10f;
        [SerializeField] private float bodyRadius = 0.15f;
        [SerializeField] private float comHeight = 0.5f;
        [SerializeField] private float cameraTopHeight = 1.0f;
        [SerializeField] private float halfTreadWidth = 0.12f;

        [Header("カメラ水平加速度の制約")]
        [SerializeField] private float cameraAccelLimit = 1.0f;
        [Range(0.5f, 1f)]
        [SerializeField] private float accelSafetyMargin = 0.9f;

        [Header("直進プロファイル既定値")]
        [SerializeField] private float defaultMaxSpeed = 0.5f;

        [Header("信地旋回プロファイル既定値")]
        [Range(0.1f, 0.9f)]
        [SerializeField] private float pivotCentripetalBudgetRatio = 0.6f;

        [Header("超信地旋回プロファイル既定値（制約対象外）")]
        [SerializeField] private float spinMaxAngularSpeed = 3.0f;
        [SerializeField] private float spinMaxAngularAccel = 6.0f;

        [Header("コース自動走行")]
        [SerializeField] private bool autoRunCourse = true;
        [SerializeField] private float autoRunStartDelay = 0.3f;

        private Rigidbody rb;
        private WheelDrive wheelDrive;
        private TrajectoryFollower follower;
        private AccelerationLogger accelLogger;
        private ValidityChecker validityChecker;

        public Transform CameraTop { get; private set; }
        public float DefaultMaxLinearAccel => cameraAccelLimit * accelSafetyMargin;
        public bool IsBusy => !follower.IsIdle && !follower.IsSegmentFinished;

        private void Awake()
        {
            rb = GetComponent<Rigidbody>();
            wheelDrive = GetComponent<WheelDrive>();
            follower = GetComponent<TrajectoryFollower>();
            accelLogger = GetComponent<AccelerationLogger>();
            validityChecker = GetComponent<ValidityChecker>();

            BuildRig();

            wheelDrive.Init(rb, halfTreadWidth, comHeight);
            float yawInertia = 0.5f * robotMass * bodyRadius * bodyRadius;
            follower.Init(rb, wheelDrive, robotMass, yawInertia);
            accelLogger.Init(rb, CameraTop);
            validityChecker.Init(rb, this);
        }

        private void BuildRig()
        {
            rb.mass = robotMass;
            rb.centerOfMass = Vector3.zero;
            rb.drag = 0f;
            rb.angularDrag = 0f;
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
            rb.interpolation = RigidbodyInterpolation.Interpolate;

            var collider = GetComponent<Collider>();
            if (collider == null)
            {
                var capsule = gameObject.AddComponent<CapsuleCollider>();
                capsule.direction = 1; // Y軸
                capsule.radius = bodyRadius;
                capsule.height = cameraTopHeight; // 床(-comHeight)からカメラ頂部(+cameraTopHeight-comHeight)まで
                capsule.center = new Vector3(0f, cameraTopHeight * 0.5f - comHeight, 0f);
                var mat = new PhysicMaterial("RobotHighGrip")
                {
                    dynamicFriction = 1f,
                    staticFriction = 1f,
                    frictionCombine = PhysicMaterialCombine.Maximum,
                    bounciness = 0f,
                    bounceCombine = PhysicMaterialCombine.Minimum,
                };
                capsule.material = mat;
            }

            var camMount = transform.Find("CameraTop");
            if (camMount == null)
            {
                var camGo = new GameObject("CameraTop");
                camGo.transform.SetParent(transform, false);
                camGo.transform.localPosition = new Vector3(0f, cameraTopHeight - comHeight, 0f);
                var cam = camGo.AddComponent<Camera>();
                cam.enabled = false; // 監視カメラの取り付け位置のみ利用（描画は任意）
                camMount = camGo.transform;
            }
            CameraTop = camMount;
        }

        private void Start()
        {
            var course = CourseBuilder.Instance;
            if (course != null)
            {
                transform.SetPositionAndRotation(course.StartPosition + Vector3.up * comHeight, course.StartRotation);
            }

            if (autoRunCourse && course != null)
            {
                StartCoroutine(DriveCourseRoutine(course));
            }
        }

        // ==== 高レベルAPI ====

        /// <summary>前進（distanceMeters &gt; 0）。台形速度プロファイルでカメラ水平加速度を上限ぎりぎりまで使う。</summary>
        public void MoveForward(float distanceMeters, float maxSpeed = -1f, float maxLinearAccel = -1f)
        {
            float speed = maxSpeed > 0f ? maxSpeed : defaultMaxSpeed;
            float accel = maxLinearAccel > 0f ? maxLinearAccel : DefaultMaxLinearAccel;
            var profile = MotionProfiler.CreateLinear(distanceMeters, speed, accel);
            follower.FollowStraight(profile);
        }

        /// <summary>後退（distanceMeters &gt; 0を渡す）。</summary>
        public void MoveBackward(float distanceMeters, float maxSpeed = -1f, float maxLinearAccel = -1f)
        {
            MoveForward(-Mathf.Abs(distanceMeters), maxSpeed, maxLinearAccel);
        }

        public void Stop()
        {
            follower.Stop();
        }

        /// <summary>信地旋回（片輪固定）。signedAngleDeg&gt;0で右回り（左輪固定・右輪駆動）。
        /// 支点半径＝輪トレッド半分で生じる向心加速度を含め、カメラ水平加速度が
        /// cameraAccelLimitを超えないよう角速度プロファイルを設計する。</summary>
        public void PivotTurn(float signedAngleDeg, float maxCombinedAccel = -1f)
        {
            float aMax = maxCombinedAccel > 0f ? maxCombinedAccel : DefaultMaxLinearAccel;
            var profile = MotionProfiler.CreatePivotTurn(
                signedAngleDeg * Mathf.Deg2Rad, wheelDrive.HalfTreadWidth, aMax, pivotCentripetalBudgetRatio);
            follower.FollowPivotTurn(profile, wheelDrive.HalfTreadWidth);
        }

        /// <summary>超信地旋回（その場旋回、重心固定）。カメラ水平加速度の制約対象外。</summary>
        public void SpinTurn(float signedAngleDeg, float maxAngularSpeed = -1f, float maxAngularAccel = -1f)
        {
            float w = maxAngularSpeed > 0f ? maxAngularSpeed : spinMaxAngularSpeed;
            float a = maxAngularAccel > 0f ? maxAngularAccel : spinMaxAngularAccel;
            var profile = MotionProfiler.CreateSpinTurn(signedAngleDeg * Mathf.Deg2Rad, w, a);
            follower.FollowSpinTurn(profile);
        }

        // ==== コース自動走行（直線区間=台形加減速、コーナー=超信地旋回） ====

        private IEnumerator DriveCourseRoutine(CourseBuilder course)
        {
            yield return new WaitForSeconds(autoRunStartDelay);

            var points = course.PathPoints;
            for (int i = 1; i < points.Count; i++)
            {
                if (validityChecker != null && !validityChecker.IsValid) yield break;

                Vector3 delta = points[i] - points[i - 1];
                if (delta.sqrMagnitude > 1e-6f)
                {
                    float desiredHeadingDeg = Quaternion.LookRotation(delta, Vector3.up).eulerAngles.y;
                    float turnAngle = Mathf.DeltaAngle(transform.eulerAngles.y, desiredHeadingDeg);
                    if (Mathf.Abs(turnAngle) > 0.5f)
                    {
                        SpinTurn(turnAngle);
                        yield return WaitUntilSegmentDone();
                    }
                }

                if (validityChecker != null && !validityChecker.IsValid) yield break;

                float distance = new Vector2(delta.x, delta.z).magnitude;
                MoveForward(distance);
                yield return WaitUntilSegmentDone();
            }

            Stop();
        }

        private IEnumerator WaitUntilSegmentDone()
        {
            yield return new WaitForFixedUpdate();
            while (IsBusy) yield return new WaitForFixedUpdate();
        }
    }
}
