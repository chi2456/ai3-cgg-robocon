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

        [Header("カメラ水平加速度の制約（目標値。実際の上限保証はTrajectoryFollowerのhardLinearAccelLimitで行う）")]
        [SerializeField] private float cameraAccelLimit = 1.0f;
        [Range(0.5f, 1f)]
        [SerializeField] private float accelSafetyMargin = 0.95f;

        [Header("直進プロファイル既定値（仕様上「走行速度の上限を設けない」ため、実質無制限の値にする。" +
                "加速度上限だけで律速されると台形プロファイルは自動的に三角形（最短時間）になる）")]
        [SerializeField] private float defaultMaxSpeed = 100f;

        [Header("信地旋回プロファイル既定値")]
        [Range(0.1f, 0.9f)]
        [SerializeField] private float pivotCentripetalBudgetRatio = 0.6f;

        [Header("超信地旋回プロファイル既定値（カメラ加速度制約の対象外。Rigidbody.maxAngularVelocity" +
                "の既定値7rad/sを超えない範囲でできるだけ速く回す）")]
        [SerializeField] private float spinMaxAngularSpeed = 6.0f;
        [SerializeField] private float spinMaxAngularAccel = 20.0f;

        [Header("コース自動走行")]
        [SerializeField] private bool autoRunCourse = true;
        [SerializeField] private float autoRunStartDelay = 0.3f;
        [SerializeField] private float segmentSettleSeconds = 0.2f;

        [Header("見た目（Collider/Rigidbodyには影響しない表示専用メッシュ）")]
        [SerializeField] private float bodyVisualHeight = 0.15f;
        [SerializeField] private float cameraMastRadius = 0.02f;
        [SerializeField] private Color bodyColor = new Color(0.2f, 0.45f, 0.9f);
        [SerializeField] private Color cameraColor = new Color(0.85f, 0.15f, 0.15f);

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

            // 車輪への力は重心と同じ高さ（鉛直オフセット0）で加える。床面（重心から0.5m下）で
            // 加えると、左右輪の力が同方向のためピッチ方向の転倒トルクが常に発生してしまい、
            // 「水平面内のヨー回転のみ」という前提（＝カメラ頂部の水平加速度=重心並進加速度）が
            // 崩れてしまうため。トレッド幅方向（ローカルX）のオフセットのみ残すのでヨーモーメントは従来通り生じる。
            wheelDrive.Init(rb, halfTreadWidth, 0f);
            // フィードフォワード/PID補正のトルク換算に使う慣性モーメントは、手計算の推定値
            // （中実円柱近似 0.5*m*r^2）ではなく、Unityが実際のコライダー形状から計算した
            // rb.inertiaTensorを使う。推定値とのズレがあると角速度制御が過大/過小になり、
            // 最悪Rigidbody.maxAngularVelocity（既定7rad/s）に張り付いて回転が止まらなくなる。
            float yawInertia = rb.inertiaTensor.y;
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
                // 駆動力は車輪位置へのAddForceAtPositionで与えるため、胴体コライダーと床の
                // 摩擦は不要（あるとPID補正力の上限を静止摩擦が上回り走行不能になる）。
                var mat = new PhysicMaterial("RobotLowFriction")
                {
                    dynamicFriction = 0f,
                    staticFriction = 0f,
                    frictionCombine = PhysicMaterialCombine.Minimum,
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

            BuildVisuals();
        }

        /// <summary>Game/Sceneビューでの視認用に、半径0.15mの胴体円柱とカメラ台座までの
        /// 細い円柱・カメラ頭部を追加する。いずれもColliderは持たせない（物理形状は変更しない）。</summary>
        private void BuildVisuals()
        {
            float floorLocalY = -comHeight;
            float topLocalY = cameraTopHeight - comHeight;

            if (transform.Find("BodyVisual") == null)
            {
                float bodyCenterY = floorLocalY + bodyVisualHeight * 0.5f;
                CreateVisualCylinder("BodyVisual", transform, new Vector3(0f, bodyCenterY, 0f), bodyRadius, bodyVisualHeight, bodyColor);
            }

            if (transform.Find("CameraMast") == null)
            {
                float mastBottomY = floorLocalY + bodyVisualHeight;
                float mastHeight = Mathf.Max(topLocalY - mastBottomY, 0.01f);
                float mastCenterY = mastBottomY + mastHeight * 0.5f;
                CreateVisualCylinder("CameraMast", transform, new Vector3(0f, mastCenterY, 0f), cameraMastRadius, mastHeight, cameraColor);
            }

            if (CameraTop.Find("CameraHead") == null)
            {
                var head = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                head.name = "CameraHead";
                var headCollider = head.GetComponent<Collider>();
                headCollider.enabled = false;
                Destroy(headCollider);
                head.transform.SetParent(CameraTop, false);
                head.transform.localPosition = Vector3.zero;
                head.transform.localScale = Vector3.one * (cameraMastRadius * 4f);
                head.GetComponent<Renderer>().sharedMaterial = CreateUnlitColorMaterial(cameraColor);
            }
        }

        private void CreateVisualCylinder(string name, Transform parent, Vector3 localPosition, float radius, float height, Color color)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            go.name = name;
            var col = go.GetComponent<Collider>();
            col.enabled = false;
            Destroy(col);
            go.transform.SetParent(parent, false);
            go.transform.localPosition = localPosition;
            // 既定のCylinderメッシュは半径0.5・高さ2なので、目的の半径・高さに合わせてスケールする。
            go.transform.localScale = new Vector3(radius * 2f, height * 0.5f, radius * 2f);
            go.GetComponent<Renderer>().sharedMaterial = CreateUnlitColorMaterial(color);
        }

        private static Material CreateUnlitColorMaterial(Color color)
        {
            var shader = Shader.Find("Standard");
            var mat = new Material(shader) { color = color };
            return mat;
        }

        private void Start()
        {
            var course = CourseBuilder.Instance;
            if (course != null)
            {
                Vector3 spawnPos = course.StartPosition + Vector3.up * comHeight;
                transform.SetPositionAndRotation(spawnPos, course.StartRotation);
                // Rigidbodyの物理状態を即時同期する（Physics.autoSyncTransformsの設定に依存しないため）。
                rb.position = spawnPos;
                rb.rotation = course.StartRotation;
                Physics.SyncTransforms();
                Debug.Log($"[RobotController] Spawned at {transform.position}, heading={transform.eulerAngles.y:F1}deg");
            }
            else
            {
                Debug.LogWarning("[RobotController] CourseBuilder.Instance is null in Start(); robot stays at its scene-placed transform.");
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
            // 摩擦ゼロのため残留速度・角速度は自然には減衰しない。目標値0のままPIDに
            // 収束させる時間を与えてから次のコマンドへ進み、旋回中のドリフトを防ぐ。
            yield return new WaitForSeconds(segmentSettleSeconds);
        }
    }
}
