using UnityEngine;

namespace Robocon.Robot
{
    /// <summary>
    /// 左右仮想輪（ローカルX = ±halfTreadWidth、床面高さ）にAddForceAtPositionで
    /// 前後方向の力を加える低レベルアクチュエータ。並進力とヨーモーメントの
    /// 指令値を左右輪力へ分配するだけで、速度制御そのものはTrajectoryFollowerが担う。
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class WheelDrive : MonoBehaviour
    {
        private Rigidbody rb;
        private float halfTreadWidth;
        private float wheelDropFromCom;
        private bool initialized;

        public float HalfTreadWidth => halfTreadWidth;
        public float LastLeftForce { get; private set; }
        public float LastRightForce { get; private set; }

        /// <param name="halfTreadWidth">重心から左右輪までの水平距離 [m]</param>
        /// <param name="wheelDropFromCom">重心から輪の接地点までの鉛直距離 [m]（正値）</param>
        public void Init(Rigidbody body, float halfTreadWidth, float wheelDropFromCom)
        {
            rb = body;
            this.halfTreadWidth = halfTreadWidth;
            this.wheelDropFromCom = wheelDropFromCom;
            initialized = true;
        }

        public Vector3 LeftWheelPosition => WheelPosition(-1f);
        public Vector3 RightWheelPosition => WheelPosition(1f);

        private Vector3 WheelPosition(float side)
        {
            return rb.worldCenterOfMass
                   + transform.right * (side * halfTreadWidth)
                   - transform.up * wheelDropFromCom;
        }

        /// <summary>並進力[N]とヨーモーメント[N・m]を左右輪の前後方向力に分配して加える。</summary>
        public void ApplyDrive(float forwardForceNewtons, float yawTorqueNewtonMeters)
        {
            if (!initialized) return;

            float halfForce = forwardForceNewtons * 0.5f;
            float diffForce = halfTreadWidth > 1e-6f ? yawTorqueNewtonMeters / (2f * halfTreadWidth) : 0f;

            // 注意: r_left×F_left + r_right×F_right を計算すると、rightForceを増やすほど
            // 実際のトルクはUP軸の負方向に生じる（right×forward = -up のため）。
            // 指令したyawTorqueNewtonMetersの符号と実際に生じるトルクの符号を一致させるため、
            // 右輪にはdiffForceを「引く」側を割り当てる（直感とは逆に見えるが正しい）。
            float leftForce = halfForce + diffForce;
            float rightForce = halfForce - diffForce;

            LastLeftForce = leftForce;
            LastRightForce = rightForce;

            rb.AddForceAtPosition(transform.forward * leftForce, LeftWheelPosition, ForceMode.Force);
            rb.AddForceAtPosition(transform.forward * rightForce, RightWheelPosition, ForceMode.Force);
        }
    }
}
