using UnityEngine;

namespace Robocon.Robot
{
    /// <summary>時刻tにおける目標位置・速度・加速度（符号付き、直進なら[m][m/s][m/s^2]、
    /// 旋回なら[rad][rad/s][rad/s^2]）。</summary>
    public struct MotionState
    {
        public float Position;
        public float Velocity;
        public float Acceleration;
        public bool Finished;
    }

    /// <summary>単一区間の台形速度プロファイル。距離が短く最高速度に到達しない場合は
    /// 自動的に三角形プロファイルへ縮退する。</summary>
    public class TrapezoidalProfile
    {
        public readonly float Duration;
        public readonly float TotalDistance;
        public readonly float CruiseVelocity;
        public readonly float Accel;

        private readonly float sign;
        private readonly float tAccel;
        private readonly float tCruise;
        private readonly float distAccel;
        private readonly float absDistance;

        private TrapezoidalProfile(float absDistance, float maxSpeed, float maxAccel, float sign)
        {
            this.sign = sign;
            this.absDistance = absDistance;
            Accel = Mathf.Max(maxAccel, 1e-6f);
            maxSpeed = Mathf.Max(maxSpeed, 1e-6f);

            float fullAccelDist = maxSpeed * maxSpeed / (2f * Accel);
            if (fullAccelDist * 2f >= absDistance)
            {
                CruiseVelocity = Mathf.Sqrt(absDistance * Accel);
                tAccel = CruiseVelocity / Accel;
                tCruise = 0f;
                distAccel = absDistance * 0.5f;
            }
            else
            {
                CruiseVelocity = maxSpeed;
                tAccel = CruiseVelocity / Accel;
                distAccel = fullAccelDist;
                float cruiseDist = absDistance - 2f * fullAccelDist;
                tCruise = cruiseDist / CruiseVelocity;
            }
            TotalDistance = absDistance * sign;
            Duration = 2f * tAccel + tCruise;
        }

        public static TrapezoidalProfile Create(float distance, float maxSpeed, float maxAccel)
        {
            float sign = distance >= 0f ? 1f : -1f;
            return new TrapezoidalProfile(Mathf.Abs(distance), Mathf.Abs(maxSpeed), Mathf.Abs(maxAccel), sign);
        }

        public MotionState Sample(float t)
        {
            if (Duration <= 1e-9f || t >= Duration)
            {
                return new MotionState { Position = TotalDistance, Velocity = 0f, Acceleration = 0f, Finished = true };
            }
            if (t < 0f) t = 0f;

            float pos, vel, acc;
            if (t < tAccel)
            {
                pos = 0.5f * Accel * t * t;
                vel = Accel * t;
                acc = Accel;
            }
            else if (t < tAccel + tCruise)
            {
                float dt = t - tAccel;
                pos = distAccel + CruiseVelocity * dt;
                vel = CruiseVelocity;
                acc = 0f;
            }
            else
            {
                float remaining = Duration - t;
                pos = absDistance - 0.5f * Accel * remaining * remaining;
                vel = Accel * remaining;
                acc = -Accel;
            }

            return new MotionState { Position = pos * sign, Velocity = vel * sign, Acceleration = acc * sign, Finished = false };
        }
    }

    /// <summary>台形速度プロファイルを生成する軌道生成ユーティリティ。</summary>
    public static class MotionProfiler
    {
        /// <summary>直進区間用。カメラ頂部水平加速度＝重心並進加速度なので、maxLinearAccelが
        /// そのままカメラ水平加速度の上限になる。</summary>
        public static TrapezoidalProfile CreateLinear(float distance, float maxSpeed, float maxLinearAccel)
        {
            return TrapezoidalProfile.Create(distance, maxSpeed, maxLinearAccel);
        }

        /// <summary>
        /// 信地旋回（支点半径pivotRadius、片輪固定でCOMが弧を描く）用の角速度台形プロファイル。
        /// COM上の合成加速度は接線成分pivotRadius*角加速度と向心成分pivotRadius*角速度^2の
        /// ベクトル和（直交）になるため、|a| = pivotRadius*sqrt(角加速度^2 + 角速度^4)。
        /// 巡航角速度到達時は接線成分が0になる前提で、centripetalBudgetRatioの割合を
        /// 巡航時の向心加速度に、残りをランプ中の接線加速度上限に配分することで、
        /// ランプ全区間（0..角速度上限）でmaxCombinedAccelを超えないことを保証する
        /// （最短時間ではないが安全側に単純化した設計）。
        /// </summary>
        public static TrapezoidalProfile CreatePivotTurn(float angleRad, float pivotRadius, float maxCombinedAccel, float centripetalBudgetRatio = 0.6f)
        {
            pivotRadius = Mathf.Max(pivotRadius, 1e-4f);
            float omegaMax = Mathf.Sqrt(centripetalBudgetRatio * maxCombinedAccel / pivotRadius);
            float angularAccel = maxCombinedAccel * Mathf.Sqrt(1f - centripetalBudgetRatio * centripetalBudgetRatio) / pivotRadius;
            return TrapezoidalProfile.Create(angleRad, omegaMax, angularAccel);
        }

        /// <summary>超信地旋回（重心固定）はカメラ水平加速度の制約対象外のため、
        /// 専用の角速度・角加速度上限をそのまま用いる。</summary>
        public static TrapezoidalProfile CreateSpinTurn(float angleRad, float maxAngularSpeed, float maxAngularAccel)
        {
            return TrapezoidalProfile.Create(angleRad, maxAngularSpeed, maxAngularAccel);
        }
    }
}
