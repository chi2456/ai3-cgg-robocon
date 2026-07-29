# ai3-cgg-robocon

ロボット掃除機＋見守りカメラのシミュレーションロボコン課題。Unityの物理演算（Rigidbody・Continuous
Collision Detection）を用い、加速度・走行ルートを制御してコース走破時間を計測する。

## 基本仕様

| 項目 | 値 |
|---|---|
| Unityバージョン | 2022.3.62f3 |
| Fixed Timestep | 0.01 s（シミュレーション中一定） |
| 全質量 | 10 kg |
| 重心高さ | 床面から 0.50 m |
| カメラ頭頂部高さ | 床面から 1.00 m |
| 本体直径 | 0.30 m |
| カメラ水平合成加速度の上限 | 1.00 m/s²（フィードフォワード+PID補正の合計を0.98 m/s²でハードクランプして保証） |
| Collision Detection | Continuous |

## フォルダ構成

```
Assets/Scripts/
├── Course/
│   ├── CourseBuilder.cs   … 矩形データからコース（壁・床・スタート/ゴール・中心線経路）を実行時に自動生成
│   ├── StartLine.cs       … スタートライン（触れた時点で計測開始）
│   ├── GoalLine.cs        … ゴールライン（完全に通過し終えた時点で計測終了）
│   └── RunTimer.cs        … 走行タイムの計測・記録
├── Robot/
│   ├── WheelDrive.cs         … 左右仮想輪へAddForceAtPositionで力を加える低レベルアクチュエータ
│   ├── MotionProfiler.cs     … 台形（区間長によっては三角形に縮退する最短時間）速度プロファイル生成
│   ├── TrajectoryFollower.cs … 状態フィードバック＋PID＋フィードフォワードによる追従制御、加速度ハードクランプ
│   └── RobotController.cs    … 前進/後退/停止/信地旋回/超信地旋回の高レベルAPI、コース自動走行
└── Sensing/
    ├── AccelerationLogger.cs … カメラ頂部の水平加速度・ジャークを毎ステップCSVに記録
    ├── ValidityChecker.cs    … 壁接触・コース外・転倒を検知し無効化
    └── DataVisualizer.cs     … 加速度グラフ・各種数値をUnity実行中にリアルタイム表示

Submission/
├── simulation_stats_report.pdf … 走破時間・最大/平均加速度・最大速度・最大角速度・最大ジャーク等の計測結果
└── appeal_points.pdf           … 設計上のアピールポイント（状態フィードバック、最短時間制御、デバッグ過程など）
```

## コース

通路幅0.6m、外形2.4m×3.0mの「2」の字型コース。5個の矩形データのみでコース定義し、壁・中心線経路
（BFS探索）・スタート/ゴール位置はすべて自動導出される。

## 走行結果

コース走破時間 15.500 s、最大水平合成加速度 0.987 m/s²（上限1.00 m/s²を全区間で遵守）。詳細は
[`Submission/simulation_stats_report.pdf`](Submission/simulation_stats_report.pdf) を参照。
