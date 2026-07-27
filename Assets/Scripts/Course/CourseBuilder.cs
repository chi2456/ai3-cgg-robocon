using System.Collections.Generic;
using UnityEngine;

namespace Robocon.Course
{
    /// <summary>
    /// 5区間の矩形定義からコース形状（壁・床・スタート/ゴール・中心線経路）を
    /// 実行時に構築する。区間データを単一の情報源とし、壁境界と経路は
    /// 矩形の格子分割から汎用アルゴリズムで導出する（座標のハードコードはしない）。
    /// </summary>
    public class CourseBuilder : MonoBehaviour
    {
        [System.Serializable]
        public struct CourseRect
        {
            public float xMin, xMax, zMin, zMax;
            public CourseRect(float xMin, float xMax, float zMin, float zMax)
            {
                this.xMin = xMin; this.xMax = xMax; this.zMin = zMin; this.zMax = zMax;
            }
            public bool Contains(float x, float z) => x >= xMin && x <= xMax && z >= zMin && z <= zMax;
        }

        [Header("コース区間定義 (R1-R5)")]
        [SerializeField]
        private CourseRect[] segments = new CourseRect[]
        {
            new CourseRect(0.0f, 1.8f, 0.0f, 0.6f), // R1 下段/スタート
            new CourseRect(1.2f, 1.8f, 0.6f, 1.2f), // R2 縦ターン
            new CourseRect(1.2f, 2.4f, 1.2f, 1.8f), // R3 中段
            new CourseRect(1.8f, 2.4f, 1.8f, 2.4f), // R4 縦ターン
            new CourseRect(0.6f, 2.4f, 2.4f, 3.0f), // R5 上段/ゴール
        };

        [Header("スタート/ゴール セル (格子インデックス)")]
        [SerializeField] private Vector2Int startCell = new Vector2Int(0, 0);
        [SerializeField] private Vector2Int goalCell = new Vector2Int(1, 4);

        [Header("壁・床の見た目/物理パラメータ")]
        [SerializeField] private float wallHeight = 0.4f;
        [SerializeField] private float wallThickness = 0.02f;
        [SerializeField] private float floorMargin = 0.5f;
        [SerializeField] private float floorThickness = 0.05f;

        [Header("スタート/ゴール トリガー")]
        [SerializeField] private float triggerThickness = 0.05f;
        [SerializeField] private float triggerHeight = 0.6f;
        [SerializeField] private float triggerWidthMargin = 0.02f;

        public static CourseBuilder Instance { get; private set; }

        public IReadOnlyList<CourseRect> Segments => segments;
        public IReadOnlyList<Vector3> PathPoints => pathPoints;
        public Vector3 StartPosition => pathPoints[0];
        public Vector3 GoalPosition => pathPoints[pathPoints.Count - 1];
        public Quaternion StartRotation { get; private set; }
        public StartLine StartLineInstance { get; private set; }
        public GoalLine GoalLineInstance { get; private set; }

        private float[] xs;
        private float[] zs;
        private bool[,] inside;
        private readonly List<Vector3> pathPoints = new List<Vector3>();
        private PhysicMaterial highGripMaterial;

        private void Awake()
        {
            Instance = this;

            BuildGrid();
            BuildPhysicsMaterial();
            BuildFloor();
            BuildWalls();
            BuildPath();
            BuildStartAndGoalTriggers();
        }

        private void BuildPhysicsMaterial()
        {
            highGripMaterial = new PhysicMaterial("CourseHighGrip")
            {
                dynamicFriction = 1f,
                staticFriction = 1f,
                frictionCombine = PhysicMaterialCombine.Maximum,
                bounciness = 0f,
                bounceCombine = PhysicMaterialCombine.Minimum,
            };
        }

        /// <summary>矩形群の境界X/Z座標を抽出し、セル単位の内外グリッドを作る。</summary>
        private void BuildGrid()
        {
            var xSet = new SortedSet<float>();
            var zSet = new SortedSet<float>();
            foreach (var r in segments)
            {
                xSet.Add(r.xMin); xSet.Add(r.xMax);
                zSet.Add(r.zMin); zSet.Add(r.zMax);
            }
            xs = new float[xSet.Count];
            xSet.CopyTo(xs);
            zs = new float[zSet.Count];
            zSet.CopyTo(zs);

            inside = new bool[xs.Length - 1, zs.Length - 1];
            for (int i = 0; i < xs.Length - 1; i++)
            {
                float cx = (xs[i] + xs[i + 1]) * 0.5f;
                for (int j = 0; j < zs.Length - 1; j++)
                {
                    float cz = (zs[j] + zs[j + 1]) * 0.5f;
                    inside[i, j] = IsInsideAnySegment(cx, cz);
                }
            }
        }

        private bool IsInsideAnySegment(float x, float z)
        {
            foreach (var r in segments)
            {
                if (r.Contains(x, z)) return true;
            }
            return false;
        }

        /// <summary>コース内かどうかをワールドXZ座標で判定する（コース外検知用）。</summary>
        public bool IsInsideCourse(Vector2 xz)
        {
            return IsInsideAnySegment(xz.x, xz.y);
        }

        private Transform GetOrCreateChild(string name)
        {
            var t = transform.Find(name);
            if (t != null) return t;
            var go = new GameObject(name);
            go.transform.SetParent(transform, false);
            return go.transform;
        }

        private void BuildFloor()
        {
            float xMin = xs[0] - floorMargin;
            float xMax = xs[xs.Length - 1] + floorMargin;
            float zMin = zs[0] - floorMargin;
            float zMax = zs[zs.Length - 1] + floorMargin;

            var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.name = "Floor";
            floor.transform.SetParent(GetOrCreateChild("Geometry"), false);
            floor.transform.position = new Vector3((xMin + xMax) * 0.5f, -floorThickness * 0.5f, (zMin + zMax) * 0.5f);
            floor.transform.localScale = new Vector3(xMax - xMin, floorThickness, zMax - zMin);
            floor.GetComponent<Collider>().material = highGripMaterial;
        }

        /// <summary>内外セルの境界エッジを列挙し、そのまま1本ずつ壁ボックスとして生成する。</summary>
        private void BuildWalls()
        {
            var wallParent = GetOrCreateChild("Walls");
            int nx = xs.Length - 1;
            int nz = zs.Length - 1;

            for (int i = 0; i < nx; i++)
            {
                for (int j = 0; j < nz; j++)
                {
                    if (!inside[i, j]) continue;

                    bool leftOutside = i == 0 || !inside[i - 1, j];
                    bool rightOutside = i == nx - 1 || !inside[i + 1, j];
                    bool bottomOutside = j == 0 || !inside[i, j - 1];
                    bool topOutside = j == nz - 1 || !inside[i, j + 1];

                    if (leftOutside) CreateWallSegment(wallParent, new Vector2(xs[i], zs[j]), new Vector2(xs[i], zs[j + 1]));
                    if (rightOutside) CreateWallSegment(wallParent, new Vector2(xs[i + 1], zs[j]), new Vector2(xs[i + 1], zs[j + 1]));
                    if (bottomOutside) CreateWallSegment(wallParent, new Vector2(xs[i], zs[j]), new Vector2(xs[i + 1], zs[j]));
                    if (topOutside) CreateWallSegment(wallParent, new Vector2(xs[i], zs[j + 1]), new Vector2(xs[i + 1], zs[j + 1]));
                }
            }
        }

        private void CreateWallSegment(Transform parent, Vector2 a, Vector2 b)
        {
            Vector2 mid = (a + b) * 0.5f;
            Vector2 dir = (b - a).normalized;
            float length = (b - a).magnitude;
            // 壁の内側面が境界線に一致するよう、法線方向に厚み半分だけ外側へオフセットする。
            Vector2 outwardNormal = new Vector2(dir.y, -dir.x);
            if (IsInsideAnySegment(mid.x + outwardNormal.x * 0.01f, mid.y + outwardNormal.y * 0.01f))
            {
                outwardNormal = -outwardNormal;
            }
            Vector2 center = mid + outwardNormal * (wallThickness * 0.5f);

            var wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
            wall.name = "Wall";
            wall.tag = "Wall";
            wall.transform.SetParent(parent, false);
            wall.transform.position = new Vector3(center.x, wallHeight * 0.5f, center.y);
            wall.transform.rotation = Quaternion.LookRotation(new Vector3(outwardNormal.x, 0f, outwardNormal.y), Vector3.up);
            wall.transform.localScale = new Vector3(length, wallHeight, wallThickness);
            wall.GetComponent<Collider>().material = highGripMaterial;
        }

        private Vector2Int CellOf(Vector2 xz)
        {
            int i = 0, j = 0;
            for (int k = 0; k < xs.Length - 1; k++) if (xz.x >= xs[k]) i = k;
            for (int k = 0; k < zs.Length - 1; k++) if (xz.y >= zs[k]) j = k;
            return new Vector2Int(i, j);
        }

        private Vector3 CellCenter(Vector2Int cell)
        {
            float cx = (xs[cell.x] + xs[cell.x + 1]) * 0.5f;
            float cz = (zs[cell.y] + zs[cell.y + 1]) * 0.5f;
            return new Vector3(cx, 0f, cz);
        }

        /// <summary>startCellからgoalCellまで4連結BFSで経路セル列を求め、
        /// 直進方向が変わる点だけをウェイポイントとして残す（中心線経路）。</summary>
        private void BuildPath()
        {
            int nx = xs.Length - 1, nz = zs.Length - 1;
            var prev = new Vector2Int?[nx, nz];
            var visited = new bool[nx, nz];
            var queue = new Queue<Vector2Int>();
            queue.Enqueue(startCell);
            visited[startCell.x, startCell.y] = true;

            Vector2Int[] dirs = { new Vector2Int(1, 0), new Vector2Int(-1, 0), new Vector2Int(0, 1), new Vector2Int(0, -1) };
            while (queue.Count > 0)
            {
                var cur = queue.Dequeue();
                if (cur == goalCell) break;
                foreach (var d in dirs)
                {
                    var next = cur + d;
                    if (next.x < 0 || next.x >= nx || next.y < 0 || next.y >= nz) continue;
                    if (!inside[next.x, next.y] || visited[next.x, next.y]) continue;
                    visited[next.x, next.y] = true;
                    prev[next.x, next.y] = cur;
                    queue.Enqueue(next);
                }
            }

            var cellChain = new List<Vector2Int>();
            var trace = goalCell;
            cellChain.Add(trace);
            while (trace != startCell)
            {
                var p = prev[trace.x, trace.y];
                if (p == null)
                {
                    Debug.LogError("CourseBuilder: startCellからgoalCellへの経路が見つかりません。");
                    break;
                }
                trace = p.Value;
                cellChain.Add(trace);
            }
            cellChain.Reverse();

            pathPoints.Clear();
            var rawPoints = new List<Vector3>();
            foreach (var c in cellChain) rawPoints.Add(CellCenter(c));

            pathPoints.Add(rawPoints[0]);
            for (int k = 1; k < rawPoints.Count - 1; k++)
            {
                Vector3 dPrev = (rawPoints[k] - rawPoints[k - 1]).normalized;
                Vector3 dNext = (rawPoints[k + 1] - rawPoints[k]).normalized;
                if (Vector3.Dot(dPrev, dNext) < 0.999f)
                {
                    pathPoints.Add(rawPoints[k]);
                }
            }
            pathPoints.Add(rawPoints[rawPoints.Count - 1]);

            StartRotation = pathPoints.Count >= 2
                ? Quaternion.LookRotation(pathPoints[1] - pathPoints[0], Vector3.up)
                : Quaternion.identity;
        }

        private void BuildStartAndGoalTriggers()
        {
            float corridorWidth = Mathf.Min(
                zs[startCell.y + 1] - zs[startCell.y],
                xs[startCell.x + 1] - xs[startCell.x]);

            Vector3 startDir = pathPoints.Count >= 2 ? (pathPoints[1] - pathPoints[0]).normalized : Vector3.forward;
            Vector3 goalDir = pathPoints.Count >= 2
                ? (pathPoints[pathPoints.Count - 1] - pathPoints[pathPoints.Count - 2]).normalized
                : Vector3.forward;

            var startGo = new GameObject("StartLine");
            startGo.transform.SetParent(transform, false);
            startGo.transform.position = pathPoints[0];
            startGo.transform.rotation = Quaternion.LookRotation(startDir, Vector3.up);
            var startTrigger = startGo.AddComponent<BoxCollider>();
            startTrigger.isTrigger = true;
            startTrigger.center = new Vector3(0f, triggerHeight * 0.5f, 0f);
            startTrigger.size = new Vector3(corridorWidth + triggerWidthMargin, triggerHeight, triggerThickness);
            StartLineInstance = startGo.AddComponent<StartLine>();

            var goalGo = new GameObject("GoalLine");
            goalGo.transform.SetParent(transform, false);
            goalGo.transform.position = pathPoints[pathPoints.Count - 1];
            goalGo.transform.rotation = Quaternion.LookRotation(goalDir, Vector3.up);
            var goalTrigger = goalGo.AddComponent<BoxCollider>();
            goalTrigger.isTrigger = true;
            goalTrigger.center = new Vector3(0f, triggerHeight * 0.5f, 0f);
            goalTrigger.size = new Vector3(corridorWidth + triggerWidthMargin, triggerHeight, triggerThickness);
            GoalLineInstance = goalGo.AddComponent<GoalLine>();
        }
    }
}
