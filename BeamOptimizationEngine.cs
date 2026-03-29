using BriefFiniteElementNet;
using BriefFiniteElementNet.Elements;
using BriefFiniteElementNet.Materials;
using BriefFiniteElementNet.Sections;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using BfePoint = BriefFiniteElementNet.Point;

namespace FEA_Beam_Solver
{
    internal sealed class BeamGraph
    {
        public BeamGraph(List<Point3d> nodes, List<BeamGraphEdge> edges, HashSet<int> loadNodeIndices, HashSet<int> supportNodeIndices, double weldTolerance)
        {
            Nodes = nodes;
            Edges = edges;
            LoadNodeIndices = loadNodeIndices;
            SupportNodeIndices = supportNodeIndices;
            WeldTolerance = weldTolerance;
        }

        public List<Point3d> Nodes { get; }
        public List<BeamGraphEdge> Edges { get; }
        public HashSet<int> LoadNodeIndices { get; }
        public HashSet<int> SupportNodeIndices { get; }
        public double WeldTolerance { get; }

        public BeamGraph Clone()
        {
            return new BeamGraph(
                Nodes.Select(point => new Point3d(point.X, point.Y, point.Z)).ToList(),
                Edges.Select(edge => new BeamGraphEdge(edge.StartIndex, edge.EndIndex)).ToList(),
                new HashSet<int>(LoadNodeIndices),
                new HashSet<int>(SupportNodeIndices),
                WeldTolerance);
        }

        public List<Curve> ToCurves()
        {
            return Edges
                .Select(edge => (Curve)new LineCurve(Nodes[edge.StartIndex], Nodes[edge.EndIndex]))
                .ToList();
        }

        public int GetDegree(int nodeIndex)
        {
            return Edges.Count(edge => edge.StartIndex == nodeIndex || edge.EndIndex == nodeIndex);
        }

        public bool HasEdge(int a, int b)
        {
            return Edges.Any(edge =>
                (edge.StartIndex == a && edge.EndIndex == b) ||
                (edge.StartIndex == b && edge.EndIndex == a));
        }
    }

    internal sealed class BeamGraphEdge
    {
        public BeamGraphEdge(int startIndex, int endIndex)
        {
            StartIndex = startIndex;
            EndIndex = endIndex;
        }

        public int StartIndex { get; set; }
        public int EndIndex { get; set; }
    }

    internal sealed class BeamNodeResult
    {
        public BeamNodeResult(int index, Point3d originalPoint, Vector3d displacement, bool isSupport, bool isLoad, int degree)
        {
            Index = index;
            OriginalPoint = originalPoint;
            Displacement = displacement;
            IsSupport = isSupport;
            IsLoad = isLoad;
            Degree = degree;
        }

        public int Index { get; }
        public Point3d OriginalPoint { get; }
        public Vector3d Displacement { get; }
        public double DisplacementMagnitude => Displacement.Length;
        public bool IsSupport { get; }
        public bool IsLoad { get; }
        public int Degree { get; }
    }

    internal sealed class BeamElementResult
    {
        public BeamElementResult(int index, int startNodeIndex, int endNodeIndex, LineCurve originalCurve, LineCurve deformedCurve, double forceValue, double averageDisplacement)
        {
            Index = index;
            StartNodeIndex = startNodeIndex;
            EndNodeIndex = endNodeIndex;
            OriginalCurve = originalCurve;
            DeformedCurve = deformedCurve;
            ForceValue = forceValue;
            AverageDisplacement = averageDisplacement;
        }

        public int Index { get; }
        public int StartNodeIndex { get; }
        public int EndNodeIndex { get; }
        public LineCurve OriginalCurve { get; }
        public LineCurve DeformedCurve { get; }
        public double ForceValue { get; }
        public double AverageDisplacement { get; }
    }

    internal sealed class BeamAnalysisResult
    {
        public BeamAnalysisResult(
            string summary,
            List<Curve> elementCurves,
            List<double> elementValues,
            List<Curve> displacementVectors,
            List<double> nodeDisplacementMagnitudes,
            List<Curve> deformedCurves,
            List<BeamNodeResult> nodes,
            List<BeamElementResult> elements)
        {
            Summary = summary;
            ElementCurves = elementCurves;
            ElementValues = elementValues;
            DisplacementVectors = displacementVectors;
            NodeDisplacementMagnitudes = nodeDisplacementMagnitudes;
            DeformedCurves = deformedCurves;
            Nodes = nodes;
            Elements = elements;
        }

        public string Summary { get; }
        public List<Curve> ElementCurves { get; }
        public List<double> ElementValues { get; }
        public List<Curve> DisplacementVectors { get; }
        public List<double> NodeDisplacementMagnitudes { get; }
        public List<Curve> DeformedCurves { get; }
        public List<BeamNodeResult> Nodes { get; }
        public List<BeamElementResult> Elements { get; }
        public double MaxNodeDisplacement => NodeDisplacementMagnitudes.DefaultIfEmpty(0.0).Max();
        public double AverageNodeDisplacement => NodeDisplacementMagnitudes.DefaultIfEmpty(0.0).Average();
    }

    internal sealed class BeamOptimizationOptions
    {
        public int Metric { get; set; } = 2;
        public double DeformScale { get; set; } = 2.0;
        public int Iterations { get; set; } = 8;
        public int TargetAddCount { get; set; } = 0;
        public int TargetRemoveCount { get; set; } = 0;
        public int SearchBreadth { get; set; } = 6;
        public double MoveFactor { get; set; } = 0.35;
        public double AddDisplacementThresholdRatio { get; set; } = 0.75;
        public double RemoveForceThresholdRatio { get; set; } = 0.10;
        public double MaxConnectionDistance { get; set; } = 0.0;
        public double WeldTolerance { get; set; } = 0.5;
        public double ImprovementTolerance { get; set; } = 1e-6;
        public int MaxCandidatesPerAction { get; set; } = 3;
    }

    internal sealed class BeamOptimizationResult
    {
        public BeamOptimizationResult(
            BeamAnalysisResult initialAnalysis,
            BeamAnalysisResult finalAnalysis,
            BeamGraph optimizedGraph,
            Mesh forceMesh,
            int addedCount,
            int removedCount,
            int movedCount,
            string log)
        {
            InitialAnalysis = initialAnalysis;
            FinalAnalysis = finalAnalysis;
            OptimizedGraph = optimizedGraph;
            ForceMesh = forceMesh;
            AddedCount = addedCount;
            RemovedCount = removedCount;
            MovedCount = movedCount;
            Log = log;
        }

        public BeamAnalysisResult InitialAnalysis { get; }
        public BeamAnalysisResult FinalAnalysis { get; }
        public BeamGraph OptimizedGraph { get; }
        public Mesh ForceMesh { get; }
        public int AddedCount { get; }
        public int RemovedCount { get; }
        public int MovedCount { get; }
        public string Log { get; }
        public double DisplacementReduction => InitialAnalysis.MaxNodeDisplacement - FinalAnalysis.MaxNodeDisplacement;
    }

    internal enum OptimizationAction
    {
        None,
        Move,
        Add,
        Remove
    }

    internal static class BeamGraphBuilder
    {
        public static BeamGraph Build(IReadOnlyList<Curve> beams, IReadOnlyList<Point3d> loads, IReadOnlyList<Point3d> supports, double weldTolerance)
        {
            var nodes = new List<Point3d>();
            var edges = new List<BeamGraphEdge>();
            var nodeIndexByKey = new Dictionary<(long, long, long), int>();
            var seenEdges = new HashSet<((long, long, long), (long, long, long))>();

            for (int i = 0; i < beams.Count; i++)
            {
                Curve beam = beams[i];
                if (beam == null)
                {
                    continue;
                }

                Point3d start = beam.PointAtStart;
                Point3d end = beam.PointAtEnd;
                if (start.DistanceTo(end) <= weldTolerance * 0.1)
                {
                    continue;
                }

                int startIndex = GetOrCreateNodeIndex(start, nodes, nodeIndexByKey, weldTolerance);
                int endIndex = GetOrCreateNodeIndex(end, nodes, nodeIndexByKey, weldTolerance);
                if (startIndex == endIndex)
                {
                    continue;
                }

                var startKey = Quantize(nodes[startIndex], weldTolerance);
                var endKey = Quantize(nodes[endIndex], weldTolerance);
                var undirected = CompareKeys(startKey, endKey) <= 0 ? (startKey, endKey) : (endKey, startKey);
                if (!seenEdges.Add(undirected))
                {
                    continue;
                }

                edges.Add(new BeamGraphEdge(startIndex, endIndex));
            }

            if (edges.Count == 0)
            {
                throw new Exception("No valid beam segments were found after welding and duplicate removal.");
            }

            var loadNodes = new HashSet<int>();
            foreach (Point3d load in loads)
            {
                loadNodes.Add(FindNearestNodeIndex(load, nodes, weldTolerance, "load"));
            }

            var supportNodes = new HashSet<int>();
            foreach (Point3d support in supports)
            {
                supportNodes.Add(FindNearestNodeIndex(support, nodes, weldTolerance, "support"));
            }

            if (supportNodes.Count == 0)
            {
                throw new Exception("At least one support node is required.");
            }

            return new BeamGraph(nodes, edges, loadNodes, supportNodes, weldTolerance);
        }

        internal static (long, long, long) Quantize(Point3d point, double tolerance)
        {
            return (
                (long)Math.Round(point.X / tolerance),
                (long)Math.Round(point.Y / tolerance),
                (long)Math.Round(point.Z / tolerance));
        }

        internal static int CompareKeys((long, long, long) a, (long, long, long) b)
        {
            int x = a.Item1.CompareTo(b.Item1);
            if (x != 0)
            {
                return x;
            }

            int y = a.Item2.CompareTo(b.Item2);
            if (y != 0)
            {
                return y;
            }

            return a.Item3.CompareTo(b.Item3);
        }

        private static int GetOrCreateNodeIndex(Point3d point, List<Point3d> nodes, Dictionary<(long, long, long), int> nodeIndexByKey, double tolerance)
        {
            var key = Quantize(point, tolerance);
            int existingIndex;
            if (nodeIndexByKey.TryGetValue(key, out existingIndex))
            {
                return existingIndex;
            }

            int index = nodes.Count;
            nodes.Add(point);
            nodeIndexByKey[key] = index;
            return index;
        }

        private static int FindNearestNodeIndex(Point3d point, List<Point3d> nodes, double tolerance, string label)
        {
            int bestIndex = -1;
            double bestDistance = double.MaxValue;
            for (int i = 0; i < nodes.Count; i++)
            {
                double distance = nodes[i].DistanceTo(point);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestIndex = i;
                }
            }

            if (bestIndex < 0 || bestDistance > tolerance * 2.0)
            {
                throw new Exception($"No welded node was found near the {label} point {point} using tol={tolerance}.");
            }

            return bestIndex;
        }
    }

    internal static class BeamAnalysisEngine
    {
        public static BeamAnalysisResult Analyze(BeamGraph graph, int metric, double deformScale)
        {
            var model = new Model();
            var material = UniformIsotropicMaterial.CreateFromYoungPoisson(2.3e3, 0.33);
            var section = MakeRectSection(30.0, 30.0, -1);

            var bfeNodes = new List<Node>(graph.Nodes.Count);
            for (int i = 0; i < graph.Nodes.Count; i++)
            {
                Point3d point = graph.Nodes[i];
                var node = new Node(point.X, point.Y, point.Z) { Label = $"n{i}" };
                node.Constraints = Constraints.Released;
                model.Nodes.Add(node);
                bfeNodes.Add(node);
            }

            var elements = new List<BarElement>(graph.Edges.Count);
            for (int i = 0; i < graph.Edges.Count; i++)
            {
                BeamGraphEdge edge = graph.Edges[i];
                var element = new BarElement(bfeNodes[edge.StartIndex], bfeNodes[edge.EndIndex])
                {
                    Label = $"e{i}",
                    Behavior = BarElementBehaviours.FullFrame,
                    Section = section,
                    Material = material,
                    WebRotation = ComputeWebRotationDegrees(bfeNodes[edge.StartIndex].Location, bfeNodes[edge.EndIndex].Location)
                };
                model.Elements.Add(element);
                elements.Add(element);
            }

            ValidateMinimumLengths(elements, 0.1);

            var supportList = graph.SupportNodeIndices
                .OrderBy(index => index)
                .Select(index => bfeNodes[index])
                .ToList();
            supportList[0].Constraints = Constraints.Fixed;
            for (int i = 1; i < supportList.Count; i++)
            {
                supportList[i].Constraints = Constraints.MovementFixed;
            }

            foreach (int loadNodeIndex in graph.LoadNodeIndices)
            {
                bfeNodes[loadNodeIndex].Loads.Add(new NodalLoad(new Force(0, 0, -2, 0, 0, 0)));
            }

            EnsureEveryComponentHasSupport(model, new HashSet<Node>(supportList));
            model.Solve_MPC();

            var nodeResults = new List<BeamNodeResult>(graph.Nodes.Count);
            var nodeDispVectors = new Dictionary<int, Vector3d>();
            var displacementCurves = new List<Curve>();
            var nodeDispMagnitudes = new List<double>();

            for (int i = 0; i < bfeNodes.Count; i++)
            {
                Node node = bfeNodes[i];
                var disp = node.GetNodalDisplacement(LoadCase.DefaultLoadCase);
                var dispVector = new Vector3d(disp.DX, disp.DY, disp.DZ);
                nodeDispVectors[i] = dispVector;
                nodeDispMagnitudes.Add(dispVector.Length);

                Point3d start = graph.Nodes[i];
                Point3d end = start + (dispVector * deformScale);
                displacementCurves.Add(new LineCurve(start, end));

                nodeResults.Add(new BeamNodeResult(
                    i,
                    start,
                    dispVector,
                    graph.SupportNodeIndices.Contains(i),
                    graph.LoadNodeIndices.Contains(i),
                    graph.GetDegree(i)));
            }

            var elementCurves = new List<Curve>(graph.Edges.Count);
            var elementValues = new List<double>(graph.Edges.Count);
            var deformedCurves = new List<Curve>(graph.Edges.Count);
            var elementResults = new List<BeamElementResult>(graph.Edges.Count);

            for (int i = 0; i < elements.Count; i++)
            {
                BeamGraphEdge edge = graph.Edges[i];
                Point3d start = graph.Nodes[edge.StartIndex];
                Point3d end = graph.Nodes[edge.EndIndex];
                var original = new LineCurve(start, end);
                elementCurves.Add(original);

                Point3d deformedStart = start + (nodeDispVectors[edge.StartIndex] * deformScale);
                Point3d deformedEnd = end + (nodeDispVectors[edge.EndIndex] * deformScale);
                var deformed = new LineCurve(deformedStart, deformedEnd);
                deformedCurves.Add(deformed);

                Force force = elements[i].GetInternalForceAt(0.0, LoadCase.DefaultLoadCase);
                double forceValue = ExtractMetricValue(force, metric);
                elementValues.Add(forceValue);

                double averageDisp = 0.5 * (nodeResults[edge.StartIndex].DisplacementMagnitude + nodeResults[edge.EndIndex].DisplacementMagnitude);
                elementResults.Add(new BeamElementResult(i, edge.StartIndex, edge.EndIndex, original, deformed, forceValue, averageDisp));
            }

            Force sumReactions = Force.Zero;
            foreach (Node node in bfeNodes)
            {
                if (!node.Constraints.Equals(Constraints.Released))
                {
                    sumReactions += node.GetSupportReaction(LoadCase.DefaultLoadCase);
                }
            }

            Force sumApplied = Force.Zero;
            foreach (Node node in bfeNodes)
            {
                sumApplied += node.GetTotalApplyingForces(LoadCase.DefaultLoadCase);
            }

            string summary =
                $"Support reactions sum: Fx={sumReactions.Fx:0.###}, Fy={sumReactions.Fy:0.###}, Fz={sumReactions.Fz:0.###}, " +
                $"Mx={sumReactions.Mx:0.###}, My={sumReactions.My:0.###}, Mz={sumReactions.Mz:0.###}\n" +
                $"Applied loads sum: Fx={sumApplied.Fx:0.###}, Fy={sumApplied.Fy:0.###}, Fz={sumApplied.Fz:0.###}, " +
                $"Mx={sumApplied.Mx:0.###}, My={sumApplied.My:0.###}, Mz={sumApplied.Mz:0.###}";

            return new BeamAnalysisResult(summary, elementCurves, elementValues, displacementCurves, nodeDispMagnitudes, deformedCurves, nodeResults, elementResults);
        }

        private static double ExtractMetricValue(Force force, int metric)
        {
            switch (metric)
            {
                case 0:
                    return Math.Abs(force.Fx);
                case 1:
                    return Math.Sqrt((force.Fy * force.Fy) + (force.Fz * force.Fz));
                case 2:
                    return Math.Sqrt((force.My * force.My) + (force.Mz * force.Mz));
                case 3:
                    return Math.Abs(force.Mx);
                default:
                    return Math.Sqrt((force.My * force.My) + (force.Mz * force.Mz));
            }
        }

        private static UniformGeometric1DSection MakeRectSection(double b, double h, double jOverride)
        {
            double y = b / 2.0;
            double z = h / 2.0;
            var geom = new PointYZ[]
            {
                new PointYZ(-y, -z),
                new PointYZ(y, -z),
                new PointYZ(y, z),
                new PointYZ(-y, z),
                new PointYZ(-y, -z)
            };

            return jOverride > 0
                ? new UniformGeometric1DSection(geom, jOverride)
                : new UniformGeometric1DSection(geom);
        }

        private static double ComputeWebRotationDegrees(BfePoint a, BfePoint b)
        {
            var direction = new Vector3d(b.X - a.X, b.Y - a.Y, b.Z - a.Z);
            if (!direction.Unitize())
            {
                return 0.0;
            }

            return Math.Abs(direction.Z) > 0.9 ? 90.0 : 0.0;
        }

        private static void ValidateMinimumLengths(IEnumerable<BarElement> elements, double minimumLength)
        {
            foreach (BarElement element in elements)
            {
                BfePoint a = element.StartNode.Location;
                BfePoint b = element.EndNode.Location;
                double dx = a.X - b.X;
                double dy = a.Y - b.Y;
                double dz = a.Z - b.Z;
                double length = Math.Sqrt((dx * dx) + (dy * dy) + (dz * dz));
                if (length < minimumLength)
                {
                    throw new Exception($"Too-short element (<{minimumLength}): {element.Label}  L={length}");
                }
            }
        }

        private static void EnsureEveryComponentHasSupport(Model model, HashSet<Node> supportNodes)
        {
            var adjacency = new Dictionary<Node, List<Node>>();
            foreach (Node node in model.Nodes)
            {
                adjacency[node] = new List<Node>();
            }

            foreach (BarElement element in model.Elements)
            {
                adjacency[element.StartNode].Add(element.EndNode);
                adjacency[element.EndNode].Add(element.StartNode);
            }

            var visited = new HashSet<Node>();
            int componentId = 0;

            foreach (Node start in model.Nodes)
            {
                if (visited.Contains(start))
                {
                    continue;
                }

                componentId++;
                var queue = new Queue<Node>();
                queue.Enqueue(start);
                visited.Add(start);
                bool hasSupport = supportNodes.Contains(start);
                int count = 0;

                while (queue.Count > 0)
                {
                    Node current = queue.Dequeue();
                    count++;
                    if (supportNodes.Contains(current))
                    {
                        hasSupport = true;
                    }

                    foreach (Node neighbor in adjacency[current])
                    {
                        if (visited.Add(neighbor))
                        {
                            queue.Enqueue(neighbor);
                        }
                    }
                }

                if (!hasSupport)
                {
                    throw new Exception($"Component #{componentId} (size {count}) has no supports and will be singular.");
                }
            }
        }
    }

    internal static class BeamOptimizationEngine
    {
        public static BeamOptimizationResult Optimize(IReadOnlyList<Curve> beams, IReadOnlyList<Point3d> loads, IReadOnlyList<Point3d> supports, Brep brep, BeamOptimizationOptions options)
        {
            BeamGraph currentGraph = BeamGraphBuilder.Build(beams, loads, supports, options.WeldTolerance);
            BeamAnalysisResult initial = BeamAnalysisEngine.Analyze(currentGraph, options.Metric, options.DeformScale);
            BeamAnalysisResult currentAnalysis = initial;

            int addedCount = 0;
            int removedCount = 0;
            int movedCount = 0;
            var logLines = new List<string>
            {
                $"Initial max displacement: {initial.MaxNodeDisplacement:0.####}",
                $"Targets -> add: {Math.Max(0, options.TargetAddCount)}, remove: {Math.Max(0, options.TargetRemoveCount)}, search breadth: {Math.Max(1, options.SearchBreadth)}"
            };

            for (int iteration = 0; iteration < Math.Max(1, options.Iterations); iteration++)
            {
                bool changed = false;

                if (addedCount < Math.Max(0, options.TargetAddCount))
                {
                    Candidate bestAdd = FindBestCandidate(GenerateAddCandidates(currentGraph, currentAnalysis, options));
                    if (ApplyCandidate(bestAdd, currentAnalysis, ref currentGraph, ref currentAnalysis, ref addedCount, ref removedCount, ref movedCount, options, logLines, iteration + 1))
                    {
                        changed = true;
                    }
                }

                if (removedCount < Math.Max(0, options.TargetRemoveCount))
                {
                    Candidate bestRemove = FindBestCandidate(GenerateRemoveCandidates(currentGraph, currentAnalysis, options));
                    if (ApplyCandidate(bestRemove, currentAnalysis, ref currentGraph, ref currentAnalysis, ref addedCount, ref removedCount, ref movedCount, options, logLines, iteration + 1))
                    {
                        changed = true;
                    }
                }

                Candidate bestMove = FindBestCandidate(GenerateMoveCandidates(currentGraph, currentAnalysis, brep, options));
                if (ApplyCandidate(bestMove, currentAnalysis, ref currentGraph, ref currentAnalysis, ref addedCount, ref removedCount, ref movedCount, options, logLines, iteration + 1))
                {
                    changed = true;
                }

                if (!changed)
                {
                    logLines.Add($"Iteration {iteration + 1}: no improving candidate found.");
                    break;
                }
            }

            Mesh forceMesh = CreateForceMesh(brep, currentAnalysis);
            logLines.Add($"Final max displacement: {currentAnalysis.MaxNodeDisplacement:0.####}");
            logLines.Add($"Displacement reduction: {initial.MaxNodeDisplacement - currentAnalysis.MaxNodeDisplacement:0.####}");

            return new BeamOptimizationResult(
                initial,
                currentAnalysis,
                currentGraph,
                forceMesh,
                addedCount,
                removedCount,
                movedCount,
                string.Join(Environment.NewLine, logLines));
        }

        private static Candidate FindBestCandidate(IEnumerable<Candidate> candidates)
        {
            Candidate best = null;
            foreach (Candidate candidate in candidates)
            {
                if (candidate == null)
                {
                    continue;
                }

                if (best == null || candidate.Improvement > best.Improvement)
                {
                    best = candidate;
                }
            }

            return best;
        }

        private static bool ApplyCandidate(
            Candidate candidate,
            BeamAnalysisResult baselineAnalysis,
            ref BeamGraph currentGraph,
            ref BeamAnalysisResult currentAnalysis,
            ref int addedCount,
            ref int removedCount,
            ref int movedCount,
            BeamOptimizationOptions options,
            List<string> logLines,
            int iteration)
        {
            if (candidate == null || candidate.Improvement <= options.ImprovementTolerance)
            {
                return false;
            }

            currentGraph = candidate.Graph;
            currentAnalysis = candidate.Analysis;

            switch (candidate.Action)
            {
                case OptimizationAction.Add:
                    addedCount++;
                    break;
                case OptimizationAction.Remove:
                    removedCount++;
                    break;
                case OptimizationAction.Move:
                    movedCount++;
                    break;
            }

            logLines.Add(
                $"Iteration {iteration}: accepted {candidate.Action} candidate, " +
                $"max displacement {baselineAnalysis.MaxNodeDisplacement:0.####} -> {candidate.Analysis.MaxNodeDisplacement:0.####}, " +
                $"improvement = {candidate.Improvement:0.####}");

            return true;
        }

        private static IEnumerable<Candidate> GenerateMoveCandidates(BeamGraph currentGraph, BeamAnalysisResult currentAnalysis, Brep brep, BeamOptimizationOptions options)
        {
            var results = new List<Candidate>();
            int searchBreadth = Math.Max(1, Math.Max(options.SearchBreadth, options.MaxCandidatesPerAction));
            var movableNodes = currentAnalysis.Nodes
                .Where(node => !node.IsSupport && !node.IsLoad)
                .OrderByDescending(node => node.DisplacementMagnitude)
                .Take(searchBreadth * 2)
                .ToList();

            double[] moveScales =
            {
                Math.Max(0.05, options.MoveFactor * 0.5),
                Math.Max(0.05, options.MoveFactor),
                Math.Max(0.05, options.MoveFactor * 1.5)
            };

            foreach (BeamNodeResult node in movableNodes)
            {
                if (node.DisplacementMagnitude <= options.ImprovementTolerance)
                {
                    continue;
                }

                for (int scaleIndex = 0; scaleIndex < moveScales.Length; scaleIndex++)
                {
                    BeamGraph movedGraph = currentGraph.Clone();
                    Vector3d moveVector = node.Displacement * -moveScales[scaleIndex];
                    Point3d movedPoint = movedGraph.Nodes[node.Index] + moveVector;
                    if (brep != null)
                    {
                        movedPoint = ProjectPointToBrep(brep, movedPoint, options.WeldTolerance, movedGraph.Nodes[node.Index]);
                    }

                    if (movedPoint.DistanceTo(movedGraph.Nodes[node.Index]) <= options.WeldTolerance * 0.1)
                    {
                        continue;
                    }

                    movedGraph.Nodes[node.Index] = movedPoint;
                    Candidate candidate = TryAnalyzeCandidate(movedGraph, currentAnalysis, OptimizationAction.Move, options);
                    if (candidate != null)
                    {
                        results.Add(candidate);
                    }
                }
            }

            return results;
        }

        private static IEnumerable<Candidate> GenerateAddCandidates(BeamGraph currentGraph, BeamAnalysisResult currentAnalysis, BeamOptimizationOptions options)
        {
            var results = new List<Candidate>();
            int searchBreadth = Math.Max(1, Math.Max(options.SearchBreadth, options.MaxCandidatesPerAction));
            double maxDisp = Math.Max(currentAnalysis.MaxNodeDisplacement, options.ImprovementTolerance);
            double maxConnectionDistance = options.MaxConnectionDistance > 0.0
                ? options.MaxConnectionDistance
                : ComputeGraphScale(currentGraph) * 0.45;

            var sourceNodes = currentAnalysis.Nodes
                .Where(node => node.DisplacementMagnitude >= maxDisp * options.AddDisplacementThresholdRatio)
                .OrderByDescending(node => node.DisplacementMagnitude)
                .Take(searchBreadth * 2)
                .ToList();

            foreach (BeamNodeResult source in sourceNodes)
            {
                var targetScores = new List<(int targetIndex, double score)>();
                for (int targetIndex = 0; targetIndex < currentGraph.Nodes.Count; targetIndex++)
                {
                    if (targetIndex == source.Index || currentGraph.HasEdge(source.Index, targetIndex))
                    {
                        continue;
                    }

                    Point3d a = currentGraph.Nodes[source.Index];
                    Point3d b = currentGraph.Nodes[targetIndex];
                    double distance = a.DistanceTo(b);
                    if (distance <= options.WeldTolerance * 2.0 || distance > maxConnectionDistance)
                    {
                        continue;
                    }

                    BeamNodeResult targetNode = currentAnalysis.Nodes[targetIndex];
                    double score = distance * (0.5 + targetNode.DisplacementMagnitude);
                    if (currentGraph.SupportNodeIndices.Contains(targetIndex))
                    {
                        score *= 0.6;
                    }
                    if (currentGraph.GetDegree(targetIndex) <= 1)
                    {
                        score *= 0.8;
                    }

                    targetScores.Add((targetIndex, score));
                }

                foreach ((int targetIndex, double score) in targetScores
                    .OrderBy(item => item.score)
                    .Take(searchBreadth))
                {
                    BeamGraph addedGraph = currentGraph.Clone();
                    addedGraph.Edges.Add(new BeamGraphEdge(source.Index, targetIndex));
                    Candidate candidate = TryAnalyzeCandidate(addedGraph, currentAnalysis, OptimizationAction.Add, options);
                    if (candidate != null)
                    {
                        results.Add(candidate);
                    }
                }
            }

            return results;
        }

        private static IEnumerable<Candidate> GenerateRemoveCandidates(BeamGraph currentGraph, BeamAnalysisResult currentAnalysis, BeamOptimizationOptions options)
        {
            var results = new List<Candidate>();
            int searchBreadth = Math.Max(1, Math.Max(options.SearchBreadth, options.MaxCandidatesPerAction));
            double maxForce = currentAnalysis.ElementValues.DefaultIfEmpty(0.0).Max();
            if (maxForce <= options.ImprovementTolerance)
            {
                return results;
            }

            var removable = currentAnalysis.Elements
                .Where(element =>
                    element.ForceValue <= maxForce * options.RemoveForceThresholdRatio &&
                    currentGraph.GetDegree(element.StartNodeIndex) > 2 &&
                    currentGraph.GetDegree(element.EndNodeIndex) > 2 &&
                    !currentGraph.SupportNodeIndices.Contains(element.StartNodeIndex) &&
                    !currentGraph.SupportNodeIndices.Contains(element.EndNodeIndex))
                .OrderBy(element => element.ForceValue)
                .ThenBy(element => element.AverageDisplacement)
                .Take(searchBreadth * 2);

            foreach (BeamElementResult element in removable)
            {
                BeamGraph removedGraph = currentGraph.Clone();
                removedGraph.Edges.RemoveAll(edge =>
                    (edge.StartIndex == element.StartNodeIndex && edge.EndIndex == element.EndNodeIndex) ||
                    (edge.StartIndex == element.EndNodeIndex && edge.EndIndex == element.StartNodeIndex));

                if (removedGraph.Edges.Count == 0)
                {
                    continue;
                }

                Candidate candidate = TryAnalyzeCandidate(removedGraph, currentAnalysis, OptimizationAction.Remove, options);
                if (candidate != null)
                {
                    results.Add(candidate);
                }
            }

            return results;
        }

        private static Candidate TryAnalyzeCandidate(BeamGraph graph, BeamAnalysisResult currentAnalysis, OptimizationAction action, BeamOptimizationOptions options)
        {
            try
            {
                BeamAnalysisResult analysis = BeamAnalysisEngine.Analyze(graph, options.Metric, options.DeformScale);
                double improvement = currentAnalysis.MaxNodeDisplacement - analysis.MaxNodeDisplacement;
                return new Candidate(graph, analysis, action, improvement);
            }
            catch
            {
                return null;
            }
        }

        private static double ComputeGraphScale(BeamGraph graph)
        {
            if (graph.Nodes.Count == 0)
            {
                return 1.0;
            }

            BoundingBox boundingBox = new BoundingBox(graph.Nodes);
            return boundingBox.Diagonal.Length <= 0.0 ? 1.0 : boundingBox.Diagonal.Length;
        }

        private static Point3d ProjectPointToBrep(Brep brep, Point3d point, double tolerance, Point3d fallback)
        {
            Point3d closestPoint;
            ComponentIndex componentIndex;
            double s;
            double t;
            Vector3d normal;
            return brep.ClosestPoint(point, out closestPoint, out componentIndex, out s, out t, tolerance, out normal)
                ? closestPoint
                : fallback;
        }

        private static Mesh CreateForceMesh(Brep brep, BeamAnalysisResult analysis)
        {
            if (brep == null)
            {
                return new Mesh();
            }

            Mesh[] brepMeshes = Mesh.CreateFromBrep(brep, MeshingParameters.FastRenderMesh);
            if (brepMeshes == null || brepMeshes.Length == 0)
            {
                return new Mesh();
            }

            var mesh = new Mesh();
            foreach (Mesh piece in brepMeshes)
            {
                if (piece != null && piece.IsValid)
                {
                    mesh.Append(piece);
                }
            }

            mesh.Vertices.CombineIdentical(true, true);
            mesh.Vertices.CullUnused();
            mesh.Faces.CullDegenerateFaces();
            mesh.VertexColors.Clear();

            double minValue = analysis.ElementValues.DefaultIfEmpty(0.0).Min();
            double maxValue = analysis.ElementValues.DefaultIfEmpty(0.0).Max();

            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                Point3d vertex = mesh.Vertices.Point3dAt(i);
                double nearestValue = FindNearestElementValue(vertex, analysis.Elements);
                double t = Remap01(nearestValue, minValue, maxValue);
                mesh.VertexColors.Add(MapGradient(t));
            }

            return mesh;
        }

        private static double FindNearestElementValue(Point3d point, IReadOnlyList<BeamElementResult> elements)
        {
            double bestDistance = double.MaxValue;
            double bestValue = 0.0;
            for (int i = 0; i < elements.Count; i++)
            {
                double parameter;
                if (!elements[i].OriginalCurve.ClosestPoint(point, out parameter))
                {
                    continue;
                }

                Point3d closest = elements[i].OriginalCurve.PointAt(parameter);
                double distance = closest.DistanceTo(point);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestValue = elements[i].ForceValue;
                }
            }

            return bestValue;
        }

        internal static Color MapGradient(double t)
        {
            t = Math.Max(0.0, Math.Min(1.0, t));
            if (t <= 0.5)
            {
                return LerpColor(Color.FromArgb(50, 120, 255), Color.FromArgb(255, 220, 80), t * 2.0);
            }

            return LerpColor(Color.FromArgb(255, 220, 80), Color.FromArgb(220, 60, 40), (t - 0.5) * 2.0);
        }

        internal static Color LerpColor(Color a, Color b, double t)
        {
            t = Math.Max(0.0, Math.Min(1.0, t));
            int r = (int)Math.Round(a.R + ((b.R - a.R) * t));
            int g = (int)Math.Round(a.G + ((b.G - a.G) * t));
            int bl = (int)Math.Round(a.B + ((b.B - a.B) * t));
            return Color.FromArgb(r, g, bl);
        }

        internal static double Remap01(double x, double min, double max)
        {
            if (max <= min)
            {
                return 0.0;
            }

            return (x - min) / (max - min);
        }

        private sealed class Candidate
        {
            public Candidate(BeamGraph graph, BeamAnalysisResult analysis, OptimizationAction action, double improvement)
            {
                Graph = graph;
                Analysis = analysis;
                Action = action;
                Improvement = improvement;
            }

            public BeamGraph Graph { get; }
            public BeamAnalysisResult Analysis { get; }
            public OptimizationAction Action { get; }
            public double Improvement { get; }
        }
    }
}
