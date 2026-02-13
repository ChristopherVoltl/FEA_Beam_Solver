using Grasshopper;
using Grasshopper.Kernel;
using BriefFiniteElementNet;
using BriefFiniteElementNet.Elements;
using BriefFiniteElementNet.Materials;
using BriefFiniteElementNet.Sections;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Drawing;
using BfePoint = BriefFiniteElementNet.Point;

namespace FEA_Beam_Solver
{
    public class FEA_Beam_SolverComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public FEA_Beam_SolverComponent()
          : base("Beam FEA", "bFE",
            "Analyse a set of spatial beams",
            "FGAM", "FEA")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // Use the pManager object to register your input parameters.
            // You can often supply default values when creating parameters.
            // All parameters must have the correct access type. If you want 
            // to import lists or trees of values, modify the ParamAccess flag.
            pManager.AddCurveParameter("Beams", "B", "Input beams of the Structure", GH_ParamAccess.list);
            pManager.AddPointParameter("Loads", "L", "Loads of the structure", GH_ParamAccess.list);
            pManager.AddPointParameter("Supports", "S", "Fixed supportsof the structure", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Metric", "M",
                                    "0=Axial | 1=Shear | 2=BendingMoment | 3=Torsion", GH_ParamAccess.item, 2);
            pManager.AddNumberParameter("DeformScale", "S", "Display scale factor for displacements", GH_ParamAccess.item, 2.0);




            // If you want to change properties of certain parameters, 
            // you can use the pManager instance to access them by index:
            //pManager[0].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            // Use the pManager object to register your output parameters.
            // Output parameters do not have default values, but they too must have the correct access type.
            pManager.AddTextParameter("Result", "R", "Solve result", GH_ParamAccess.item);
            pManager.AddGenericParameter("Dots", "D", "Node label dots", GH_ParamAccess.list);
            pManager.AddCurveParameter("Curves", "C", "Element curves", GH_ParamAccess.list); 
            pManager.AddNumberParameter("Values", "V", "Scalar per element", GH_ParamAccess.list);
            pManager.AddCurveParameter("DispVectors", "DV", "Displacement vectors (scaled)", GH_ParamAccess.list);
            pManager.AddNumberParameter("NodeDispMag", "DN", "Node displacement magnitude", GH_ParamAccess.list);
            pManager.AddCurveParameter("DeformedCurves", "DC", "Deformed element curves (scaled)", GH_ParamAccess.list);



            // Sometimes you want to hide a specific parameter from the Rhino preview.
            // You can use the HideParameter() method as a quick way:
            //pManager.HideParameter(0);
        }



        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Curve> beams = new List<Curve>();
            List<Point3d> loads = new List<Point3d>();
            List<Point3d> supports = new List<Point3d>(); 
            int metric = 0;
            double deformScale = 2.0;



            // Then we need to access the input parameters individually. 
            // When data cannot be extracted from a parameter, we should abort this method.
            if (!DA.GetDataList<Curve>(0, beams)) return;
            if (!DA.GetDataList<Point3d>(1, loads)) return;
            if (!DA.GetDataList<Point3d>(2, supports)) return;
            if (!DA.GetData<int>(3, ref metric)) return;
            if (!DA.GetData<double>(4, ref deformScale)) return;



            /*
            Step1: Create Model, Members and Nodes.
            Step2: Add the Nodes and Elements to Model
            Step3: Assign geometrical and mechanical properties to Elements.
            Step4: Assign Constraints to Nodes (fix the DoF s).
            Step5: Assign Load to Node.
            */

            (string result,
             List<Curve> curves,
             List<double> values,
             List<TextDot> dots,
             List<Curve> dispVectors,
             List<double> nodeDispMag,
             List<Curve> deformedCurves) = SpatialAnalyse(beams, loads, supports, metric, deformScale);

            result = "Total reactions SUM :" + result;



            DA.SetData(0, result);
            DA.SetDataList(1, dots);
            DA.SetDataList(2, curves);
            DA.SetDataList(3, values);
            DA.SetDataList(4, dispVectors);
            DA.SetDataList(5, nodeDispMag);
            DA.SetDataList(6, deformedCurves);
        }

        static (string result,
                List<Curve> curves,
                List<double> values,
                List<TextDot> dots,
                List<Curve> dispVectors,
                List<double> nodeDispMag,
                List<Curve> deformedCurves)
                SpatialAnalyse(
                    List<Curve> beams,
                    List<Point3d> loads,
                    List<Point3d> supports,
                    int metric, double deformScale)
        {
            var model = new Model();

            var material = UniformIsotropicMaterial.CreateFromYoungPoisson(2.3e3, 0.33); // N/mm^2
            var section = MakeRectSection(b: 30.0, h: 30.0, jOverride: -1);            // mm

            // Node welding
            double tol = 0.5;// adjust to Rhino units
            var nodeByKey = new Dictionary<(long, long, long), Node>();

            (long, long, long) KeyFromPoint(Point3d p)
            {
                long kx = (long)Math.Round(p.X / tol);
                long ky = (long)Math.Round(p.Y / tol);
                long kz = (long)Math.Round(p.Z / tol);
                return (kx, ky, kz);
            }

            Node GetOrCreateNode(Point3d p, string label)
            {
                var key = KeyFromPoint(p);
                if (nodeByKey.TryGetValue(key, out var existing))
                    return existing;

                var n = new Node(p.X, p.Y, p.Z) { Label = label };
                nodeByKey[key] = n;
                model.Nodes.Add(n);
                return n;
            }

            // Build elements
            for (int i = 0; i < beams.Count; i++)
            {
                var a = beams[i].PointAtStart;
                var b = beams[i].PointAtEnd;

                var nStart = GetOrCreateNode(a, $"n{i}_start");
                var nEnd = GetOrCreateNode(b, $"n{i}_end");

                var e = new BarElement(nStart, nEnd)
                {
                    Label = $"e{i}",

                    // Frame behavior (bending + torsion).
                    Behavior = BarElementBehaviours.FullFrame,

                    Section = section,
                    Material = material,

                    // Set a consistent orientation about the bar axis
                    WebRotation = ComputeWebRotationDegrees(nStart.Location, nEnd.Location)
                };

                model.Elements.Add(e);
            }

            //find duplicates 
            var seen = new HashSet<((long, long, long), (long, long, long))>();
            var filtered = new List<BarElement>();

            foreach (BarElement e in model.Elements)
            {
                var k0 = KeyFromPoint(new Point3d(e.StartNode.Location.X, e.StartNode.Location.Y, e.StartNode.Location.Z));
                var k1 = KeyFromPoint(new Point3d(e.EndNode.Location.X, e.EndNode.Location.Y, e.EndNode.Location.Z));

                // undirected edge key
                var edge = (k0.CompareTo(k1) <= 0) ? (k0, k1) : (k1, k0);

                if (seen.Add(edge))
                    filtered.Add(e);
            }

            model.Elements.Clear();
            foreach (var e in filtered)
                model.Elements.Add(e);

            // Validate zero-length
            double minLen = 0.1; // mm

            foreach (BarElement e in model.Elements)
            {
                var a = e.StartNode.Location;
                var b = e.EndNode.Location;

                double dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z;
                double L = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                if (L < minLen)
                    throw new Exception($"Too-short element (<{minLen}mm): {e.Label}  L={L}");
            }

            // Default free
            foreach (var n in model.Nodes)
                n.Constraints = Constraints.Released;

            // Collect supports in a list so we can pick an anchor
            var supportList = new List<Node>();
            var supportSet = new HashSet<Node>();

            foreach (var sp in supports)
            {
                var key = KeyFromPoint(sp);
                if (!nodeByKey.TryGetValue(key, out var sn))
                    throw new Exception($"No model node found near support point {sp} using tol={tol}");

                // avoid duplicates
                if (supportSet.Add(sn))
                    supportList.Add(sn);
            }

            if (supportList.Count == 0)
                throw new Exception("No support nodes found (after welding).");

            // Anchor ONE support node as fully fixed (removes rigid-body spin modes)
            supportList[0].Constraints = Constraints.Fixed;

            // All other supports are pinned: translations fixed, rotations released
            for (int i = 1; i < supportList.Count; i++)
                supportList[i].Constraints = Constraints.MovementFixed;

            // If you still use your BFS check, use this supported set:
            var supportNodes = new HashSet<Node>(supportList);

            // Loads
            foreach (var lp in loads)
            {
                var key = KeyFromPoint(lp);
                if (!nodeByKey.TryGetValue(key, out var ln))
                    throw new Exception($"No model node found near load point {lp} using tol={tol}");

                ln.Loads.Add(new NodalLoad(new Force(0, 0, -2, 0, 0, 0)));
            }

            //Prove supports are matching nodes
            int matched = 0;
            foreach (var sp in supports)
            {
                if (nodeByKey.ContainsKey(KeyFromPoint(sp))) matched++;
            }
            System.Diagnostics.Debug.WriteLine($"Support points: {supports.Count}, matched nodes: {matched}");


            // Ensure each connected component has at least one support
            EnsureEveryComponentHasSupport(model, supportNodes);

            // Preview dots
            var dots = model.Nodes
                .Select(n => new TextDot(n.Label, new Point3d(n.Location.X, n.Location.Y, n.Location.Z)))
                .ToList();

            // Solve
            model.Solve_MPC();

            //node displacement
            var dispVectors = new List<Curve>();
            var nodeDispMag = new List<double>();
            var nodeDispVec = new Dictionary<Node, Vector3d>();

            foreach (var n in model.Nodes)
            {
                var d = n.GetNodalDisplacement(LoadCase.DefaultLoadCase);

                var v = new Vector3d(d.DX, d.DY, d.DZ);
                nodeDispVec[n] = v;
                nodeDispMag.Add(v.Length);

                var p0 = new Point3d(n.Location.X, n.Location.Y, n.Location.Z);
                var p1 = p0 + v * deformScale;          // deformScale passed into SpatialAnalyse
                dispVectors.Add(new LineCurve(p0, p1));
            }

            // Deformed member curves
            var deformedCurves = new List<Curve>();
            foreach (BarElement e in model.Elements)
            {
                var n0 = e.StartNode;
                var n1 = e.EndNode;

                var p0 = new Point3d(n0.Location.X, n0.Location.Y, n0.Location.Z) + nodeDispVec[n0] * deformScale;
                var p1 = new Point3d(n1.Location.X, n1.Location.Y, n1.Location.Z) + nodeDispVec[n1] * deformScale;

                deformedCurves.Add(new LineCurve(p0, p1));
            }


            var outCrvs = new List<Curve>();
            var values = new List<double>();

            foreach (BarElement e in model.Elements)
            {
                var a = e.StartNode.Location;
                var b = e.EndNode.Location;

                outCrvs.Add(new LineCurve(
                    new Point3d(a.X, a.Y, a.Z),
                    new Point3d(b.X, b.Y, b.Z)));

                Force f = e.GetInternalForceAt(0.0, LoadCase.DefaultLoadCase);

                double val;
                switch (metric)
                {
                    case 0: val = Math.Abs(f.Fx); break;                                 // Axial
                    case 1: val = Math.Sqrt(f.Fy * f.Fy + f.Fz * f.Fz); break;           // Shear
                    case 2: val = Math.Sqrt(f.My * f.My + f.Mz * f.Mz); break;           // Moment
                    case 3: val = Math.Abs(f.Mx); break;                                 // Torsion
                    default: val = Math.Sqrt(f.My * f.My + f.Mz * f.Mz); break;
                }

                values.Add(val);
            }

            Force sumR = Force.Zero;

            // Only sum on constrained (support) nodes
            foreach (var n in model.Nodes)
            {
                if (n.Constraints.Equals(Constraints.Released))
                    continue;

                sumR += n.GetSupportReaction(LoadCase.DefaultLoadCase);
            }

            // Optional: also compute sum of applied nodal loads to check equilibrium
            Force sumApplied = Force.Zero;
            foreach (var n in model.Nodes)
            {
                // Total applying forces are external nodal loads + equivalent elemental loads (not reactions)
                sumApplied += n.GetTotalApplyingForces(LoadCase.DefaultLoadCase);
            }

            // Build your output string
            string result =
                $"Support reactions sum: Fx={sumR.Fx:0.###}, Fy={sumR.Fy:0.###}, Fz={sumR.Fz:0.###}, " +
                $"Mx={sumR.Mx:0.###}, My={sumR.My:0.###}, Mz={sumR.Mz:0.###}\n" +
                $"Applied loads sum:      Fx={sumApplied.Fx:0.###}, Fy={sumApplied.Fy:0.###}, Fz={sumApplied.Fz:0.###}, " +
                $"Mx={sumApplied.Mx:0.###}, My={sumApplied.My:0.###}, Mz={sumApplied.Mz:0.###}";


            return (result, outCrvs, values, dots, dispVectors, nodeDispMag, deformedCurves);
        }

        public enum ColorMetric
        {
            Axial = 0,
            Shear = 1,
            BendingMoment = 2,
            Torsion = 3
        }

        static UniformGeometric1DSection MakeRectSection(double b, double h, double jOverride = -1)
        {
            double y = b / 2.0;
            double z = h / 2.0;

            var geom = new PointYZ[]
            {
                new PointYZ(-y, -z),
                new PointYZ( y, -z),
                new PointYZ( y,  z),
                new PointYZ(-y,  z),
                new PointYZ(-y, -z) // CLOSE the polygon (repeat first point)
            };

            return (jOverride > 0)
                ? new UniformGeometric1DSection(geom, jOverride)
                : new UniformGeometric1DSection(geom);
        }

        static double ComputeWebRotationDegrees(BfePoint a, BfePoint b)
        {
            // Goal: stable orientation for local Y/Z.


            var v = new Vector3d(b.X - a.X, b.Y - a.Y, b.Z - a.Z);
            if (!v.Unitize()) return 0.0;

            // "near vertical" means mostly Z
            double verticalness = Math.Abs(v.Z);
            if (verticalness > 0.9)
                return 90.0;

            return 0.0;
        }

        static void EnsureEveryComponentHasSupport(Model model, HashSet<Node> supportNodes)
        {
            // adjacency
            var adj = new Dictionary<Node, List<Node>>();
            foreach (var n in model.Nodes) adj[n] = new List<Node>();

            foreach (BarElement e in model.Elements)
            {
                adj[e.StartNode].Add(e.EndNode);
                adj[e.EndNode].Add(e.StartNode);
            }

            var visited = new HashSet<Node>();
            int compId = 0;

            foreach (var start in model.Nodes)
            {
                if (visited.Contains(start)) continue;

                compId++;
                var q = new Queue<Node>();
                q.Enqueue(start);
                visited.Add(start);

                bool hasSupport = supportNodes.Contains(start);
                int count = 0;

                while (q.Count > 0)
                {
                    var u = q.Dequeue();
                    count++;

                    if (supportNodes.Contains(u)) hasSupport = true;

                    foreach (var v in adj[u])
                        if (visited.Add(v)) q.Enqueue(v);
                }

                if (!hasSupport)
                    throw new Exception($"Component #{compId} (size {count}) has NO supports → singular.");
            }
        }

        static Color Lerp(Color a, Color b, double t)
        {
            t = Math.Max(0.0, Math.Min(1.0, t));
            int r = (int)Math.Round(a.R + (b.R - a.R) * t);
            int g = (int)Math.Round(a.G + (b.G - a.G) * t);
            int bl = (int)Math.Round(a.B + (b.B - a.B) * t);
            return Color.FromArgb(r, g, bl);
        }

        static double Remap01(double x, double min, double max)
        {
            if (max <= min) return 0.0;
            return (x - min) / (max - min);
        }




    /// <summary>
    /// The Exposure property controls where in the panel a component icon 
    /// will appear. There are seven possible locations (primary to septenary), 
    /// each of which can be combined with the GH_Exposure.obscure flag, which 
    /// ensures the component will only be visible on panel dropdowns.
    /// </summary>
    public override GH_Exposure Exposure => GH_Exposure.primary;

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon => null;

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("309fa0e4-0905-420f-b082-ffbd5fb307cf");
    }
}