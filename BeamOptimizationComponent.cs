using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace FEA_Beam_Solver
{
    public class BeamOptimizationComponent : GH_Component
    {
        public BeamOptimizationComponent()
          : base("Beam Optimize", "bOPT",
            "Heuristically add, move, and remove beam segments to reduce node displacement.",
            "FGAM", "FEA")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Beams", "B", "Connected beam/segment lines to optimize.", GH_ParamAccess.list);
            pManager.AddPointParameter("Loads", "L", "Load points that are matched to welded beam nodes.", GH_ParamAccess.list);
            pManager.AddPointParameter("Supports", "S", "Support points that are matched to welded beam nodes.", GH_ParamAccess.list);
            pManager.AddBrepParameter("Brep", "Br", "Optional Brep used to project moved nodes and color a force mesh.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Metric", "M", "0=Axial | 1=Shear | 2=BendingMoment | 3=Torsion", GH_ParamAccess.item, 2);
            pManager.AddNumberParameter("DeformScale", "DS", "Display scale factor for node displacement and deformed members.", GH_ParamAccess.item, 2.0);
            pManager.AddIntegerParameter("Iterations", "I", "Maximum optimization iterations.", GH_ParamAccess.item, 8);
            pManager.AddIntegerParameter("Add Count", "AC", "Target number of segment additions the optimizer should try to accept if they improve displacement.", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("Remove Count", "RC", "Target number of segment removals the optimizer should try to accept if they improve displacement.", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("Search Breadth", "SB", "How many high-priority candidates per action are explored each round. Higher values increase search space and solve time.", GH_ParamAccess.item, 6);
            pManager.AddNumberParameter("MoveFactor", "MF", "Fraction of the node displacement vector used when trying node moves.", GH_ParamAccess.item, 0.35);
            pManager.AddNumberParameter("AddThreshold", "AT", "Nodes above this fraction of the max displacement are candidates for adding new segments.", GH_ParamAccess.item, 0.75);
            pManager.AddNumberParameter("RemoveThreshold", "RT", "Segments below this fraction of the max force are candidates for removal.", GH_ParamAccess.item, 0.10);
            pManager.AddNumberParameter("MaxConnDist", "CD", "Maximum distance for added segments. Use 0 for an automatic value based on graph size.", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("WeldTol", "WT", "Tolerance for welding line endpoints into shared nodes.", GH_ParamAccess.item, 0.5);
            pManager[3].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Log", "L", "Optimization and analysis log.", GH_ParamAccess.item);
            pManager.AddCurveParameter("Optimized Beams", "OB", "Optimized beam geometry.", GH_ParamAccess.list);
            pManager.AddCurveParameter("Force Lines", "FL", "Beam lines associated with the reported force values.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Force Values", "FV", "Selected force metric per beam.", GH_ParamAccess.list);
            pManager.AddColourParameter("Force Colors", "FC", "Display colors for the beam force values.", GH_ParamAccess.list);
            pManager.AddCurveParameter("Disp Vectors", "DV", "Node displacement vectors.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Node Disp", "ND", "Node displacement magnitudes.", GH_ParamAccess.list);
            pManager.AddColourParameter("Disp Colors", "DC", "Display colors for node displacement magnitudes.", GH_ParamAccess.list);
            pManager.AddCurveParameter("Deformed Beams", "DB", "Deformed beam lines at the requested display scale.", GH_ParamAccess.list);
            pManager.AddMeshParameter("Force Mesh", "FM", "Optional Brep mesh colored by nearby beam force values.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Added", "A", "Number of accepted segment additions.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Removed", "R", "Number of accepted segment removals.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Moved", "Mv", "Number of accepted node move operations.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Disp Reduction", "DR", "Reduction in maximum node displacement between the initial and final analyses.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var beams = new List<Curve>();
            var loads = new List<Point3d>();
            var supports = new List<Point3d>();
            Brep brep = null;
            int metric = 2;
            double deformScale = 2.0;
            int iterations = 8;
            int addCount = 0;
            int removeCount = 0;
            int searchBreadth = 6;
            double moveFactor = 0.35;
            double addThreshold = 0.75;
            double removeThreshold = 0.10;
            double maxConnectionDistance = 0.0;
            double weldTolerance = 0.5;

            if (!DA.GetDataList(0, beams)) return;
            if (!DA.GetDataList(1, loads)) return;
            if (!DA.GetDataList(2, supports)) return;
            DA.GetData(3, ref brep);
            if (!DA.GetData(4, ref metric)) return;
            if (!DA.GetData(5, ref deformScale)) return;
            if (!DA.GetData(6, ref iterations)) return;
            if (!DA.GetData(7, ref addCount)) return;
            if (!DA.GetData(8, ref removeCount)) return;
            if (!DA.GetData(9, ref searchBreadth)) return;
            if (!DA.GetData(10, ref moveFactor)) return;
            if (!DA.GetData(11, ref addThreshold)) return;
            if (!DA.GetData(12, ref removeThreshold)) return;
            if (!DA.GetData(13, ref maxConnectionDistance)) return;
            if (!DA.GetData(14, ref weldTolerance)) return;

            try
            {
                var result = BeamOptimizationEngine.Optimize(
                    beams,
                    loads,
                    supports,
                    brep,
                    new BeamOptimizationOptions
                    {
                        Metric = metric,
                        DeformScale = deformScale,
                        Iterations = iterations,
                        TargetAddCount = addCount,
                        TargetRemoveCount = removeCount,
                        SearchBreadth = searchBreadth,
                        MoveFactor = moveFactor,
                        AddDisplacementThresholdRatio = addThreshold,
                        RemoveForceThresholdRatio = removeThreshold,
                        MaxConnectionDistance = maxConnectionDistance,
                        WeldTolerance = weldTolerance,
                    });

                List<Color> forceColors = BuildColors(result.FinalAnalysis.ElementValues);
                List<Color> displacementColors = BuildColors(result.FinalAnalysis.NodeDisplacementMagnitudes);

                DA.SetData(0, result.Log + Environment.NewLine + result.FinalAnalysis.Summary);
                DA.SetDataList(1, result.OptimizedGraph.ToCurves());
                DA.SetDataList(2, result.FinalAnalysis.ElementCurves);
                DA.SetDataList(3, result.FinalAnalysis.ElementValues);
                DA.SetDataList(4, forceColors);
                DA.SetDataList(5, result.FinalAnalysis.DisplacementVectors);
                DA.SetDataList(6, result.FinalAnalysis.NodeDisplacementMagnitudes);
                DA.SetDataList(7, displacementColors);
                DA.SetDataList(8, result.FinalAnalysis.DeformedCurves);
                DA.SetData(9, result.ForceMesh);
                DA.SetData(10, result.AddedCount);
                DA.SetData(11, result.RemovedCount);
                DA.SetData(12, result.MovedCount);
                DA.SetData(13, result.DisplacementReduction);
            }
            catch (Exception ex)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
            }
        }

        private static List<Color> BuildColors(IReadOnlyList<double> values)
        {
            double min = values.DefaultIfEmpty(0.0).Min();
            double max = values.DefaultIfEmpty(0.0).Max();
            return values
                .Select(value => BeamOptimizationEngine.MapGradient(BeamOptimizationEngine.Remap01(value, min, max)))
                .ToList();
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;

        protected override Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("76d5bf66-b4dd-4d2b-ae1f-2d9f4c6331e1");
    }
}
