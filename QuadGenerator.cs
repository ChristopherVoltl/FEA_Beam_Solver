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
using Rhino.Geometry.Collections;
using Rhino.Geometry.Intersect;
using System.Security.Cryptography;


namespace FEA_Beam_Solver
{
    public class QuadGenerator_Component : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public QuadGenerator_Component()
          : base("Quad Generator", "Qgen",
            "Generate Planar and non-planar quads for spatial extrusion",
            "FGAM", "MESH")
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
            pManager.AddBrepParameter("Brep", "B", "Brep to be quaded", GH_ParamAccess.item);
            pManager.AddNumberParameter("Size", "S", "Size of the quad", GH_ParamAccess.item);




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
            pManager.AddCurveParameter("Planar Quad Wires", "pQW", "Edges from Quad Meshing", GH_ParamAccess.list);
            pManager.AddCurveParameter("Non-Planar Quad Wires", "npQW", "Edges from Quad Meshing", GH_ParamAccess.list);




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
            Brep brep = new Brep();
            double size = 1.0;




            // Then we need to access the input parameters individually. 
            // When data cannot be extracted from a parameter, we should abort this method.
            if (!DA.GetData(0, ref brep)) return;
            if (!DA.GetData(1, ref size)) return;

            //planar logic
            List<Curve> planarDivisions = PlanarQuadWires(brep, size);


            //non-planar logic
            List<Curve> nonPlanarDivisions = NonPlanarQuadWires(brep, size);

            //Outputs
            DA.SetDataList(0, planarDivisions);
            DA.SetDataList(1, nonPlanarDivisions);
        }

        public List<Curve> PlanarQuadWires(Brep brep, double size)
        {
            //final planar quads
            List<Curve> planarQuadWires = new List<Curve>();

            //create a section division based on height of size
            BoundingBox bb = brep.GetBoundingBox(true);

            double height = bb.Max.Z - bb.Min.Z;

            int numSections = (int)(height / size) + 1;

            //divide brep into planar sections by size 
            for (int i = 0; i < numSections; i++)
            {
                double z = bb.Min.Z + i * size;
                Plane sectionPlane = new Plane(new Point3d(0, 0, z), Vector3d.ZAxis);

                Curve[] crvs;
                Point3d[] pts;
                bool ok = Intersection.BrepPlane(brep, sectionPlane, .001, out crvs, out pts);

                if (ok && crvs != null)
                    planarQuadWires.AddRange(crvs);
            }

            return planarQuadWires;

            

        }

        public List<Curve> NonPlanarQuadWires(Brep brep, double size)
        {
            List<Curve> nonPlanarQuadWires = new List<Curve>();
            List<Curve> verticalCrvs = new List<Curve>();


            //divide brep by the shortest vertical edge
            List<double> edgeLengths = new List<double>();

            //check if edge is vertical by comparing start and end points

            foreach (BrepEdge edge in brep.Edges)
            {
                //check if edge is vertical by comparing start and end points
                Point3d start = edge.PointAtStart;
                Point3d end = edge.PointAtEnd;

                if (Math.Abs(start.Z) < Math.Abs(end.Z))
                {
                    edgeLengths.Add(edge.GetLength());
                    verticalCrvs.Add(edge.EdgeCurve);
                } 
            }

            double minEdgeLength = edgeLengths.Min();

            //divide shortest edge by size to get number of divisions
            int numDivisions = (int)(minEdgeLength / size) + 1;

            //divide curves by number of divisions
            List<Curve> dividedCurves = new List<Curve>();


            List<Point3d> allParamPts = new List<Point3d> { };

            foreach (Curve curve in verticalCrvs)
            {
                double[] parameters = curve.DivideByCount(numDivisions, true);
                Point3d[] points = new Point3d[parameters.Length];
                
                for (int i = 0; i < parameters.Length; i++)
                {
                    
                    points[i] = curve.PointAt(parameters[i]);
                    allParamPts.Add(points[i]);

                }
            }

            //create section curves by connecting points on adjacent curves
            for (int i = 0; i < allParamPts.Count - numDivisions - 1; i++)
            {
                nonPlanarQuadWires.Add(new LineCurve(allParamPts[i], allParamPts[i + numDivisions + 1]));
            }
            //connect the last point to the first point to close the loop
            for (int i = 0; i < numDivisions + 1; i++)
            {
                nonPlanarQuadWires.Add(new LineCurve(allParamPts[i], allParamPts[i -1 + (allParamPts.Count - numDivisions)]));
            }


                var joined = Curve.JoinCurves(nonPlanarQuadWires);

            return joined?.ToList() ?? new List<Curve>();
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
        public override Guid ComponentGuid => new Guid("3e0645b4-7380-4e53-acf1-cb4141c08e64");
    }
}