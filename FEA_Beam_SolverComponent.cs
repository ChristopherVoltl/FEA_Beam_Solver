using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using BriefFiniteElementNet;
using BriefFiniteElementNet.Elements;
using BriefFiniteElementNet.Materials;
using BriefFiniteElementNet.Sections;
using static System.Collections.Specialized.BitVector32;

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
          : base("FEA_Beam_SolverComponent", "ASpi",
            "Construct an Archimedean, or arithmetic, spiral given its radii and number of turns.",
            "Curve", "Primitive")
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
            pManager.AddLineParameter("Beams", "B", "Input beams of the Structure", GH_ParamAccess.list);


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
            pManager.AddCurveParameter("Spiral", "S", "Spiral curve", GH_ParamAccess.item);

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
            List<Line> beam = new List<Line>();


            // Then we need to access the input parameters individually. 
            // When data cannot be extracted from a parameter, we should abort this method.
            if (!DA.GetDataList<Line>(0, beam)) return;

            DA.SetDataList(0, beam);

            /*
            Step1: Create Model, Members and Nodes.
            Step2: Add the Nodes and Elements to Model
            Step3: Assign geometrical and mechanical properties to Elements.
            Step4: Assign Constraints to Nodes (fix the DoF s).
            Step5: Assign Load to Node.
            */

        }


         static void SpatialAnalyse(List<Line> beam)
         {
            // Initiating Model, Nodes and Members
            var model = new Model();

            //create nodes from the beam elements
            var n1 = new Node(1, 1, 0);
            n1.Label = "n1";//Set a unique label for node
            var n2 = new Node(-1, 1, 0) { Label = "n2" };//using object initializer for assigning Label
            var n3 = new Node(1, -1, 0) { Label = "n3" };
            var n4 = new Node(-1, -1, 0) { Label = "n4" };
            var n5 = new Node(0, 0, 1) { Label = "n5" };


            //create the elements
            var e1 = new BarElement(n1, n5) { Label = "e1", Behavior = BarElementBehaviours.Truss };
            var e2 = new BarElement(n2, n5) { Label = "e2", Behavior = BarElementBehaviours.Truss };
            var e3 = new BarElement(n3, n5) { Label = "e3", Behavior = BarElementBehaviours.Truss };
            var e4 = new BarElement(n4, n5) { Label = "e4", Behavior = BarElementBehaviours.Truss };

            //add node and elements to the model
            model.Nodes.Add(n1, n2, n3, n4, n5);
            model.Elements.Add(e1, e2, e3, e4);

            //Assign geometrical and mechanical properties to Elements
            e1.Section = new UniformParametric1DSection() { A = 9e-4 };
            e2.Section = new UniformParametric1DSection() { A = 9e-4 };
            e3.Section = new UniformParametric1DSection() { A = 9e-4 };
            e4.Section = new UniformParametric1DSection() { A = 9e-4 };

            e1.Material = UniformIsotropicMaterial.CreateFromYoungPoisson(210e9, 0.3);
            e2.Material = UniformIsotropicMaterial.CreateFromYoungPoisson(210e9, 0.3);
            e3.Material = UniformIsotropicMaterial.CreateFromYoungPoisson(210e9, 0.3);
            e4.Material = UniformIsotropicMaterial.CreateFromYoungPoisson(210e9, 0.3);

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