using Grasshopper;
using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace FEA_Beam_Solver
{
    public class FEA_Beam_SolverInfo : GH_AssemblyInfo
    {
        public override string Name => "FEA_Beam_Solver";

        //Return a 24x24 pixel bitmap to represent this GHA library.
        public override Bitmap Icon => null;

        //Return a short string describing the purpose of this GHA library.
        public override string Description => "";

        public override Guid Id => new Guid("7ceccd1b-c251-4cb0-9bfc-82dcfaddf725");

        //Return a string identifying you or your company.
        public override string AuthorName => "";

        //Return a string representing your preferred contact details.
        public override string AuthorContact => "";
    }
}