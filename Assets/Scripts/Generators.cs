using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using Obi;
using UnityEngine;

static class Generators
{
    public static ObiRope Rope(List<Vector3> points, 
                                Material material,
                                int collider_filter,
                                int collider_filter_end,
                                float rope_radius=0.01f, 
                                float resolution=1.0f, 
                                int pooled_particles=0, 
                                string name="", 
                                float stretch_compliance=0.00f,
                                float stretching_scale=0.99f,
                                float bend_compliance=0.00f,
                                float max_bending=0.015f,
                                float mass=0.1f,
                                float damping=0.0f,
                                float substeps=1,
                                bool self_collision=true) {
        // CREATING A SOLVER
        // create an object containing both the solver and the updater:
        GameObject solverObject = GameObject.Find("Obi Solver"+name);
        if (solverObject==null)
            solverObject = new GameObject("Obi Solver"+name, typeof(ObiSolver));
        ObiSolver solver = solverObject.GetComponent<ObiSolver>();
        // Configure the solver
        solver.gravity = new Vector3(0.0f, -9.81f, 0.0f); // Vector3.zero;
        solver.parameters.damping = damping;
        solver.parameters.surfaceCollisionIterations = 4;
        solver.PushSolverParameters();
        // Configure the updater parameters.
        // ObiFixedUpdater updater = solverObject.GetComponent<ObiFixedUpdater>();
        // updater.substeps = 1;
        // updater.solvers.Add(solver); // add the solver to the updater:

        // CREATING A BLUPRINT
        // create the blueprint: (ltObiRopeBlueprint, ObiRodBlueprint)
        ObiRopeBlueprint ropeBlueprint = ScriptableObject.CreateInstance<ObiRopeBlueprint>();
        ropeBlueprint.thickness = rope_radius; // radius of the particles (metres)
        ropeBlueprint.resolution = resolution;
        ropeBlueprint.pooledParticles = pooled_particles;

        // AddControlPoint: position, inTangentVector, outTangentVector, normal, mass, rotationalMass, thickness, filter, color, name
        // obi rod use 'normal' to define the orientation at each control point. not used in ropes
        // first point does not need in anchor
        // last point does not need out anchor
        Vector3 temp = (points[1]-points[0])/2;
        ropeBlueprint.path.Clear();
        // int mask = (1 << 0) | (1 << 1); //  | (1 << 3) | (1 << 6)
        // int end_filter = ObiUtils.MakeFilter(mask, 1);
        // int filter = ObiUtils.MakeFilter(ObiUtils.CollideWithEverything, 0);
        ropeBlueprint.path.AddControlPoint(points[0], Vector3.one, temp, Vector3.one, mass, mass, 1, collider_filter_end, Color.white, "0");
        if (points.Count >= 3)
            for (int i=1; i<points.Count-1; i++) {
                Vector3 inTan = (points[i-1]-points[i])/2;
                Vector3 outTan = (points[i+1]-points[i])/2;            
                ropeBlueprint.path.AddControlPoint(points[i], inTan, outTan, Vector3.one, mass, mass, 1, collider_filter, Color.white, i.ToString());
            }
        temp = (points[points.Count-2]-points[points.Count-1])/2;
        ropeBlueprint.path.AddControlPoint(points[points.Count-1], temp, Vector3.one, Vector3.one, mass, mass, 1, collider_filter_end, Color.white, (points.Count-1).ToString());
        ropeBlueprint.path.FlushEvents();

        // generate the particle representation of the rope (wait until it has finished):
        ropeBlueprint.Generate();

        // CREATING AN ACTOR
        GameObject ropeObject = new GameObject("Obi Rope", typeof(ObiRope), typeof(ObiRopeExtrudedRenderer)); // create a rope:
        // GameObject ropeObject = new GameObject("Obi Rope", typeof(ObiRope), typeof(ObiRopeLineRenderer)); // create a rope:
        
        ObiRope rope = ropeObject.GetComponent<ObiRope>(); // get component references:
        rope.selfCollisions = self_collision;
        rope.surfaceCollisions = true;
        rope.stretchingScale = stretching_scale;
        rope.stretchCompliance = stretch_compliance;
        rope.bendCompliance = bend_compliance;
        rope.maxBending = max_bending;

        ObiPathSmoother path_smoother = ropeObject.GetComponent<ObiPathSmoother>();
        // path_smoother.decimation = 0.15f;
        path_smoother.smoothing = 2;

        ObiRopeExtrudedRenderer ropeRenderer = ropeObject.GetComponent<ObiRopeExtrudedRenderer>();        
        ropeRenderer.section = Resources.Load<ObiRopeSection>("DefaultRopeSection"); // load the default rope section:
        ropeRenderer.thicknessScale = 1.0f;
        ropeRenderer.uvScale = new Vector2(1,2);

        // ObiRopeLineRenderer ropeRenderer = ropeObject.GetComponent<ObiRopeLineRenderer>();        
        // ropeRenderer.thicknessScale = 0.5f;
        // ropeRenderer.uvScale = new Vector2(1,5);

        // ObiParticleRenderer particleRenderer = ropeObject.GetComponent<ObiParticleRenderer>();
        // particleRenderer.shader = Shader.Find("Obi/Particles");

        rope.ropeBlueprint = ScriptableObject.Instantiate(ropeBlueprint); // instantiate and set the blueprint:
        rope.transform.parent = solver.transform; // parent the cloth under a solver to start simulation:

        // CHANGE ROPE MATERIAL
        MeshRenderer meshRenderer = ropeObject.GetComponent<MeshRenderer>();
        meshRenderer.material = material;
        // meshRenderer.material.color = Color.white;

        // ropeObject.transform.localScale = new Vector3(0.5f,0.5f,0.5f);
        
        return rope;
    }
}