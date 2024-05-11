using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace Obi
{

    [CreateAssetMenu(fileName = "softbody surface blueprint", menuName = "Obi/Softbody Surface Blueprint", order = 160)]
    public class ObiSoftbodySurfaceBlueprint : ObiSoftbodyBlueprintBase
    {
        [Tooltip("Radius of particles. Use small values to create a finer representation of the mesh (max 1 particle per vertex), and large values to create a rough representation.")]
        public float particleRadius = 0.1f;         /**< Radius of particles.*/
        [Range(0, 0.75f)]
        [Tooltip("Percentage of overlap allowed between particles.")]
        public float particleOverlap = 0.2f;        /**< Percentage of overlap allowed for particles. A value of zero will not allow particles to overlap. Using values between 0.2 and 0.5 is recommended for good surface coverage.*/

        [Range(0, 1)]
        [Tooltip("Amount of shape smoothing applied before generating particles.")]
        public float shapeSmoothing = 0.5f;

        [Tooltip("Radius around each particle used to calculate their anisotropy.")]
        public float anisotropyNeighborhood = 0.2f; /**< Neighborhood around each particle. Used to calculate their anisotropy (size and orientation).*/

        [Tooltip("Maximum aspect ratio allowed for particles. High values will allow particles to deform more to better fit their neighborhood.")]
        public float maxAnisotropy = 3;             /**< Maximum particle anisotropy. High values will allow particles to deform to better fit their neighborhood.*/

        [Tooltip("Size of shape matching clusters. Large radii will include more particles in each cluster, making the softbody more rigid and increasing computational cost. If parts of the softbody are detaching, increase the radius so that they are included in at least one cluster.")]
        public float softClusterRadius = 0.3f;      /**< Size of clusters. Particles belonging to the same cluster are linked together by a shape matching constraint.*/

        [Tooltip("Generates one-sided particles instead of round ones. This results in better penetration recovery for convex objects.")]
        public bool oneSided = false;

        [HideInInspector] public Mesh generatedMesh = null;
        [HideInInspector] public int[] vertexToParticle = null;           /**< How much mesh surface area each particle represents.*/

        public const float DEFAULT_PARTICLE_MASS = 0.1f;

        protected override IEnumerator Initialize()
        {

            if (inputMesh == null || !inputMesh.isReadable)
            {
                // TODO: return an error in the coroutine.
                Debug.LogError("The input mesh is null, or not readable.");
                yield break;
            }

            ClearParticleGroups();

            Vector3[] vertices = inputMesh.vertices;
            Vector3[] normals = inputMesh.normals;
            vertexToParticle = new int[vertices.Length];
            List<Vector3> particles = new List<Vector3>();

            // Add particles to every vertex, as long as they are not too close to the already added ones:
            for (int i = 0; i < vertices.Length; ++i)
            {

                bool intersects = false;
                Vector3 vertexScaled = Vector3.Scale(scale, vertices[i]);

                for (int j = 0; j < particles.Count; ++j)
                {
                    if (Vector3.Distance(vertexScaled, particles[j]) < particleRadius * 2 * (1 - particleOverlap))
                    {
                        intersects = true; 
                        break;
                    }
                }
                if (intersects) continue; 

                particles.Add(vertexScaled);

                if (i % 100 == 0)
                    yield return new CoroutineJob.ProgressInfo("ObiSoftbody: sampling mesh...", i / (float)vertices.Length);
            }

            // Find out the closes particle to each vertex:
            for (int i = 0; i < vertices.Length; ++i)
            {
                Vector3 vertexScaled = Vector3.Scale(scale, vertices[i]);
                float minDistance = float.MaxValue;
                vertexToParticle[i] = 0;

                for (int j = 0; j < particles.Count; ++j)
                {
                    float distance = Vector3.SqrMagnitude(vertexScaled - particles[j]);
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        vertexToParticle[i] = j;
                    }
                }

                if (i % 100 == 0)
                    yield return new CoroutineJob.ProgressInfo("ObiSoftbody: mapping vertices to particles...", i / (float)vertices.Length);
            }

            positions = new Vector3[particles.Count];
            orientations = new Quaternion[particles.Count];
            restPositions = new Vector4[particles.Count];
            restOrientations = new Quaternion[particles.Count];
            velocities = new Vector3[particles.Count];
            angularVelocities = new Vector3[particles.Count];
            invMasses = new float[particles.Count];
            invRotationalMasses = new float[particles.Count];
            principalRadii = new Vector3[particles.Count];
            phases = new int[particles.Count];
            colors = new Color[particles.Count];

            m_ActiveParticleCount = particles.Count;

            for (int i = 0; i < particles.Count; ++i)
            {

                // Perform ellipsoid fitting:
                Vector3 avgNormal = Vector3.zero;
                List<Vector3> neighbourVertices = new List<Vector3>();

                for (int j = 0; j < vertices.Length; ++j)
                {

                    Vector3 vertexScaled = Vector3.Scale(scale,vertices[j]);

                    if (Vector3.Distance(vertexScaled, particles[i]) < anisotropyNeighborhood)
                    {
                        neighbourVertices.Add(vertexScaled);
                        avgNormal += normals[j];
                    }
                }
                if (neighbourVertices.Count > 0)
                    avgNormal /= neighbourVertices.Count;

                Vector3 centroid = particles[i];
                Quaternion orientation = Quaternion.identity;
                Vector3 principalValues = Vector3.one;
                Oni.GetPointCloudAnisotropy(neighbourVertices.ToArray(), neighbourVertices.Count, maxAnisotropy, particleRadius, ref avgNormal, ref centroid, ref orientation, ref principalValues);

                invRotationalMasses[i] = invMasses[i] = 1.0f; 
                positions[i] = Vector3.Lerp(particles[i], centroid, shapeSmoothing);
                restPositions[i] = positions[i];
                restPositions[i][3] = 1; // activate rest position.
                orientations[i] = orientation;
                restOrientations[i] = orientation;
                principalRadii[i] = principalValues;
                phases[i] = Oni.MakePhase(1, oneSided ? Oni.ParticleFlags.OneSided : 0); 
                colors[i] = Color.white;

                if (i % 100 == 0)
                    yield return new CoroutineJob.ProgressInfo("ObiSoftbody: generating particles...", i / (float)particles.Count);
            }


            //if (makeSolidCluster){
            /*indices.Clear();
            for (int i = 0; i < particles.Count; ++i)
                indices.Add(i);
            shapeBatch.AddConstraint(indices.ToArray(),1,0,0,true);
            }*/

            IEnumerator sc = CreateShapeMatchingConstraints(particles);

            while (sc.MoveNext())
                yield return sc.Current;
            
            // Initialize pin constraints:
            pinConstraintsData = new ObiPinConstraintsData();
            ObiPinConstraintsBatch pinBatch = new ObiPinConstraintsBatch();
            pinConstraintsData.AddBatch(pinBatch);

            generatedMesh = inputMesh;

        }

        protected override void SwapWithFirstInactiveParticle(int index)
        {
            base.SwapWithFirstInactiveParticle(index);

            // Keep vertexToParticle map in sync:
            for (int i = 0; i < vertexToParticle.Length; ++i)
            {
                if (vertexToParticle[i] == index)
                    vertexToParticle[i] = m_ActiveParticleCount;
                else if (vertexToParticle[i] == m_ActiveParticleCount)
                    vertexToParticle[i] = index;
            }
        }

        protected virtual IEnumerator CreateShapeMatchingConstraints(List<Vector3> particles)
        {
            //Create shape matching clusters:
            shapeMatchingConstraintsData = new ObiShapeMatchingConstraintsData();

            List<int> indices = new List<int>();

            List<int> particleIndices = new List<int>();
            List<int> constraintIndices = new List<int>();

            for (int i = 0; i < particles.Count; ++i)
            {
                constraintIndices.Add(particleIndices.Count);
                particleIndices.Add(i);

                for (int j = 0; j < particles.Count; ++j)
                {
                    if (i != j && Vector3.Distance(particles[j], particles[i]) < softClusterRadius)
                    {
                        particleIndices.Add(j);
                    }
                }

                if (i % 500 == 0)
                    yield return new CoroutineJob.ProgressInfo("ObiSoftbody: generating shape matching constraints...", i / (float)particles.Count);
            }

            constraintIndices.Add(particleIndices.Count);

            // pass a copy of the particleIndices array, as we need to preserve particle order (first particle in each constraint is the center particle)
            int[] constraintColors = GraphColoring.Colorize(particleIndices.ToArray(), constraintIndices.ToArray());

            for (int i = 0; i < constraintColors.Length; ++i)
            {
                int color = constraintColors[i];
                int cIndex = constraintIndices[i];

                // Add a new batch if needed:
                if (color >= shapeMatchingConstraintsData.GetBatchCount())
                    shapeMatchingConstraintsData.AddBatch(new ObiShapeMatchingConstraintsBatch());

                int amount = constraintIndices[i + 1] - cIndex;
                int[] clusterIndices = new int[amount];
                for (int j = 0; j < amount; ++j)
                    clusterIndices[j] = particleIndices[cIndex + j];

                shapeMatchingConstraintsData.batches[color].AddConstraint(clusterIndices,false);
            }

            // Set initial amount of active constraints:
            for (int i = 0; i < shapeMatchingConstraintsData.batches.Count; ++i)
            {
                shapeMatchingConstraintsData.batches[i].activeConstraintCount = shapeMatchingConstraintsData.batches[i].constraintCount;
            }
        }
    }
}