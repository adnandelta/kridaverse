using UnityEngine;
using System.Collections;

namespace Obi
{
    [AddComponentMenu("Physics/Obi/Obi Softbody", 930)]
    public class ObiSoftbody : ObiActor
    {
        [SerializeField] protected ObiSoftbodyBlueprintBase m_SoftbodyBlueprint;

        [SerializeField] protected bool m_SelfCollisions = false;

        [SerializeField] [HideInInspector] private int centerBatch = -1;
        [SerializeField] [HideInInspector] private int centerShape = -1;

        // shape matching constraints:
        [SerializeField] protected bool _shapeMatchingConstraintsEnabled = true;
        [SerializeField] [Range(0, 1)] protected float _deformationResistance = 1;
        [SerializeField] [Range(0, 1)] protected float _maxDeformation = 0;
        [SerializeField] protected float _plasticYield = 0;
        [SerializeField] protected float _plasticCreep = 0;
        [SerializeField] protected float _plasticRecovery = 0;

        public override ObiActorBlueprint blueprint
        {
            get { return m_SoftbodyBlueprint; }
        }

        public ObiSoftbodyBlueprintBase softbodyBlueprint
        {
            get { return m_SoftbodyBlueprint; }
            set
            {
                if (m_SoftbodyBlueprint != value)
                {
                    RemoveFromSolver();
                    ClearState();
                    m_SoftbodyBlueprint = value;
                    AddToSolver();
                }
            }
        }

        public bool selfCollisions
        {
            get { return m_SelfCollisions; }
            set { if (value != m_SelfCollisions) { m_SelfCollisions = value; SetSelfCollisions(selfCollisions); } }
        }

        public override bool usesAnisotropicParticles
        {
            get { return true; }
        }

        protected override void OnValidate()
        {
            base.OnValidate();
            SetupRuntimeConstraints();
        }

        private void SetupRuntimeConstraints()
        {
            PushShapeMatchingConstraints(_shapeMatchingConstraintsEnabled,
                                         _deformationResistance,
                                         _plasticYield,
                                         _plasticCreep,
                                         _plasticRecovery,
                                         _maxDeformation);
            SetSelfCollisions(m_SelfCollisions);
        }

        public override void LoadBlueprint(ObiSolver solver)
        {
            base.LoadBlueprint(solver);
            SetupRuntimeConstraints();

            RecalculateRestShapeMatching();
            RecalculateCenterShape();
        }

        public void RecalculateRestShapeMatching()
        {
            if (Application.isPlaying && isLoaded)
            {
                var sc = GetConstraintsByType(Oni.ConstraintType.ShapeMatching) as ObiConstraints<ObiShapeMatchingConstraintsBatch>;

                foreach (var batch in sc.batches)
                    Oni.CalculateRestShapeMatching(solver.OniSolver, batch.oniBatch);
            }
        }

        /**
         * Recalculates the shape used as reference for transform position/orientation when there are no fixed particles.
         * Should be called manually when changing the amount of fixed particles and/or active particles.
         */
        public void RecalculateCenterShape()
        {

            centerShape = -1;
            centerBatch = -1;

            if (Application.isPlaying && isLoaded)
            {

                for (int i = 0; i < solverIndices.Length; ++i)
                {
                    if (solver.invMasses[solverIndices[i]] <= 0)
                        return;
                }

                var sc = m_SoftbodyBlueprint.GetConstraintsByType(Oni.ConstraintType.ShapeMatching) as ObiConstraints<ObiShapeMatchingConstraintsBatch>;

                // Get the particle whose center is closest to the actor's center (in mesh space)
                float minDistance = float.MaxValue;
                int j = 0;
                foreach (ObiShapeMatchingConstraintsBatch batch in sc.GetBatchInterfaces())
                {
                    for (int i = 0; i < batch.activeConstraintCount; ++i)
                    {
                        float dist = m_SoftbodyBlueprint.positions[batch.particleIndices[batch.firstIndex[i]]].sqrMagnitude;

                        if (dist < minDistance)
                        {
                            minDistance = dist;
                            centerShape = i;
                            centerBatch = j;
                        }
                    }
                    j++;
                }
            }
        }

        public override void UpdateParticleProperties()
        {
            RecalculateRestShapeMatching();
            RecalculateCenterShape();
        }

        public override void Interpolate()
        {
            var sc = GetConstraintsByType(Oni.ConstraintType.ShapeMatching) as ObiConstraints<ObiShapeMatchingConstraintsBatch>;

            if (Application.isPlaying && isActiveAndEnabled && centerBatch > -1 && centerBatch < sc.batches.Count)
            {
                var batch = sc.batches[centerBatch] as ObiShapeMatchingConstraintsBatch;
                if ( centerShape > -1 && centerShape < batch.activeConstraintCount)
                {
                    transform.position = (Vector3)batch.coms[centerShape] - batch.orientations[centerShape] * batch.restComs[centerShape];
                    transform.rotation = batch.orientations[centerShape];
                }
            }

            SetSelfCollisions(selfCollisions);

            base.Interpolate();
        }

    }

}