using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Obi
{
    [AddComponentMenu("Physics/Obi/Obi Softbody Skinner", 931)]
    [ExecuteInEditMode]
    [RequireComponent(typeof(SkinnedMeshRenderer))]
    public class ObiSoftbodySkinner : MonoBehaviour
    {

        [Tooltip("The ratio at which the cluster's influence on a vertex falls off with distance.")]
        public float m_SkinningFalloff = 1.0f;
        [Tooltip("The maximum distance a cluster can be from a vertex before it will not influence it any more.")]
        public float m_SkinningMaxDistance = 0.5f;

        [HideInInspector] [SerializeField] List<Matrix4x4> m_Bindposes = new List<Matrix4x4>();
        [HideInInspector] [SerializeField] BoneWeight[] m_BoneWeights = new BoneWeight[0];

        private SkinnedMeshRenderer m_Target;
        private Mesh m_SoftMesh;                  /**< deformable mesh instance*/
        private List<Transform> m_Bones = new List<Transform>();

        [HideInInspector] [SerializeField] private bool m_Bound = false;

        [SerializeField] [HideInInspector] private ObiSoftbody m_Source;

        public ObiSoftbody Source
        {
            set
            {
                if (m_Source != value)
                {
                    if (m_Source != null)
                        m_Source.OnInterpolate += UpdateBones;

                    m_Source = value;

                    if (m_Source != null)
                        m_Source.OnInterpolate -= UpdateBones;
                }

            }
            get { return m_Source; }
        }

        public void Awake()
        {
            // autoinitialize "target" with the first skinned mesh renderer we find up our hierarchy.
            m_Target = GetComponent<SkinnedMeshRenderer>();

            // autoinitialize "source" with the first softbody we find up our hierarchy.
            if (Source == null)
                Source = GetComponentInParent<ObiSoftbody>();
            else
                Source.OnInterpolate += UpdateBones;

            CreateBones();
        }

        public void OnDestroy()
        {
            if (m_Source != null)
                m_Source.OnInterpolate -= UpdateBones;

            DestroyBones();
        }

        private void CreateBones()
        {
            if (Application.isPlaying)
            {
                // Setup the mesh from scratch, in case it is not a clone of an already setup mesh.
                if (m_SoftMesh == null)
                {
                    // Create a copy of the original mesh:
                    m_SoftMesh = Mesh.Instantiate(m_Target.sharedMesh);

                    // Unity bug workaround:
                    Vector3[] vertices = m_SoftMesh.vertices;
                    m_SoftMesh.vertices = vertices;

                    AppendBoneWeights();
                    AppendBindposes();
                    AppendBoneTransforms();
                }
                // Reuse the same mesh, just copy bone references as we need to update bones every frame.
                else
                {
                    m_SoftMesh = Mesh.Instantiate(m_SoftMesh);
                    CopyBoneTransforms();
                }

                // Set the new mesh:
                m_SoftMesh.RecalculateBounds();
                m_Target.sharedMesh = m_SoftMesh;

                // Recalculate bounds:
                m_Target.localBounds = m_SoftMesh.bounds;
                m_Target.rootBone = m_Source.transform;
            }
        }

        private void DestroyBones()
        {
            foreach (Transform t in m_Bones)
                if (t) Destroy(t.gameObject);

            m_Bones.Clear();

            if (m_SoftMesh)
                DestroyImmediate(m_SoftMesh);
        }

        private void UpdateBones(ObiActor actor)
        {
            if (m_Bones.Count > 0 && m_Source.solver != null)
	        {
                int boneIndex = 0;
                var constraints = m_Source.softbodyBlueprint.GetConstraintsByType(Oni.ConstraintType.ShapeMatching) as ObiConstraints<ObiShapeMatchingConstraintsBatch>;
                foreach (ObiShapeMatchingConstraintsBatch batch in constraints.GetBatchInterfaces())
                {
                    for (int i = 0; i < batch.activeConstraintCount; ++i, ++boneIndex)
                    {
                        // the first particle index in each shape is the "center" particle.
                        int shapeIndex = batch.particleIndices[batch.firstIndex[i]];
                        m_Bones[boneIndex].position = m_Source.GetParticlePosition(m_Source.solverIndices[shapeIndex]);
                        m_Bones[boneIndex].rotation = m_Source.GetParticleOrientation(m_Source.solverIndices[shapeIndex]);
                    }
                }
	        }
        }

        public IEnumerator BindSkin()
        {
            m_Bound = false;

            if (m_Source == null || m_Source.solver == null || m_Target.sharedMesh == null){
				yield break;
			}			

            var constraints = m_Source.softbodyBlueprint.GetConstraintsByType(Oni.ConstraintType.ShapeMatching) as ObiConstraints<ObiShapeMatchingConstraintsBatch>;
            int boneCount = constraints.GetActiveConstraintCount();

            var clusterCenters = new List<Vector3>(boneCount);
            var clusterOrientations = new List<Quaternion>(boneCount);
	
            Vector3[] vertices = m_Target.sharedMesh.vertices;
            m_Bindposes.Clear();
            m_Bindposes.Capacity = boneCount;
			m_BoneWeights = new BoneWeight[vertices.Length];

			// Calculate softbody local to world matrix, and target to world matrix.
            Matrix4x4 source2w = m_Source.transform.localToWorldMatrix;
            Quaternion source2wRot = source2w.rotation;
			Matrix4x4 target2w = transform.localToWorldMatrix;

            foreach (ObiShapeMatchingConstraintsBatch batch in constraints.GetBatchInterfaces())
            {
                // Create bind pose matrices, one per shape matching cluster:
                for (int i = 0; i < batch.activeConstraintCount; ++i)
                {
                    int shapeIndex = batch.particleIndices[batch.firstIndex[i]];

                    Vector3 clusterCenter = source2w.MultiplyPoint3x4(m_Source.softbodyBlueprint.restPositions[shapeIndex]);
                    Quaternion clusterOrientation = source2wRot * m_Source.softbodyBlueprint.restOrientations[shapeIndex];

                    // world space bone center/orientation:
                    clusterCenters.Add(clusterCenter);
                    clusterOrientations.Add(clusterOrientation);

                    // world space bone transform * object local to world.
                    m_Bindposes.Add(Matrix4x4.TRS(clusterCenter, clusterOrientation, Vector3.one).inverse * target2w);

                    yield return new CoroutineJob.ProgressInfo("ObiSoftbody: calculating bind poses...", i / (float)batch.activeConstraintCount);
                }
            }
	
			// Calculate skin weights and bone indices:
			for (int j = 0; j < m_BoneWeights.Length; ++j)
			{
				// transform each vertex to world space:
				m_BoneWeights[j] = CalculateBoneWeights(target2w.MultiplyPoint3x4(vertices[j]),clusterCenters);
				yield return new CoroutineJob.ProgressInfo("ObiSoftbody: calculating bone weights...",j/(float)m_BoneWeights.Length);
			}

            m_Bound = true;
        }

        private BoneWeight CalculateBoneWeights(Vector3 vertex, List<Vector3> clusterCenters)
        {

            BoneWeight w = new BoneWeight();

            w.boneIndex0 = -1;
            w.boneIndex1 = -1;
            w.boneIndex2 = -1;
            w.boneIndex3 = -1;

            for (int i = 0; i < clusterCenters.Count; ++i)
            {
                float distance = Vector3.Distance(vertex, clusterCenters[i]);

                if (distance <= m_SkinningMaxDistance)
                {

                    float weight = distance > 0 ? m_SkinningMaxDistance / distance : 100;
                    weight = Mathf.Pow(weight, m_SkinningFalloff);

                    if (weight > w.weight0)
                    {
                        w.weight0 = weight;
                        w.boneIndex0 = i;
                    }
                    else if (weight > w.weight1)
                    {
                        w.weight1 = weight;
                        w.boneIndex1 = i;
                    }
                    else if (weight > w.weight2)
                    {
                        w.weight2 = weight;
                        w.boneIndex2 = i;
                    }
                    else if (weight > w.weight3)
                    {
                        w.weight3 = weight;
                        w.boneIndex3 = i;
                    }
                }
            }

            NormalizeBoneWeight(ref w);
            return w;
        }

        private void NormalizeBoneWeight(ref BoneWeight w)
        {

            float sum = w.weight0 + w.weight1 + w.weight2 + w.weight3;
            if (sum > 0)
            {
                w.weight0 /= sum;
                w.weight1 /= sum;
                w.weight2 /= sum;
                w.weight3 /= sum;
            }
        }

        private void AppendBindposes()
        {
            List<Matrix4x4> bindposes = new List<Matrix4x4>(m_SoftMesh.bindposes);
            bindposes.AddRange(m_Bindposes);
            m_SoftMesh.bindposes = bindposes.ToArray();
        }

        private void AppendBoneWeights()
        {

            int bonesOffset = m_SoftMesh.bindposes.Length;
            BoneWeight[] boneWeights = m_SoftMesh.boneWeights.Length > 0 ? m_SoftMesh.boneWeights : new BoneWeight[m_BoneWeights.Length];

            for (int i = 0; i < boneWeights.Length; ++i)
            {
                // If no soft skinning could be performed for this vertex, leave original skin data untouched:
                if (m_BoneWeights[i].boneIndex0 == -1 && m_BoneWeights[i].boneIndex1 == -1 &&
                    m_BoneWeights[i].boneIndex2 == -1 && m_BoneWeights[i].boneIndex3 == -1)
                    continue;

                // Copy bone weights (adding required offsets):
                boneWeights[i].boneIndex0 = Mathf.Max(0, m_BoneWeights[i].boneIndex0) + bonesOffset;
                boneWeights[i].weight0 = m_BoneWeights[i].weight0;
                boneWeights[i].boneIndex1 = Mathf.Max(0, m_BoneWeights[i].boneIndex1) + bonesOffset;
                boneWeights[i].weight1 = m_BoneWeights[i].weight1;
                boneWeights[i].boneIndex2 = Mathf.Max(0, m_BoneWeights[i].boneIndex2) + bonesOffset;
                boneWeights[i].weight2 = m_BoneWeights[i].weight2;
                boneWeights[i].boneIndex3 = Mathf.Max(0, m_BoneWeights[i].boneIndex3) + bonesOffset;
                boneWeights[i].weight3 = m_BoneWeights[i].weight3;
            }

            m_SoftMesh.boneWeights = boneWeights;
        }

        private void AppendBoneTransforms()
        {

            // Calculate softbody local to world matrix, and target to world matrix.
            Matrix4x4 source2w = m_Source.transform.localToWorldMatrix;
            Quaternion source2wRot = source2w.rotation;

            m_Bones.Clear();

            var constraints = m_Source.softbodyBlueprint.GetConstraintsByType(Oni.ConstraintType.ShapeMatching) as ObiConstraints<ObiShapeMatchingConstraintsBatch>;
            foreach (ObiShapeMatchingConstraintsBatch batch in constraints.GetBatchInterfaces())
            {
                for (int i = 0; i < batch.activeConstraintCount; ++i)
                {
                    int shapeIndex = batch.particleIndices[batch.firstIndex[i]];

                    GameObject bone = new GameObject("Cluster" + i);
                    bone.transform.parent = transform;
                    bone.transform.position = source2w.MultiplyPoint3x4(m_Source.softbodyBlueprint.restPositions[shapeIndex]);
                    bone.transform.rotation = source2wRot * m_Source.softbodyBlueprint.restOrientations[shapeIndex];
                    bone.hideFlags = HideFlags.DontSave | HideFlags.HideInHierarchy;
                    m_Bones.Add(bone.transform);
                }
            }

            // append our bone list to the original one:
            List<Transform> originalBones = new List<Transform>(m_Target.bones);
            originalBones.AddRange(m_Bones);
            m_Target.bones = originalBones.ToArray();
        }

        private void CopyBoneTransforms()
        {
            var constraints = m_Source.softbodyBlueprint.GetConstraintsByType(Oni.ConstraintType.ShapeMatching) as ObiConstraints<ObiShapeMatchingConstraintsBatch>;
            int boneCount = constraints.GetActiveConstraintCount();

            m_Bones.Clear();
            m_Bones.Capacity = boneCount;

            for (int i = m_Target.bones.Length - boneCount, j = 0; i < m_Target.bones.Length && j < boneCount; ++i, ++j)
                m_Bones[j] = m_Target.bones[i];
        }

       
    }
}
