using UnityEngine;

namespace K3.Mech.IK
{

    /// <summary>
    /// Some very special properties of this system:
    /// - Each joint only has one degree of freedom, that being a rotation around one of its axes
    /// - There is exactly one joint that handles YAW and it is the topmost one.
    /// - All the other joints, all the way up until the tip, handle other rotation in the same axis
    /// - The last bone in the chain, the "foot", is very short and its desired angle to the parent bone
    /// can be determined by simple lookups
    /// This means that we can remove root and tip bone from the IK chain, and only have a complex
    /// chain for in-between bones.
    /// The order of solution is:
    ///  - given a target point T, rotate the yaw around its vertical Y axis that its negative X looks towards the target.
    ///  - given the target point distance from root, construct target T2 which is T + looked-up-direction * foot length
    ///  - now solve the IK chain in a single plane for T2
    /// </summary>

    [DefaultExecutionOrder(100)]
    public class MultijointedLegIK : MonoBehaviour, IHasIKTarget
    {
        public Vector3 IKTarget { get; set; }

        [SerializeField] Transform yawTransform;

        [SerializeField] Transform[] rotationTransforms;

        [SerializeField] Transform tip;

        class ChainSegment {
            internal Transform transform;
            internal Vector2 startingPoint;
            internal float length;

            internal Vector2 nodePos;

            internal float compensationAngle;
        }

        [SerializeField][Range(1, 20)] int iterations;

        ChainSegment[] chain;

        [SerializeField] float footLength;
        [SerializeField] AnimationCurve footOffsetPerDistance;

        Transform spaceRoot;

        private void Start() {
            CaptureIKChain();
        }

        public void LateUpdate() {
            if (spaceRoot == null) return;

            // rot needs to be pointing in the direction of the projection of IKTarget on the plane of yawTransform

            var parent = yawTransform.parent;

            var foo = parent.InverseTransformPoint(IKTarget);
            foo.y = 0;
            if (foo.sqrMagnitude > float.Epsilon) {
                var localRot = Quaternion.LookRotation(foo.normalized, Vector3.up);
                yawTransform.localRotation = localRot;
            }

            var localTarget2d = FlattenToYZ(spaceRoot.InverseTransformPoint(IKTarget));
            
            var planarDistance = localTarget2d.y;
            
            var footOffset =  footOffsetPerDistance.Evaluate(planarDistance);

            footOffset = 0;

            var footH = Mathf.Sqrt(1 - footOffset * footOffset);
            var footDir = new Vector2(0,footH);
            var finalTarget = localTarget2d + footDir * footLength;

            SolveFABRIK(chain, finalTarget, iterations);

            AlignBonesToSolvedPositions();
        }

        private void AlignBonesToSolvedPositions() {
            for (var i = 0; i < chain.Length - 1; i++) {
                var knee = chain[i].transform;
                // find worldspace target that we wish to "look at" longitudinally
                var worldFarKneePos = spaceRoot.TransformPoint(DeflattenFromYZ(chain[i + 1].nodePos));

                var farDir = (worldFarKneePos - knee.position).normalized;
                // var farDirLocal = FlattenToYZ(knee.InverseTransformDirection(farDir));
                // farDirLocal = farDirLocal.Rotated(chain[i].compensationAngle);
                // farDir = knee.TransformDirection(farDirLocal);
                // var desiredWorldForward = knee.forward;
                // var desiredWorldUp = Vector3.Cross(farDir, desiredWorldForward).normalized;

                var upVector = yawTransform.up;
                var dot = Vector3.Dot(farDir, upVector);
                if (Mathf.Abs(dot) > 0.7f) upVector = yawTransform.forward;

                chain[i].transform.rotation = Quaternion.LookRotation(farDir, upVector);
            }

            var worldFootPos = IKTarget;
            var ankle = chain[chain.Length - 1].transform;
            var ankleUp = (ankle.position - worldFootPos).normalized;
            ankle.rotation = Quaternion.LookRotation(ankle.forward, ankleUp);
        }

        Vector2 FlattenToYZ(Vector3 v) => new Vector2(v.z, v.y);
        Vector3 DeflattenFromYZ(Vector2 v) => new Vector3(0, v.y, v.x);
        private void CaptureIKChain() {
            if (spaceRoot == null) {
                spaceRoot = new GameObject($"IK helper: [{transform.name}]").transform;
            }

            spaceRoot.transform.rotation = Quaternion.LookRotation(yawTransform.forward, yawTransform.up);
            spaceRoot.transform.position = rotationTransforms[0].position;
            spaceRoot.transform.parent = yawTransform;

            var n = rotationTransforms.Length+1;
            chain = new ChainSegment[n];
            for (var i = 0; i < n-1; i++) {
                var ssp = FlattenToYZ(spaceRoot.InverseTransformPoint(rotationTransforms[i].position));

                chain[i] = new ChainSegment {transform = rotationTransforms[i], startingPoint = ssp};
                // Debug.Log($"{chain[i].transform.name} starting point = {ssp:F2}");
            }

            chain[n-1] = new ChainSegment { transform = tip, startingPoint = FlattenToYZ(spaceRoot.InverseTransformPoint(tip.position)) };
            // Debug.Log($"{chain[n-1].transform.name} starting point = {chain[n-1].startingPoint:F2}");
            
            for (var i = 0; i < n - 1; i++) { 
                chain[i].length = (chain[i].startingPoint - chain[i+1].startingPoint).magnitude;
                var comp = chain[i].transform.InverseTransformPoint(chain[i+1].transform.position);
                
                chain[i].compensationAngle = Mathf.Atan2(comp.x, comp.z) * Mathf.Rad2Deg;

                // chain[i].compensationAngle = 0;
                // Debug.Log($"{chain[i].transform.name} comp angle towards the next one : {chain[i].compensationAngle:F0}° and length: {chain[i].length*100:F0}cm");
            }
            
            foreach (var item in chain) {
                item.nodePos = item.startingPoint;
            }
        }

        #if UNITY_EDITOR
        private void OnDrawGizmos() {
            if (chain != null && chain.Length > 0) {
                Gizmos.color = Color.cyan;
                for (var i = 0; i < chain.Length; i++) {
                    Gizmos.DrawSphere(chain[i].transform.position, 0.1f);
                    if (i < chain.Length - 1) { 
                        Gizmos.DrawLine(chain[i].transform.position, chain[i+1].transform.position);
                    }
                }

                Gizmos.color = Color.red;
                for (var i = 0; i < chain.Length; i++) {
                    if (i < chain.Length - 1) { 
                        Gizmos.DrawLine(
                            spaceRoot.TransformPoint(DeflattenFromYZ(chain[i].nodePos)),
                            spaceRoot.TransformPoint(DeflattenFromYZ(chain[i+1].nodePos))
                        );
                    }
                }
            }
        }
        #endif

        static void SolveFABRIK(ChainSegment[] chain, Vector2 target, int iterations, float[] weights = null) {
            foreach (var item in chain) item.nodePos = item.startingPoint;
            for (var iter = 0; iter < iterations; iter++) {
                var n = chain.Length;
                // back:
                chain[n-1].nodePos = target;
                for (var i = n-2; i >= 0; i--) {
                    // var weight = weights?[i]??1;
                    var a = chain[i].nodePos; var b = chain[i+1].nodePos;
                    var dir = (a - b).normalized;
                    chain[i].nodePos = b + dir * chain[i].length;
                }

                // forward:
                chain[0].nodePos = chain[0].startingPoint;
                for (var i = 1; i < n; i++) {
                    // var weight = weights?[i-1]??1;
                    var a = chain[i].nodePos; var b = chain[i-1].nodePos;
                    var dir = (a - b).normalized;
                    chain[i].nodePos = b + dir * chain[i-1].length;
                }
            }            
            
        }

    }
}
