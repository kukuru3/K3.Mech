using System;
using Cysharp.Threading.Tasks;
using K3.Mech.IK;
using K3.Physics.Walker;
#if UNITY_EDITOR
using UnityEditor;
#endif

using UnityEngine;

namespace K3.Mech.LegVisuals
{
    public class MechLegView : MechComponent, IObeysStepCycle {
        private const float IK_TO_NEUTRAL_SMOOTHING = 0.3f;
        
        [SerializeField] int physicsLegIndex;
        [SerializeField] Vector2 planarEquilibrium;
        [SerializeField] float defaultDepth;

        [SerializeField] Vector2 ellipse;
        [SerializeField] LayerMask stepMask;

        [SerializeField] AnimationCurve stepCurve;
        [SerializeField][Range(0f,1f)] float stepFootRaise;

        public IPhysicsWalker Walker { get; private set; }

        protected override void MechStart() {
            base.MechStart();
            Walker = Mech.GetSystem<IPhysicsWalker>();
            foot = GetComponent<IHasIKTarget>();
            LegToPreset(LegPositionPresets.DefaultEquilibrium, true);
        }

        private IHasIKTarget foot;
        private WalkCollisionData legPhysicsState;

        bool handledContact = false;

        public event Action OnStepStarted;
        public event SimpleStepCompletion OnStepComplete;
       
        bool stepInProgress;        
        Vector3 _ikTargetSmoothing;

        public bool CurrentlyStepping => stepInProgress;
        
        private void LateUpdate() {
            if (Walker == null) return;
            legPhysicsState = Walker.GetLegPhysicsState(physicsLegIndex);

            if (legPhysicsState.inContact) {
                if (!handledContact) {
                    foot.IKTarget = legPhysicsState.lastContactPoint;
                    handledContact = true;
                }
            } else {
                handledContact = false;
                var preset = LegPositionPresets.DefaultEquilibrium;
                // var preset = wings.IsPoweredFlight ? LegPositionPresets.Stowed : LegPositionPresets.DefaultEquilibrium;
                LegToPreset(preset, false);
            }
        }

        public LegReachEllipse GetEllipseParameters() {
            var v = Walker.LocalPlanarVelocity.magnitude;
            Vector2 vector;
            if (v < 0.01f) {
                vector = Vector2.up;
            } else {
                vector = Walker.LocalPlanarVelocity.normalized;
            }

            var ellipseA  = Geometry.GetForwardIntersectOnEllipse( vector, ellipse) / 2;
            var ellipseB  = -ellipseA;
            
            var ptFoot = CurrentPlanarPositionOfFoot();
            ptFoot -= planarEquilibrium;
            var linearRelative = Geometry.ProjectPointOnSegment(ptFoot, ellipseB, ellipseA);
            return (ellipseA, ptFoot, linearRelative);
        }

        Vector2 CurrentPlanarPositionOfFoot() {
            return transform.parent.InverseTransformPoint(foot.IKTarget).Flatten();
        }

        public void ExecuteStep(float stepTime, float targetK) {
            // Debug.Log($"Stepping {name} : {stepTime:F2}s, {targetK:P0}");
            var elP = GetEllipseParameters();   
            var target = elP.localPlanarEllipseForwardIntersect * ((targetK - 0.5f) * 2f);

            target += Walker.LocalPlanarVelocity * stepTime;

            var t = GetWorldspaceStepTarget(target);
            if (t.HasValue) StartStepAsync(t.Value.pos, t.Value.normal, stepTime).Forget();
        }

        public async UniTaskVoid StartStepAsync(Vector3 target, Vector3 normal, float totalStepTime) {
            if (stepInProgress) return;
            stepInProgress = true;

            // var immersion = CalculateRelativeImmersion(target);

            OnStepStarted?.Invoke();
            var v = Walker.LocalPlanarVelocity.magnitude;
            
            var startPos = foot.IKTarget;
            var endPos = target;


            var t = 0f;
            while (t < totalStepTime) {
                await UniTask.NextFrame();
                t += Time.deltaTime;
                var nt = t / totalStepTime;

                // fudge longitudinally to give a semblance of horizontal acceleration:
                // var tLong = 1f / (1f + Mathf.Exp(0.5f + nt));

                // var tlong = -(Mathf.Cos(Mathf.PI * nt) - 1f) / 2f;
                var tlong = (nt < 0.5) ? 4 * nt * nt * nt : 1 - Mathf.Pow(-2 * nt + 2, 3) / 2;

                foot.IKTarget = Vector3.Lerp(startPos, endPos, tlong) + transform.parent.up * stepCurve.Evaluate(nt) * stepFootRaise;
            }
            foot.IKTarget = target;
            stepInProgress = false;
            OnStepComplete?.Invoke(target, normal);
        }

        (Vector3 pos, Vector3 normal)? GetWorldspaceStepTarget(Vector2 planarLocalOffset) {
            if (!UnityEngine.Physics.SphereCast(WorldspaceShoulder(offset: planarLocalOffset), 0.2f, -transform.parent.up, out var hitinfo, 4f, stepMask)) {
                return null;
            }

            return (hitinfo.point, hitinfo.normal);
        }

        Vector3 WorldspaceShoulder(float depth = 0f, Vector2 offset = default) {
            return transform.parent.TransformPoint(planarEquilibrium.Deflatten() + offset.Deflatten() + Vector3.down * depth);
        }

        Vector3 ikTargetFootLocalPos;

        enum LegPositionPresets {
            DefaultEquilibrium,
            Stowed
        }

        void LegToPreset(LegPositionPresets preset, bool immediate) {
            var signX = Mathf.Sign(planarEquilibrium.x);
            
            var worldspaceTarget = preset switch  {
                LegPositionPresets.DefaultEquilibrium => WorldspaceShoulder(defaultDepth),
                LegPositionPresets.Stowed => WorldspaceShoulder(depth: defaultDepth - 0.12f, offset: new Vector2(-0.6f * signX, -0.6f)),
                _ => default,
            };

            if (immediate) {
                foot.IKTarget = worldspaceTarget;
                ikTargetFootLocalPos = transform.parent.InverseTransformPoint(foot.IKTarget); // update this, even though it is not used right now.
            } else {
                var localTarget = transform.parent.InverseTransformPoint(worldspaceTarget);

                var (unitVector, magnitude) = ikTargetFootLocalPos.Decompose();
                if (magnitude > 3f) ikTargetFootLocalPos = unitVector * 3f;

                ikTargetFootLocalPos = Vector3.SmoothDamp(ikTargetFootLocalPos, localTarget, ref _ikTargetSmoothing, IK_TO_NEUTRAL_SMOOTHING);
                foot.IKTarget = transform.parent.TransformPoint(ikTargetFootLocalPos);
            }
        }

        #if UNITY_EDITOR
        private void OnDrawGizmos() {
            Gizmos.color = new Color(0.1f, 0.5f, 1f);
            var a = WorldspaceShoulder();
            var b = WorldspaceShoulder(defaultDepth);
            Gizmos.DrawWireSphere(a, 0.2f);
            Gizmos.DrawWireSphere(b, 0.2f);
            Gizmos.DrawLine(a, b);

            var oldMatrix = Gizmos.matrix;
            Gizmos.matrix = transform.parent.localToWorldMatrix;

            var localCenter = planarEquilibrium.Deflatten() + Vector3.down * defaultDepth;
            var localEllipse = ellipse.Deflatten();
            localEllipse.y = 0.3f;

            Gizmos.DrawWireCube(localCenter, localEllipse);

            if (EditorApplication.isPlaying) { 
                var el = GetEllipseParameters();
                Handles.Label(a, $"{el.linearRelative:F3}");
            }
            
            Gizmos.matrix = oldMatrix;

            if (foot != null) { 
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(foot.IKTarget, 0.15f);
            }
        }
        #endif
    }
}
