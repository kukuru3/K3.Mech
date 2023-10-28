using System;
using System.Collections.Generic;
using K3.Mech;
using K3.Physics.Walker;
using UnityEngine;

namespace K3.Physics.Walker
{   
    public class TerrainAdaptiveWalker : MechComponent, IPhysicsWalker, IDynamicDrag {
        [SerializeField] Transform mechViewTransform;
        [SerializeField] [Range(0f,2f)]float viewSmoothingPos;
        [SerializeField] [Range(0f,2f)]float viewSmoothingRot;

        [SerializeField] float rotCompensatorAccel;
        [SerializeField] AnimationCurve legSpringByDistanceOffset;
        [SerializeField] float legSpringStrMul;
        
        [SerializeField] float springDistance;

        [SerializeField] Vector2 baseOffset;
        [SerializeField] Vector2[] planarLegOffsets;
        [SerializeField] float radialCastOffset;
        [SerializeField] LayerMask walkMask;
        [SerializeField] float maxGripHeight;
        [SerializeField] int minCastHitsForGrip;

        [SerializeField] float minBodyHeightFromGround;
        [SerializeField] float maxBodyHeightFromGround;

        public float DesiredHeightFactor { get; set; } = 0.5f;

        // a stateless variable, used to modify the desired height
        public float HeightFactorModification { get; set; } = 0f;

        public Transform WalkerTransform => transform;

        public Rigidbody WalkerRigidbody => rbody;
        
        WalkCollisionData[] collisionData;
        WalkState walkState;

        public WalkState GetWalkState() => walkState;

        Rigidbody rbody;

        public struct Behaviour {
            public bool wantsToGrip;
            public float rotationFactor;
            public float deltaAttitude;
        }

        public struct Actuation
        {
            public float yawForce;
            public Vector2 planarForce;
            public float   maneuverabilityFactor;
        }

        public delegate void BehaviourCalculationDelegate(ref Behaviour parameters);
        public delegate void ActuationDelegate(ref Actuation actuation);

        public event BehaviourCalculationDelegate OnWillCalculateBehaviour;
        public event ActuationDelegate OnWillReceiveActuation;
        

        void Awake() {
            
            collisionData = new WalkCollisionData[planarLegOffsets.Length];
            rbody = GetComponent<Rigidbody>();
        }

        protected override void MechStart() {
            transform.parent = null;
            DesiredHeightFactor = 0.5f;
        }

        List<Vector3> collisionPoints = new List<Vector3>();

        private Vector3 _viewSmoothPosVel;
        private Quaternion _viewSmoothRotVel;

        private void LateUpdate() {
            if (mechViewTransform != null) {
                mechViewTransform.SetPositionAndRotation(
                    Vector3.SmoothDamp(mechViewTransform.position, transform.position, ref _viewSmoothPosVel, viewSmoothingPos), 
                    TransformUtility.SmoothDamp(mechViewTransform.rotation, transform.rotation, ref _viewSmoothRotVel, viewSmoothingRot)
                );
            }
        }

        private void FixedUpdate() {
            CalculateDrag();

            Behaviour beh = new Behaviour { rotationFactor = 1f, wantsToGrip = true, };
            OnWillCalculateBehaviour?.Invoke(ref beh);

            Actuation act = new Actuation { };
            OnWillReceiveActuation?.Invoke(ref act);

            CalculateCollisionData(beh);
            CalculateWalkState(beh);
            ApplyWalkerPhysics(beh, act);
        }
        public event DragDelegate OnWillCalculateDrag;
        
        private void CalculateDrag() {
            var drag = 0f;
            OnWillCalculateDrag?.Invoke(ref drag);
            rbody.drag = drag;
            rbody.angularDrag = drag;
        }

         private void RotateBodyTowardsGripNormal(Behaviour behaviour) {
            var desiredUp = walkState.legWorldNormal;
            var desiredForward = Vector3.ProjectOnPlane(transform.forward, desiredUp).normalized;
            var targetWorldRotation = Quaternion.LookRotation(desiredForward, desiredUp);   

            var deltaRot = Quaternion.Inverse(rbody.rotation) * targetWorldRotation;

            var deltaX = Mathf.DeltaAngle(0f, deltaRot.eulerAngles.x);
            var deltaY = Mathf.DeltaAngle(0f, deltaRot.eulerAngles.y);
            var deltaZ = Mathf.DeltaAngle(0f, deltaRot.eulerAngles.z);

            var deltaRotApplied = new Vector3(deltaX, deltaY, deltaZ);

            deltaRotApplied *= rotCompensatorAccel * behaviour.rotationFactor;

            // Mech.Buoyancy.SubmersionFactor.Map(0f, 0.6f, 1f, 0.15f)
            // 
            rbody.AddRelativeTorque(deltaRotApplied, ForceMode.Acceleration);
        }


        // nicked from HKVR
        public void Gyrostabilize(float strengthPerOffset, float maxStabilization, float multiplier) {
            var horizonForward = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
            var targetRot = Quaternion.LookRotation(horizonForward, Vector3.up);
            var toAchieveTargetRot = Quaternion.Inverse(transform.rotation) * targetRot;

            var v = toAchieveTargetRot.eulerAngles;
            if (v.x > 180) v.x -= 360;
            if (v.y > 180) v.y -= 360;
            if (v.z > 180) v.z -= 360;

            v *= strengthPerOffset;
            var rotationPower = Quaternion.Angle(Quaternion.identity, Quaternion.Euler(v));

            if (rotationPower > maxStabilization) v *= maxStabilization / rotationPower;

            v *= multiplier;

            v *= Mathf.Deg2Rad;
            AddTorque(v, ForceMode.Acceleration);
        }

        public Vector2 LocalPlanarVelocity { get { 
            var localV = transform.InverseTransformVector(rbody.velocity);
            return localV.Flatten();
        } }

        public float PlanarRotation => Vector3.Dot(rbody.angularVelocity, walkState.legWorldNormal);

        private void ApplyWalkerPhysics(Behaviour parameters, Actuation actuation) {
            if (walkState.hasGrip) {
                RotateBodyTowardsGripNormal(parameters);
                StabilizeBodyHeight(parameters);
                LegsResistGravity();
                ApplyActuatorForce(actuation);
                ApplyWalkerGroundDragFriction();
            }
        }

        bool IsGripSpringEffectivelyEnabled => EnableGripping;

        [SerializeField] float legFrictionFactor;
        // while legs are in contact, the walker horizontal motion induce friction in the ball joints of the legs.
        // this friction is modelled as a loss of velocity
        private void ApplyWalkerGroundDragFriction() {
            var planarWorldspaceV = Vector3.ProjectOnPlane(rbody.velocity, rbody.transform.up);
            rbody.AddForce(-planarWorldspaceV * legFrictionFactor, ForceMode.Acceleration);

            var verticalWorldspaceV = Vector3.Project(rbody.velocity, rbody.transform.up);
            
            if (IsGripSpringEffectivelyEnabled) rbody.AddForce(-verticalWorldspaceV * legFrictionFactor, ForceMode.Acceleration);

            rbody.AddTorque(-rbody.angularVelocity * legFrictionFactor, ForceMode.Acceleration);

        }    

        public void AddForce(Vector3 localspaceForce, ForceMode mode) {
            rbody.AddRelativeForce(localspaceForce, mode);
        }
        public void AddForceWorld(Vector3 worldspaceForce, ForceMode mode) {
            rbody.AddForce(worldspaceForce, mode);
        }

        public void AddTorque(Vector3 localspaceTorque, ForceMode mode) {
            rbody.AddRelativeTorque(localspaceTorque, mode);
        }

        private void ApplyActuatorForce(Actuation actuation) {
            

            rbody.AddRelativeForce(actuation.planarForce.Deflatten() * actuation.maneuverabilityFactor, ForceMode.Acceleration);
            rbody.AddRelativeTorque(0f, actuation.yawForce * actuation.maneuverabilityFactor, 0f, ForceMode.Acceleration);
        }

        [SerializeField] float springDegree = 1f;
        [SerializeField] float congruenceFactor;

        // whether or not the legs grip.
        public bool EnableGripping { get; set; } = true;

        private void StabilizeBodyHeight(Behaviour behaviour) {

            // todo here: avoid stabilization if displacement is too high

            var altDelta = behaviour.deltaAttitude * Time.fixedDeltaTime * 0.9f;
            DesiredHeightFactor += altDelta;
            DesiredHeightFactor = Mathf.Clamp01(DesiredHeightFactor);

            var desiredHeight = GetDesiredHeight();            
            
            var delta = (walkState.legDistance - desiredHeight) / springDistance;
            // if distance from ground is GREATER than desired height, then delta is POSITIVE.

            var tension = Mathf.Pow(delta, springDegree);

            var springPowerFinal = -tension * legSpringStrMul;
            // if distance from ground is GREATER than desired height, then spring power is NEGATIVE

            // if grip spring is enabled, the legs will even pull the mech TOWARDS the legs.
            // otherwise, only if the mech ground offset is shorter than expected, 
            // the legs are compressed, and they will stabilize.
            
            if (springPowerFinal > 0f || IsGripSpringEffectivelyEnabled) {
                if (Mathf.Abs(springPowerFinal) > 1e-6) {

                    var congruence = Vector3.Dot(rbody.velocity, transform.up) / springPowerFinal;
                    congruence = Mathf.Clamp(congruence, -3f, 3f);
                    springPowerFinal *= 1f - congruence * congruenceFactor;

                    var upspeed = Vector3.Dot(rbody.velocity, transform.up);
                    // Debug.Log($"Spring system: delta:{delta:F3} => tension:{tension:F3}=> final upward force:{springPowerFinal:F3}; currentUpSpeed = {upspeed:F3}; congruence = {congruence:F3}");
                    rbody.AddForce(transform.up * springPowerFinal, ForceMode.Acceleration);
                }
            }
            
            HeightFactorModification = 0f; 
        }

        private void LegsResistGravity() {
            if (walkState.hasGrip) {
                // naive:
                var dh = GetDesiredHeight();
                var distance = walkState.legDistance;
                var leeway = 0.3f;
                var antigravityForceMultiplier = (distance - dh).Map(0f, leeway, 1f, 0f);
                
                rbody.AddForce(-UnityEngine.Physics.gravity * antigravityForceMultiplier, ForceMode.Acceleration);
            }
        }

        public float GetDesiredHeight() => Mathf.Lerp(minBodyHeightFromGround, maxBodyHeightFromGround, DesiredHeightFactor + HeightFactorModification);

        public void OverrideDesiredHeight(float factor) => DesiredHeightFactor = factor;
        private void OnDrawGizmos() {
            if (walkState.hasGrip) { 
                var projectedForward = Vector3.ProjectOnPlane(transform.forward, walkState.legWorldNormal).normalized;
                var targetRot = Quaternion.LookRotation(projectedForward, walkState.legWorldNormal);
                rbody.maxAngularVelocity = 60f;

                Gizmos.color = Color.blue;
                Gizmos.DrawLine(transform.position, transform.position + walkState.legWorldNormal * 3f);
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(transform.position, transform.position + targetRot * Vector3.forward * 3f);
            }

            if (collisionData != null && collisionData.Length > 0) {
                
                for (var i = 0; i < collisionData.Length; i++) {
                    var cd = collisionData[i];

                    if (cd.inContact) {
                        Gizmos.color = Color.green;
                        Gizmos.DrawLine(cd.shoulder, cd.lastContactPoint);
                    } else {
                        Gizmos.color = Color.red;
                        var dir = Vector3.down + planarLegOffsets[i].Deflatten() * radialCastOffset;
                        Gizmos.DrawLine(cd.shoulder, cd.shoulder + transform.TransformDirection(dir.normalized) * maxGripHeight);
                    }
                }
            }

        }

        private void CalculateWalkState(Behaviour behaviour) {
            int numInContact = 0;
            collisionPoints.Clear();
            var avgDistance = 0f;
            foreach (var item in collisionData) if (item.inContact) {
                collisionPoints.Add(item.lastContactPoint);
                numInContact++;
                avgDistance += item.distance;
            }
            walkState.legDistance = avgDistance / numInContact;
            walkState.numInContact = numInContact;
            walkState.legWorldNormal = Geometry.GetBestFittingPlaneNormal(collisionPoints) ?? Vector3.up;
            walkState.hasGrip = walkState.numInContact >= minCastHitsForGrip;
        }

        // here we expose stuff for plugins to latch on to.
        // plugins can decide: whether the legs want to grip, what is the rotation factor

        

        private void CalculateCollisionData(Behaviour parameters) {

            bool wantsToGrip = parameters.wantsToGrip; 
            
            //if (LegRegime == LegRegimes.Walk) wantsToGrip = Mech.Buoyancy.SubmersionFactor < 0.3f;
            //if (LegRegime == LegRegimes.UnderwaterWalk) wantsToGrip = true;
            
            for (int i = 0; i < planarLegOffsets.Length; i++) {
                Vector2 plo = planarLegOffsets[i];
                plo.Scale(baseOffset);
                var rayStart = transform.TransformPoint(plo.Deflatten());
                var rayDirection = transform.TransformDirection(Vector3.down + plo.Deflatten() * radialCastOffset);
                var ray = new Ray(rayStart, rayDirection);
                if (wantsToGrip && UnityEngine.Physics.Raycast(ray, out var hitinfo, maxGripHeight, walkMask)) {
                    collisionData[i] = new WalkCollisionData {
                        inContact = true,
                        contactNormal = hitinfo.normal,
                        lastContactPoint = hitinfo.point,
                        distance = hitinfo.distance,
                        shoulder = rayStart,
                    };
                } else {
                    collisionData[i] = default;
                }
            }
        }

        public WalkCollisionData GetLegPhysicsState(int index) => collisionData[index];
 
        public float StepCycle { get; private set; }
    }
}
