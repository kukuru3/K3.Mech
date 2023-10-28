using UnityEngine;

namespace K3.Physics.Walker {
    public struct WalkerControls {
        public float rotation;
        public Vector2 motionAxis;
        public bool jump;
        public float deltaAttitude;
        public Vector2 flightAxis;
    }

    public struct WalkCollisionData {
        public bool inContact;
        public Vector3 lastContactPoint;
        public Vector3 contactNormal;
        public float   distance;
        internal Vector3 shoulder;
    }

    public struct WalkState {
        public float legDistance;
        public Vector3 legWorldNormal;
        public int numInContact;
        public bool hasGrip;
    }

    public interface IPhysicsWalker {
        // public void SetControls()
        WalkCollisionData GetLegPhysicsState(int index);
        float GetDesiredHeight();
        void OverrideDesiredHeight(float factor);
        WalkState GetWalkState();
        void AddForce(Vector3 localspaceForce, ForceMode mode = ForceMode.Acceleration);
        void AddForceWorld(Vector3 worldspaceForce, ForceMode mode = ForceMode.Acceleration);
        void AddTorque(Vector3 localspaceTorque, ForceMode mode = ForceMode.Acceleration);
        void Gyrostabilize(float strengthPerOffset, float maxStabilization, float multiplier = 1f);

        float PlanarRotation { get; }
        Vector2 LocalPlanarVelocity { get; }

        Transform WalkerTransform { get; }
        bool EnableGripping { get; set; }
        Rigidbody WalkerRigidbody { get; }
        // LegRegimes LegRegime{ get; set; }
    }
}