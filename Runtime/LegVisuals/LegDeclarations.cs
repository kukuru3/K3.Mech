using System;
using UnityEngine;

namespace K3.Mech.LegVisuals
{
    public delegate void SimpleStepCompletion(Vector3 worldPos, Vector3 worldNormal);

    public interface IObeysStepCycle {
        LegReachEllipse GetEllipseParameters();
        public bool CurrentlyStepping { get; }
        public void ExecuteStep(float stepTime, float targetK);
    }

    public struct LegReachEllipse {
        public Vector2 localPlanarEllipseForwardIntersect;
        public Vector2 localPlanarFootPos;
        public float linearRelative;

        public LegReachEllipse(Vector2 localPlanarEllipseForwardIntersect, Vector2 localPlanarFootPos, float linearRelative) {
            this.localPlanarEllipseForwardIntersect = localPlanarEllipseForwardIntersect;
            this.localPlanarFootPos = localPlanarFootPos;
            this.linearRelative = linearRelative;
        }

        public override bool Equals(object obj) => obj is LegReachEllipse other && localPlanarEllipseForwardIntersect.Equals(other.localPlanarEllipseForwardIntersect) && localPlanarFootPos.Equals(other.localPlanarFootPos) && linearRelative == other.linearRelative;
        public override int GetHashCode() => HashCode.Combine(localPlanarEllipseForwardIntersect, localPlanarFootPos, linearRelative);

        public void Deconstruct(out Vector2 localPlanarEllipseForwardIntersect, out Vector2 localPlanarFootPos, out float linearRelative) {
            localPlanarEllipseForwardIntersect = this.localPlanarEllipseForwardIntersect;
            localPlanarFootPos = this.localPlanarFootPos;
            linearRelative = this.linearRelative;
        }

        public static implicit operator (Vector2 localPlanarEllipseForwardIntersect, Vector2 localPlanarFootPos, float linearRelative)(LegReachEllipse value) => (value.localPlanarEllipseForwardIntersect, value.localPlanarFootPos, value.linearRelative);
        public static implicit operator LegReachEllipse((Vector2 localPlanarEllipseForwardIntersect, Vector2 localPlanarFootPos, float linearRelative) value) => new LegReachEllipse(value.localPlanarEllipseForwardIntersect, value.localPlanarFootPos, value.linearRelative);
    }
}
