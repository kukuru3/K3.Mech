
using UnityEngine;

namespace K3.Physics.Walker {
    public class SubmergedWalkerIsLessManeuverable : MonoBehaviour {
        private void Start() {
            var walker = GetComponent<TerrainAdaptiveWalker>();
            walker.OnWillReceiveActuation += InjectActuation;
        }

        private void InjectActuation(ref TerrainAdaptiveWalker.Actuation actuation) {
            actuation.maneuverabilityFactor = 1f; // var submersionManeuverabilityFactor = Mech.Buoyancy.SubmersionFactor.Map(0f, 0.4f, 1f, 0.35f);
        }
    }
}