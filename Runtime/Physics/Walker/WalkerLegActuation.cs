using K3.Physics.Walker;
using UnityEngine;

namespace K3.Physics.Walker {
    public class WalkerLegActuation : MonoBehaviour {

        [SerializeField] float engineRotationPower;
        [SerializeField] float engineForwardPower;
        [SerializeField] float engineStrafePower;

        private void Start() {
            var walker = GetComponent<TerrainAdaptiveWalker>();

            walker.OnWillReceiveActuation += InjectActuation;
        }

        private void InjectActuation(ref TerrainAdaptiveWalker.Actuation actuation) {
            var s = controls.motionAxis;

            s.Scale(new Vector2(engineStrafePower, engineForwardPower));
            actuation.planarForce += s;
            actuation.yawForce = controls.rotation * engineRotationPower;
        }

        WalkerControls controls;

        public void InjectControls(WalkerControls controls) {
            this.controls = controls;
        }
    }
}