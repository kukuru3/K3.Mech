using K3.Physics.Environment;
using UnityEditor;
using UnityEngine;

namespace K3.Physics.Extensions.Buoyancy {
    public class BuoyantSphere : MonoBehaviour, IBuoyantShape {

        [field:SerializeField] public float Radius { get; private set; }

        [field:SerializeField] public float Drag { get; private set; }
        
        private float _volume = -1;
        public float Displacement { get {  if (_volume < 0) _volume = Mathf.PI * Radius * Radius * Radius * 4 / 3; return _volume; } }
        
        public Vector3 WorldCenterOfMass => transform.position;

        // In water densities.
        // [field:SerializeField] public float Density { get; private set; }

        public float CalculateSubmersionRatio(WaterVolume volume) {
            var c = WorldCenterOfMass;
            var waterSurfaceY = volume.GetSurfaceY(c.Flatten());
            var h = waterSurfaceY - c.y + Radius;
            if (h < 0f) lastSubmersionRatio = 0f;
            else if (h > Radius + Radius) lastSubmersionRatio = 1f;
            else lastSubmersionRatio = h * h * (3 * Radius - h) / (4 * Radius * Radius * Radius);            
            return lastSubmersionRatio;
        }

        float lastSubmersionRatio;
        #if UNITY_EDITOR
        private void OnDrawGizmos() {
            Handles.Label(WorldCenterOfMass, $"{lastSubmersionRatio:P0}");
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(WorldCenterOfMass, Radius);
        }
        #endif

        void Start() {
            
        }
    }
}