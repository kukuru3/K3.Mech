using System.Collections.Generic;
using System.Linq;
using K3.Physics.Environment;
using UnityEngine;
using UnityPhysics = UnityEngine.Physics;

namespace K3.Physics.Extensions.Buoyancy {

    public class Buoyancy {
        List<IBuoyantShape> shapes = new();
        Rigidbody rb;
        public float CurrentDisplacement { get; private set; }

        public float TotalDisplacement { get; private set; }
        
        public float SubmersionFactor => CurrentDisplacement / TotalDisplacement;
        internal Buoyancy(Rigidbody rb, IEnumerable<IBuoyantShape> shapes) {
            this.shapes = shapes.ToList();
            this.rb = rb;
            TotalDisplacement = shapes.Sum(s => s.Displacement);
            Debug.Log($"total displacement calculated at {TotalDisplacement * 1000:F0}l");
        }

        public WaterVolume CurrentVolume { get; set; }
        public float CurrentDrag { get; private set; }
        internal void Process() {
            if (CurrentVolume != null) 
                UpdateBuoyancy(CurrentVolume);
        }
        private void UpdateBuoyancy(WaterVolume currentVolume) {
            CurrentDisplacement = 0f;
            var totalDrag = 0f;
            foreach (var shape in shapes) {
                var ratio = shape.CalculateSubmersionRatio(currentVolume);
                var displacedVolume = shape.Displacement * ratio;
                CurrentDisplacement += displacedVolume;
                totalDrag += shape.Drag * shape.Displacement * ratio;
                var displacedWaterWeight = displacedVolume * CurrentVolume.FluidDensity;
                var displacementForceVector = -UnityPhysics.gravity * displacedWaterWeight;
                rb.AddForceAtPosition(displacementForceVector, shape.WorldCenterOfMass, ForceMode.Force);
            }

            CurrentDrag = totalDrag * 1000 * currentVolume.Viscosity;
        }
    }
        // each buoyant shape contributes vertical archimedean buoyant force, and drag.

    interface IBuoyantShape {
        public float Displacement { get; }
        public Vector3 WorldCenterOfMass { get; }

        public float Drag { get; }
        
        public float CalculateSubmersionRatio(WaterVolume volume);
    }
}