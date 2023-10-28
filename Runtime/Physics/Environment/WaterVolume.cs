using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace K3.Physics.Environment
{
    public class WaterVolume : MonoBehaviour
    {
        float NominalSurfaceY { get; set; }

        [field:SerializeField]public float Viscosity { get; private set;} = 1;
        [field:SerializeField]public float FluidDensity { get; private set;} = 1000;

        private void Start() {
            NominalSurfaceY = transform.TransformPoint(GetComponent<BoxCollider>().size / 2).y;
        }

        public float GetSurfaceY(Vector2 planarCoords) => NominalSurfaceY;
    }
}
