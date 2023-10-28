using System.Collections.Generic;
using UnityEngine;

namespace K3.Physics { 

    public interface IMassContributor {
        public float Mass { get; }
    }

    public delegate void DragDelegate(ref float drag);

    public interface IDynamicDrag {
        event DragDelegate OnWillCalculateDrag;
    }

    public class VariableMassRigidbody : MonoBehaviour {
        [SerializeField] bool includeRigidbodyDefaultMass;
        private Rigidbody rb;
        float defaultMass;
        List<IMassContributor> contributors = new();

        private void Start() {
            rb = GetComponent<Rigidbody>();
            defaultMass = includeRigidbodyDefaultMass ? rb.mass : 0;
            
        }

        public void RegisterContributor(IMassContributor ctrb) {
            contributors.Add(ctrb);
        }

        private void FixedUpdate() {
            var m = defaultMass;
            foreach (var ctr in contributors) m += ctr.Mass;
            rb.mass = m;
        }
    } 
}