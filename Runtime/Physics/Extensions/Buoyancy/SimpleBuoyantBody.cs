using K3.Mech;
using UnityEngine;

namespace K3.Physics.Extensions.Buoyancy {
    public class SimpleBuoyantBody : MonoBehaviour, IHasBuoyancy {
        private Rigidbody rb;
        Buoyancy buoyancy;

        public Buoyancy Buoyancy => buoyancy;

        private void Start() {
            rb = GetComponent<Rigidbody>();
            buoyancy = new Buoyancy(rb, GetComponentsInChildren<IBuoyantShape>(true));
        }
        private void FixedUpdate() {
            buoyancy.Process();
            rb.drag = buoyancy.CurrentDrag / rb.mass;
            rb.angularDrag = buoyancy.CurrentDrag / rb.mass;
        }
    }

    public class MechBuoyancy : MechComponent, IHasBuoyancy {
          public Buoyancy Buoyancy { get; private set; }
        
        Rigidbody rb;

        [SerializeField] VariableMassRigidbody vmr;        
        [SerializeField] float dragFactor;
        
        void Awake() {
            rb = GetComponent<Rigidbody>();            
            Buoyancy = new Buoyancy(rb, GetComponentsInChildren<IBuoyantShape>(true));
        }

        protected override void MechStart() {
            Mech.GetSystem<IDynamicDrag>().OnWillCalculateDrag += ContributeDrag;
        }

        bool needsInit = true;

        private void ContributeDrag(ref float drag) {
            drag += Buoyancy.CurrentDrag / rb.mass * dragFactor;
        }

        private void FixedUpdate() {

            if (needsInit) {
                foreach (var contrib in Mech.ListSystems<IMassContributor>()) vmr.RegisterContributor(contrib);
                needsInit = false;
            }            
            Buoyancy.Process();
        }
    }
}