using UnityEngine;

namespace K3.Physics.Extensions.Buoyancy {

    //public class MechBuoyancy : SilentMechScript, IHasBuoyancy {
    //    public Buoyancy Buoyancy { get; private set; }
        
    //    Rigidbody rb;
    //    [SerializeField] VariableMassRigidbody vmr;
        
    //    [SerializeField] float dragFactor;
        
    //    protected override void Awake() {
    //        rb = GetComponent<Rigidbody>();
            
    //        Buoyancy = new Buoyancy(rb, GetComponentsInChildren<IBuoyantShape>(true));
    //        base.Awake();
    //    }

    //    private void Start() {
    //        Mech.Buoyancy = Buoyancy;
    //        Mech.GetSystem<IDynamicDrag>().OnWillCalculateDrag += ContributeDrag;
    //    }

    //    bool needsInit = true;

    //    private void ContributeDrag(ref float drag) {
    //        drag += Buoyancy.CurrentDrag / rb.mass * dragFactor;
    //    }

    //    private void FixedUpdate() {

    //        if (needsInit) {
    //            foreach (var contrib in Mech.ListSystems<IMassContributor>()) vmr.RegisterContributor(contrib);
    //            needsInit = false;
    //        }
    //        var v = Buoyancy.CurrentDisplacement;

    //        Buoyancy.Process();
    //    }
    //}
}