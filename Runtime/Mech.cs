using System;
using System.Collections.Generic;
using UnityEngine;

namespace K3.Mech {
    // a physics capable vehicle

    public class Mech : MonoBehaviour {
        [SerializeField] bool wireComponentChildrenOnStart;
        public T GetSystem<T>() {
            foreach (var component in components) if (component is T t) return t;
            return default;
        }
        internal IEnumerable<T> ListSystems<T>() {
            foreach (var component in components) if (component is T t) yield return t;
        }

        private void Start() {
            if (wireComponentChildrenOnStart) WireComponentChildren();
        }

        void WireComponentChildren() {
            components.Clear();
            components.AddRange(GetComponentsInChildren<IMechComponent>(true));
            foreach (var c in components) c.OnSlottedIn(this);
        }

        List<IMechComponent> components = new List<IMechComponent>();

        public void RegisterNew(IMechComponent mechComponent) {
            if (!components.Contains(mechComponent)) {
                components.Add(mechComponent);
                mechComponent.OnSlottedIn(this);
            }
        }
    }

    public interface IMechComponent {
        public Mech Mech { get; }
        void OnSlottedIn(Mech mech);
    }

    public abstract class MechComponent : MonoBehaviour, IMechComponent {
        public Mech Mech { get; private set; }

        protected virtual void Update() {        
            if (Mech != null) MechUpdate();
        }

        protected virtual void MechStart() { }
        protected virtual void MechUpdate() { }

        void IMechComponent.OnSlottedIn(Mech mech) {
            this.Mech = mech;
            MechStart();
        }
    }
}