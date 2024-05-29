#nullable enable

using Iviz.Core;
using Unity.Mathematics;
using UnityEngine;

namespace Iviz.Displays
{
    public sealed class RotationDraggable : XRScreenDraggable
    {
        [SerializeField] Vector3 normal;

        public Vector3 Normal
        {
            set => normal = value;
        }
        
        public override Quaternion BaseOrientation
        {
            set => normal = value.Forward();
        }
        
        protected override void OnPointerMove(in Ray pointerRay)
        {
            Transform mTransform = Transform;
            Transform mTarget = TargetTransform;

            if (ReferencePointLocal is not {} referencePointLocal)
            {
                InitializeReferencePoint(pointerRay);
            }
            else
            {
                var normalRay = new Ray(mTransform.TransformPoint(referencePointLocal),
                    mTransform.TransformDirection(normal));

                UnityUtils.PlaneIntersection(normalRay, pointerRay, out Vector3 intersectionWorld,
                    out float cameraDistance);

                if (cameraDistance < 0)
                {
                    return;
                }

                var intersectionLocal = mTransform.InverseTransformPoint(intersectionWorld);

                var m = new float3x3(
                    referencePointLocal.Normalized(),
                    intersectionLocal.Normalized(),
                    normal);

                float det = math.determinant(m);
                float angle = Mathf.Asin(det) * Mathf.Rad2Deg;
                float dampenedAngle = DampingPerFrame is { } damping
                    ? damping * angle
                    : angle;

                var deltaRotation =
                    Quaternion.AngleAxis(dampenedAngle, mTarget.InverseTransformDirection(normalRay.direction));

                mTarget.rotation *= deltaRotation;
                RaiseMoved();
            }
        }
    }
}