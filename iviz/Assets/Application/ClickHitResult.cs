#nullable enable

using Iviz.Core;
using UnityEngine;
using UnityEngine.XR.ARFoundation;

namespace Iviz.App
{
    public readonly struct ClickHitResult
    {
        public GameObject GameObject { get; }
        public Vector3 Position { get; }
        public Vector3 Normal { get; }

        ClickHitResult(GameObject gameObject, in Vector3 position, in Vector3 normal)
        {
            ThrowHelper.ThrowIfNull(gameObject, nameof(gameObject));
            GameObject = gameObject;
            Position = position;
            Normal = normal;
        }

        public ClickHitResult(in RaycastHit r) : this(r.collider.gameObject, r.point, r.normal)
        {
        }

        public ClickHitResult(in ARRaycastHit r) : this(r.trackable.gameObject, r.pose.position,
            ((ARPlane)r.trackable).normal)
        {
        }

        public Pose AsPose()
        {
            return UnityUtils.PoseFromUp(Position, Normal);
        }

        public void Deconstruct(out GameObject gameObject, out Vector3 position, out Vector3 normal) =>
            (gameObject, position, normal) = (GameObject, Position, Normal);

        public void Deconstruct(out Vector3 position, out Vector3 normal) => (position, normal) = (Position, Normal);
    }
}