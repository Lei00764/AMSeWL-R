#nullable enable

using System;
using System.Runtime.Serialization;
using Iviz.Common;
using Iviz.Roslib.Utils;

namespace Iviz.Core.Configurations
{
    [DataContract]
    public sealed class ARMarkersConfiguration : JsonToString
    {
        [DataMember] public float MaxMarkerDistanceInM { get; set; } = 0.5f;
        [DataMember] public ARSeenMarker[] Markers { get; set; } = Array.Empty<ARSeenMarker>();
    }
}