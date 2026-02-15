using Fixed64;

namespace Massive.Physics.Components;

public struct BoxCollider {
	public FVector3 HalfExtents;
	public bool IsTrigger;

	public FP BoundingRadius => FVector3.Length(HalfExtents);
}