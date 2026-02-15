using Fixed64;

namespace Massive.Physics;

public struct BoxColliderProxy : ISupportMappable {
	public FVector3 Position;
	public FVector3 Rotation;
	public FVector3 HalfExtents;

	public FVector3 Center => Position;

	public FVector3 SupportPoint(FVector3 direction) {
		return Center + new FVector3(
			direction.X >= 0 ? HalfExtents.X : -HalfExtents.X,
			direction.Y >= 0 ? HalfExtents.Y : -HalfExtents.Y,
			direction.Z >= 0 ? HalfExtents.Z : -HalfExtents.Z
		);
	}
}