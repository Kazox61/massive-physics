using Fixed64;
using Massive.Common;
using Massive.Netcode;
using Massive.Physics.Components;

namespace Massive.Physics.Systems;

public class PhysicsSystem : NetSystem, IUpdate {
	private readonly FVector3 _gravity = new(FP.Zero, (-40).ToFP(), FP.Zero); 
	private readonly FP _slop = 0.01f.ToFP();
	private readonly FP _percent = 0.8f.ToFP();
	
	public void Update() {
		var transforms = World.DataSet<Transform>();
		var boxColliders = World.DataSet<BoxCollider>();
		var rigidBodies = World.DataSet<RigidBody>();
		
		World.Include<TriggerEvent>().ForEach(triggerEventEntity => {
			triggerEventEntity.Destroy();
		});

		foreach (var entityId in World.Include<RigidBody>()) {
			ref var rigidBody = ref rigidBodies.Get(entityId);
			rigidBody.IsGrounded = false;
			
			if (rigidBody.InverseMass == FP.Zero || !rigidBody.UseGravity) {
				continue;
			}

			rigidBody.Velocity += _gravity * (FP.One / Session.Config.TickRate.ToFP());
		}

		foreach (var entityId in World.Include<Transform, RigidBody>()) {
			ref var transform = ref transforms.Get(entityId);
			ref var rigidBody = ref rigidBodies.Get(entityId);
			
			if (rigidBody.InverseMass == FP.Zero) {
				continue;
			}

			transform.Position += rigidBody.Velocity * (FP.One / Session.Config.TickRate.ToFP());
		}
		
		var entityIds = new List<int>();
		foreach (var entityId in World.Include<Transform, BoxCollider>()) {
			entityIds.Add(entityId);
		}

		for (var i = 0; i < entityIds.Count; i++) {
			for (var j = i + 1; j < entityIds.Count; j++) {
				var idA = entityIds[i];
				var idB = entityIds[j];

				ref var transformA = ref transforms.Get(idA);
				ref var transformB = ref transforms.Get(idB);

				ref var boxColliderA = ref boxColliders.Get(idA);
				ref var boxColliderB = ref boxColliders.Get(idB);

				var delta = transformB.Position - transformA.Position;
				var radius = boxColliderA.BoundingRadius + boxColliderB.BoundingRadius;

				if (FVector3.Dot(delta, delta) > radius * radius) {
					continue;
				}
				
				boxColliderA.Center = transformA.Position;
				boxColliderB.Center = transformB.Position;

				var gjk = GJK.Calculate(boxColliderA, boxColliderB);

				if (!gjk.CollisionHappened) {
					continue;
				}

				var collision = EPA.Calculate(gjk.Simplex, boxColliderA, boxColliderB);

				var isTrigger = boxColliderA.IsTrigger || boxColliderB.IsTrigger;

				if (isTrigger) {
					World.Create(new TriggerEvent {
						EntifierA = World.GetEntifier(idA),
						EntifierB = World.GetEntifier(idB)
					});
					
					continue;
				}

				ref var rigidBodyA = ref rigidBodies.Get(idA);
				ref var rigidBodyB = ref rigidBodies.Get(idB);

				var normal = collision.PenetrationNormal;
				var depth = collision.PenetrationDepth;

				var invMassA = rigidBodyA.InverseMass;
				var invMassB = rigidBodyB.InverseMass;

				if (invMassA + invMassB == FP.Zero) {
					continue;
				}

				var correction = normal * (FP.Max(depth - _slop, FP.Zero) / (invMassA + invMassB)) * _percent;

				transformA.Position -= correction * invMassA;
				transformB.Position += correction * invMassB;

				var groundDot = 0.7f.ToFP();

				if (normal.Y > groundDot && rigidBodyB.InverseMass > FP.Zero) {
					rigidBodyB.IsGrounded = true;

					if (rigidBodyB.Velocity.Y < FP.Zero) {
						rigidBodyB.Velocity = new FVector3(
							rigidBodyB.Velocity.X,
							FP.Zero,
							rigidBodyB.Velocity.Z
						);
					}
				}

				if (normal.Y < -groundDot && rigidBodyA.InverseMass > FP.Zero) {
					rigidBodyA.IsGrounded = true;

					if (rigidBodyA.Velocity.Y < FP.Zero) {
						rigidBodyA.Velocity = new FVector3(
							rigidBodyA.Velocity.X,
							FP.Zero,
							rigidBodyA.Velocity.Z
						);
					}
				}


				var relativeVelocity = rigidBodyB.Velocity - rigidBodyA.Velocity;
				var velocityAlongNormal = FVector3.Dot(relativeVelocity, normal);

				if (velocityAlongNormal > FP.Zero) {
					continue;
				}

				var restitution = FP.Min(rigidBodyA.Restitution, rigidBodyB.Restitution);

				var impulseMagnitude = -(FP.One + restitution) * velocityAlongNormal;
				impulseMagnitude /= invMassA + invMassB;

				var impulse = impulseMagnitude * normal;

				rigidBodyA.Velocity -= impulse * invMassA;
				rigidBodyB.Velocity += impulse * invMassB;
			}
		}
	}
}