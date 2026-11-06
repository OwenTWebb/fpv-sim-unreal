// Procedural flying field, everything with collision: rolling terrain, a gate track, trees, a concrete "bando"
// (open parking structure), a lattice tower, shipping containers and ground clutter for speed and height cues.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FpvWorld.generated.h"

class UInstancedStaticMeshComponent;
class UStaticMesh;

UCLASS()
class FPVSIM_API AFpvWorld : public AActor
{
	GENERATED_BODY()

public:
	AFpvWorld();
	void Build();
	/** Terrain height (cm) at an Unreal x/y (cm). */
	static float GroundZ(float Xcm, float Ycm);

	FVector StartCm = FVector::ZeroVector;
	float StartYawDeg = 0.f;

private:
	UInstancedStaticMeshComponent* Ism(UStaticMesh* Mesh, const FLinearColor& Color, bool bCollide, bool bShadow = true);
	void Box(UInstancedStaticMeshComponent* I, const FVector& CentreCm, const FVector& SizeCm, const FRotator& Rot = FRotator::ZeroRotator);
	void BuildTerrain();
	void BuildTrack();
	void BuildBando(const FVector& OriginCm);
	void BuildTower(const FVector& BaseCm, float HeightM);
	void BuildScatter();

	UPROPERTY() TObjectPtr<USceneComponent> Root;
	UPROPERTY() TObjectPtr<UStaticMesh> Cube;
	UPROPERTY() TObjectPtr<UStaticMesh> Cyl;
	UPROPERTY() TObjectPtr<UStaticMesh> Cone;
};
