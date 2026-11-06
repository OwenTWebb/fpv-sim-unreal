// Real-world flying: streams Google Photorealistic 3D Tiles through Cesium for Unreal (Cesium ion asset 2275207),
// with physics meshes so the quad collides with the buildings and terrain. The georeference origin is put on the ground
// at the chosen latitude / longitude, so Unreal z = 0 is ground level at the start point.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FpvRealWorld.generated.h"

class ACesium3DTileset;
class ACesiumGeoreference;

UCLASS()
class FPVSIM_API AFpvRealWorld : public AActor
{
	GENERATED_BODY()

public:
	AFpvRealWorld();

	/** Starts streaming around Lat/Lon (degrees). Ready() turns true once the ground there has loaded with collision. */
	void Load(double LatDeg, double LonDeg, const FString& IonToken, double ScreenSpaceError);
	void Unload();
	bool IsActive() const { return bActive; }
	bool Ready() const { return bReady; }
	float Progress() const;            // 0..100, tiles loaded
	FString Status;                    // shown on the OSD while loading / on failure
	FVector StartCm = FVector::ZeroVector;

protected:
	virtual void Tick(float DeltaSeconds) override;

private:
	UPROPERTY() TObjectPtr<ACesium3DTileset> Tileset;
	UPROPERTY() TObjectPtr<ACesiumGeoreference> Geo;
	double Lat = 0, Lon = 0;
	bool bActive = false, bReady = false, bHeightKnown = false, bSampling = false;
	int32 SampleTries = 0;
	double LoadStarted = 0, NextSampleAt = 0;
	void SampleGround();
};
