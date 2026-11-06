#include "FpvRealWorld.h"
#include "Cesium3DTileset.h"
#include "CesiumGeoreference.h"
#include "CesiumSampleHeightResult.h"
#include "Engine/World.h"

namespace
{
	const int64 GooglePhotorealisticAsset = 2275207;   // Google Photorealistic 3D Tiles on Cesium ion
}

AFpvRealWorld::AFpvRealWorld()
{
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

void AFpvRealWorld::Load(double LatDeg, double LonDeg, const FString& IonToken, double ScreenSpaceError)
{
	Lat = LatDeg; Lon = LonDeg;
	bActive = true; bReady = false; bHeightKnown = false;
	LoadStarted = GetWorld()->GetRealTimeSeconds();
	Status = TEXT("Loading the real world...");
	if (!Geo) Geo = ACesiumGeoreference::GetDefaultGeoreferenceForActor(this);
	Geo->SetOriginLongitudeLatitudeHeight(FVector(Lon, Lat, 0.0));   // ground height is not known yet: fixed up below
	if (!Tileset)
	{
		FActorSpawnParameters P; P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		Tileset = GetWorld()->SpawnActor<ACesium3DTileset>(FVector::ZeroVector, FRotator::ZeroRotator, P);
		Tileset->SetGeoreference(Geo);
		Tileset->SetTilesetSource(ETilesetSource::FromCesiumIon);
		Tileset->SetIonAssetID(GooglePhotorealisticAsset);
		Tileset->SetCreatePhysicsMeshes(true);
		Tileset->ShowCreditsOnScreen = true;   // Google / Cesium attribution, required by the terms
	}
	Tileset->SetIonAccessToken(IonToken);
	Tileset->SetMaximumScreenSpaceError(ScreenSpaceError);
	Tileset->SetActorHiddenInGame(false);
	Tileset->SetActorEnableCollision(true);
	Tileset->RefreshTileset();

	SampleTries = 0; bSampling = false; NextSampleAt = LoadStarted + 0.5;   // sampled from Tick: the tileset restarts itself on its first frame
}

/** Asks Cesium for the surface height at the start point, then moves the origin there so Unreal z = 0 is the ground. */
void AFpvRealWorld::SampleGround()
{
	bSampling = true; ++SampleTries;
	TWeakObjectPtr<AFpvRealWorld> Self(this);
	Tileset->SampleHeightMostDetailed({ FVector(Lon, Lat, 0.0) }, FCesiumSampleHeightMostDetailedCallback::CreateLambda(
		[Self](ACesium3DTileset*, const TArray<FCesiumSampleHeightResult>& R, const TArray<FString>& Warnings)
		{
			if (!Self.IsValid() || !Self->bActive) return;
			Self->bSampling = false;
			if (R.Num() && R[0].SampleSuccess)
			{
				Self->Geo->SetOriginLongitudeLatitudeHeight(FVector(Self->Lon, Self->Lat, R[0].LongitudeLatitudeHeight.Z));
				Self->bHeightKnown = true;
				UE_LOG(LogTemp, Display, TEXT("FPV real world: ground at %.6f, %.6f is %.1f m above the ellipsoid"), Self->Lat, Self->Lon, R[0].LongitudeLatitudeHeight.Z);
				return;
			}
			for (const FString& W : Warnings) UE_LOG(LogTemp, Warning, TEXT("FPV real world: height sample %d failed: %s"), Self->SampleTries, *W);
			Self->NextSampleAt = Self->GetWorld()->GetRealTimeSeconds() + 2.0;   // retry; after a few tries Tick traces for the ground instead
		}));
}

void AFpvRealWorld::Unload()
{
	bActive = false; bReady = false;
	if (Tileset) { Tileset->SetActorHiddenInGame(true); Tileset->SetActorEnableCollision(false); }
}

float AFpvRealWorld::Progress() const { return Tileset ? Tileset->GetLoadProgress() : 0.f; }

void AFpvRealWorld::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!bActive || bReady || !Tileset) return;
	const double Now = GetWorld()->GetRealTimeSeconds();
	if (!bHeightKnown && !bSampling && SampleTries < 4 && Now >= NextSampleAt) SampleGround();
	const bool bGaveUpSampling = !bHeightKnown && !bSampling && SampleTries >= 4;
	if (!bHeightKnown && !bGaveUpSampling) { Status = FString::Printf(TEXT("Finding the ground... %.0f %% loaded"), Progress()); return; }
	// ready once the ground under the start point has collision: trace down through the start point
	// (a wide range when the height lookup failed, as the ground can then be far from z = 0)
	const double Range = bHeightKnown ? 50000.0 : 1000000.0;
	FHitResult H;
	FCollisionQueryParams Q(SCENE_QUERY_STAT(FpvRealStart), true);
	// wait for the detailed tiles: the first, coarse ones can sit metres off the real surface and are swapped out
	// under the quad as they refine (with the origin on the sampled ground, the real surface is at z ~ 0)
	const bool bSettled = Progress() >= 99.5f || Now - LoadStarted > 60.0;
	if (bSettled && GetWorld()->LineTraceSingleByChannel(H, FVector(0, 0, Range), FVector(0, 0, -Range), ECC_Visibility, Q)
		&& (!bHeightKnown || FMath::Abs(H.ImpactPoint.Z) < 2000.0 || Now - LoadStarted > 60.0))   // within 20 m: trees and roofs differ between detail levels
	{
		StartCm = H.ImpactPoint + FVector(0, 0, 2.0);
		bReady = true; Status.Reset();
		UE_LOG(LogTemp, Display, TEXT("FPV real world: ready, start z = %.0f cm (on %s)"), StartCm.Z, *GetNameSafe(H.GetActor()));
	}
	else if (Now - LoadStarted > 90.0) Status = TEXT("Could not find the ground here: check the internet connection, or try another location");
	else Status = FString::Printf(TEXT("Loading the real world... %.0f %%"), Progress());
}
