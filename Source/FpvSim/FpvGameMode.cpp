#include "FpvGameMode.h"
#include "FpvHUD.h"
#include "FpvQuad.h"
#include "FpvWorld.h"
#include "FpvRealWorld.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/VolumetricCloudComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/ExponentialHeightFog.h"
#include "Engine/SkyLight.h"
#include "Engine/World.h"
#include "Engine/PostProcessVolume.h"
#include "GameFramework/WorldSettings.h"
#include "GameFramework/PlayerController.h"

AFpvGameMode::AFpvGameMode()
{
	DefaultPawnClass = nullptr;   // the quad is not a pawn: we only view through its cameras
	HUDClass = AFpvHUD::StaticClass();
}

void AFpvGameMode::BeginPlay()
{
	Super::BeginPlay();
	UWorld* W = GetWorld();
	if (AWorldSettings* WS = W->GetWorldSettings()) WS->bEnableWorldBoundsChecks = false;   // the real world is far bigger than the default bounds

	// sun + physical sky + real-time sky light + haze + clouds
	if (ADirectionalLight* Sun = W->SpawnActor<ADirectionalLight>(FVector(0, 0, 10000), FRotator(-38.f, 35.f, 0.f)))
	{
		UDirectionalLightComponent* L = Cast<UDirectionalLightComponent>(Sun->GetLightComponent());
		L->SetMobility(EComponentMobility::Movable);
		L->SetIntensity(9.f);
		L->SetAtmosphereSunLight(true);
		L->SetDynamicShadowDistanceMovableLight(30000.f);
	}
	W->SpawnActor<ASkyAtmosphere>(FVector::ZeroVector, FRotator::ZeroRotator);
	if (ASkyLight* Sky = W->SpawnActor<ASkyLight>(FVector::ZeroVector, FRotator::ZeroRotator))
	{
		USkyLightComponent* S = Sky->GetLightComponent();
		S->SetMobility(EComponentMobility::Movable);
		S->SetRealTimeCapture(true);
		S->RecaptureSky();
	}
	if (AExponentialHeightFog* Fog = W->SpawnActor<AExponentialHeightFog>(FVector::ZeroVector, FRotator::ZeroRotator))
	{
		UExponentialHeightFogComponent* F = Fog->GetComponent();
		F->SetFogDensity(0.0015f); F->SetFogHeightFalloff(0.1f); F->SetStartDistance(5000.f); F->SetFogInscatteringColor(FLinearColor(0.42f, 0.52f, 0.68f));
	}
	if (AVolumetricCloud* Clouds = W->SpawnActor<AVolumetricCloud>(FVector::ZeroVector, FRotator::ZeroRotator))
		if (UMaterialInterface* M = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/EngineSky/VolumetricClouds/m_SimpleVolumetricCloud_Inst.m_SimpleVolumetricCloud_Inst")))
			Clouds->FindComponentByClass<UVolumetricCloudComponent>()->SetMaterial(M);

	if (APostProcessVolume* PP = W->SpawnActor<APostProcessVolume>(FVector::ZeroVector, FRotator::ZeroRotator))
	{
		PP->bUnbound = true;   // everywhere: a slightly darker, less washed-out look
		PP->Settings.bOverride_AutoExposureBias = true; PP->Settings.AutoExposureBias = -0.5f;
	}
	AFpvWorld* World = W->SpawnActor<AFpvWorld>(FVector::ZeroVector, FRotator::ZeroRotator);
	World->Build();
	FActorSpawnParameters P; P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AFpvQuad* Quad = W->SpawnActor<AFpvQuad>(World->StartCm, FRotator(0.f, World->StartYawDeg, 0.f), P);
	Quad->Field = World;
	Quad->Real = W->SpawnActor<AFpvRealWorld>(FVector::ZeroVector, FRotator::ZeroRotator);
	Quad->ApplyWorld();   // the field, or the real world if that was saved

	if (APlayerController* PC = W->GetFirstPlayerController())
	{
		PC->SetViewTarget(Quad);
		PC->bShowMouseCursor = false;
		PC->SetInputMode(FInputModeGameOnly());
		PC->ConsoleCommand(TEXT("DisableAllScreenMessages"));
	}
}
