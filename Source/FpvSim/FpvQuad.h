// The quad: runs QuadSim at a fixed physics rate inside Tick, reads the radio, collides against the level with line traces,
// draws a 5" freestyle quad (with modelled props that blur when they spin) and carries the FPV / chase / line-of-sight
// cameras. Also owns the settings (fpv_settings.json) and the in-sim settings menu.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "QuadSim.h"
#include "FpvRadio.h"
#include "FpvQuad.generated.h"

class UCameraComponent;
class UStaticMeshComponent;
class UProceduralMeshComponent;
class UMaterialInstanceDynamic;
class UTexture2D;
class AFpvWorld;
class AFpvRealWorld;

/** One row of the settings menu: a number (shown as Value * Scale), a toggle, an action or a section heading. */
struct FFpvMenuItem
{
	enum EKind { Heading, Number, Toggle, Action, Choice };
	EKind Kind = Heading;
	FString Label, Unit;
	double* Value = nullptr; double Scale = 1, Min = 0, Max = 1, Step = 1; int32 Decimals = 0;   // Min / Max / Step in shown units
	bool* Flag = nullptr;
	int32 ActionId = 0;
	TArray<FString> Options;   // Choice: Value holds the index
};

UCLASS()
class FPVSIM_API AFpvQuad : public AActor
{
	GENERATED_BODY()

public:
	AFpvQuad();

	fpv::QuadSim Sim;
	FFpvRadio Radio;
	fpv::Inputs In;

	// settings (fpv_settings.json next to the .uproject); the menu edits these live
	double CamUptiltDeg = 25, CamFovDeg = 120;
	double PhysicsHz = 2000;
	double DragScale = 1;
	bool bPropsInView = true;

	// world: the procedural field, or the real world (Google Photorealistic 3D Tiles via Cesium)
	struct FPlace { FString Name; double Lat = 0, Lon = 0; double Alt = 0; };   // Alt: start this many m up
	TArray<FPlace> Places;
	bool bRealWorld = false;
	double PlaceIndex = 0;
	double TileDetail = 12;           // Cesium maximum screen-space error: lower = sharper, more download
	FString IonToken;                 // Cesium ion access token (cesium_ion_token.txt or $CESIUM_ION_TOKEN)
	bool bWaitingForWorld = false;    // physics paused while the real world streams in
	UPROPERTY() TObjectPtr<AFpvWorld> Field;
	UPROPERTY() TObjectPtr<AFpvRealWorld> Real;
	/** Switches to the field or the real world (at Places[PlaceIndex]) and puts the quad on the ground there. */
	void ApplyWorld();

	enum ECam { CamFpv, CamChase, CamLos, NumCams };
	int32 CamMode = CamFpv;
	bool bKeyboard = false;      // keyboard flying (no radio): Space arm, W/S throttle, A/D yaw, arrows pitch/roll
	bool bArmBlocked = true;     // after a reset or start the arm switch must be seen OFF first
	bool bShowSticks = true;
	bool bCleanView = false;     // O: recording mode, no OSD (map credits stay)
	bool bAirHold = false;       // air start: held still until armed
	double AirStartM = 0;
	bool bTurtleKey = false;     // T toggles turtle mode (for radios without a turtle switch)
	bool bResetWas = false, bCamWas = false;   // reset / camera switch edges
	bool bTestFlight = false; double TestTime = 0;
	double FlightTime = 0;       // s armed
	float Fps = 0, PhysMs = 0;
	FString Notice; double NoticeTime = -100;
	FString SettingsPath;

	// settings menu (Tab / M); the sim is paused while it is open
	bool bMenuOpen = false;
	int32 MenuSel = 0;
	TArray<FFpvMenuItem> Menu;
	FString MenuValue(const FFpvMenuItem& It) const;

	void ResetToStart();
	void LoadSettings();
	void SaveSettings();
	void ResetSettings();
	/** Re-derives everything that depends on the editable settings (inertia from mass, camera, prop visibility, ...). */
	void ApplySettings();
	FVector StartCm = FVector::ZeroVector; float StartYawDeg = 0.f;

protected:
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

private:
	void HandleKeys(float Dt);
	void HandleMenu(float Dt);
	void UpdateInputs(float Dt);
	void ApplyPose();
	void UpdateProps(float Dt);
	void UpdateCameras(float Dt);
	void Say(const FString& S);
	void BuildModel();
	void BuildMenu();
	void MenuStep(int32 Dir, bool bBig);
	void MenuMove(int32 Dir);
	UStaticMeshComponent* Part(UStaticMesh* Mesh, const FVector& LocCm, const FRotator& Rot, const FVector& SizeCm, const FLinearColor& Color, float MeshSizeCm = 100.f);

	UPROPERTY() TObjectPtr<USceneComponent> Root;
	UPROPERTY() TObjectPtr<USceneComponent> Body;
	UPROPERTY() TObjectPtr<UCameraComponent> FpvCam;
	UPROPERTY() TObjectPtr<UCameraComponent> ChaseCam;
	UPROPERTY() TObjectPtr<UCameraComponent> LosCam;
	UPROPERTY() TObjectPtr<UStaticMeshComponent> CamBlock;
	UPROPERTY() TObjectPtr<UTexture2D> BlurTex;

	struct FProp
	{
		USceneComponent* Spin = nullptr;              // rotates about the motor axis
		UProceduralMeshComponent* Blades = nullptr;   // the real blade shape: seen when stopped
		UProceduralMeshComponent* Disc = nullptr;     // translucent blur: seen when spinning
		UMaterialInstanceDynamic* DiscMat = nullptr;
		double Angle = 0;
	};
	FProp Props[4];
	UPROPERTY() TArray<TObjectPtr<UObject>> Keep;   // keeps the prop components / materials referenced

	double Accum = 0;
	float KbThrottle = 0;
	double MenuRepeatAt = 0; int32 MenuHeld = 0;
	FVector ChasePos = FVector::ZeroVector;
};
