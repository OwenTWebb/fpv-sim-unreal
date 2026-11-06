#include "FpvQuad.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ProceduralMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/PlatformTime.h"
#include "Kismet/KismetSystemLibrary.h"
#include "FpvWorld.h"
#include "FpvRealWorld.h"
#include "HAL/PlatformMisc.h"
#include "Materials/Material.h"
#if WITH_EDITOR
#include "Materials/MaterialExpressionMultiply.h"
#include "Materials/MaterialExpressionScalarParameter.h"
#include "Materials/MaterialExpressionTextureSampleParameter2D.h"
#include "Materials/MaterialExpressionVectorParameter.h"
#endif

using fpv::V3;

namespace
{
	// sim world (x forward, y left, z up, metres)  <->  Unreal (x forward, y right, z up, centimetres)
	FVector ToUe(const V3& v) { return FVector(v.x * 100.0, -v.y * 100.0, v.z * 100.0); }
	FVector ToUeDir(const V3& v) { return FVector(v.x, -v.y, v.z); }
	V3 FromUe(const FVector& v) { return V3(v.X / 100.0, -v.Y / 100.0, v.Z / 100.0); }
	V3 FromUeDir(const FVector& v) { return V3(v.X, -v.Y, v.Z); }

	/** QuadSim's collision queries answered with line traces against the level. */
	struct FUeWorld : fpv::IWorld
	{
		UWorld* World = nullptr;
		FCollisionQueryParams Params;
		bool bNear = true;   // broad phase, done once per frame
		bool Trace(const V3& From, const V3& To, V3& Hit, V3& N) override
		{
			FHitResult H;
			if (!World->LineTraceSingleByChannel(H, ToUe(From), ToUe(To), ECC_Visibility, Params) || H.bStartPenetrating) return false;
			Hit = FromUe(H.ImpactPoint); N = FromUeDir(H.ImpactNormal);
			return true;
		}
		bool AnythingNear(const V3&, double) override { return bNear; }
	};

	double Num(const TSharedPtr<FJsonObject>& O, const TCHAR* K, double Def) { double v = Def; if (O.IsValid()) O->TryGetNumberField(K, v); return v; }
	TSharedPtr<FJsonObject> Obj(const TSharedPtr<FJsonObject>& O, const TCHAR* K) { const TSharedPtr<FJsonObject>* R; return O.IsValid() && O->TryGetObjectField(K, R) ? *R : nullptr; }

	// ---- quad geometry, cm, relative to the CG (the physics hull bottom is 2.8 cm below it)
	const float ZBottom = -2.8f;                 // underside of the bottom plate / arms
	const float ZProp = 0.0f;                    // prop plane (on top of the motors), level with the CG like the physics
	const FVector LensCm(7.6f, 0.f, 0.6f);      // FPV camera lens: front of the frame, level with the props (they show at low uptilt)
	const float PropRadiusCm = 6.35f;            // 5" prop
	const float PropPitchCm = 10.9f;             // 4.3" pitch
	enum EMenuAction { ActSave = 1, ActReload, ActDefaults, ActCalibrate, ActTurtleSwitch, ActResetSwitch, ActCameraSwitch, ActLoadWorld, ActClose, ActQuit };

	/** A 5" tri-blade prop: twisted, tapered, swept blades with the real pitch distribution. Both faces are built. */
	void BuildBlades(UProceduralMeshComponent* M, double SpinDir)
	{
		TArray<FVector> V; TArray<int32> T; TArray<FVector> N;
		const int32 Sections = 14;
		for (int32 b = 0; b < 3; ++b)
		{
			const double Phi = b * 2.0 * PI / 3.0;
			const FVector Er(FMath::Cos(Phi), FMath::Sin(Phi), 0.0);
			// direction of travel of the blade: CCW from above in the sim is towards -Y from +X in Unreal
			const FVector Et = FVector(FMath::Sin(Phi), -FMath::Cos(Phi), 0.0) * SpinDir;
			const int32 Base = V.Num();
			for (int32 i = 0; i <= Sections; ++i)
			{
				const double u = double(i) / Sections, r = 0.9 + u * (PropRadiusCm - 0.9);
				double c = 1.15 + 0.75 * FMath::Sin(PI * FMath::Min(u * 1.25, 1.0));          // chord: widest around 40 % radius
				if (u > 0.82) c *= FMath::Sqrt(FMath::Max(0.0, 1.0 - FMath::Square((u - 0.82) / 0.18))) * 0.85 + 0.15;   // rounded tip
				const double Th = FMath::Atan(PropPitchCm / (2.0 * PI * r));                   // geometric pitch angle
				const double Sweep = 0.55 * u * u;                                             // swept-back tips
				const FVector C = Er * r - Et * Sweep;
				const FVector Chord = Et * FMath::Cos(Th) + FVector(0, 0, FMath::Sin(Th));   // leading edge is ahead and higher
				V.Add(C + Chord * (c * 0.45)); V.Add(C - Chord * (c * 0.55));
				const FVector Nrm = FVector::CrossProduct(Er, Chord).GetSafeNormal() * (FVector::CrossProduct(Er, Chord).Z >= 0 ? 1 : -1);
				N.Add(Nrm); N.Add(Nrm);
			}
			for (int32 i = 0; i < Sections; ++i)
			{
				const int32 a = Base + 2 * i, bb = a + 1, c = a + 2, d = a + 3;
				T.Append({ a, c, bb, bb, c, d });   // one face ...
				T.Append({ a, bb, c, bb, d, c });   // ... and the other
			}
		}
		M->CreateMeshSection(0, V, T, N, TArray<FVector2D>(), TArray<FColor>(), TArray<FProcMeshTangent>(), false);
	}

	/** Lit, two-sided translucent material for the prop blur: Color (vector), Opacity (scalar) x the texture's alpha.
	 *  Built in code because the project has no content; needs editor data, which the sim has as it runs in UnrealEditor -game. */
	UMaterialInterface* MakeBlurMaterial(UObject* Outer, UTexture2D* Tex)
	{
#if WITH_EDITOR
		UMaterial* M = NewObject<UMaterial>(Outer, TEXT("M_PropBlur"));
		M->BlendMode = BLEND_Translucent;
		M->TwoSided = true;
		M->TranslucencyLightingMode = TLM_Surface;
		M->SetShadingModel(MSM_DefaultLit);
		UMaterialExpressionTextureSampleParameter2D* T = NewObject<UMaterialExpressionTextureSampleParameter2D>(M);
		T->ParameterName = TEXT("Blur"); T->Texture = Tex; T->SamplerType = SAMPLERTYPE_Color;
		UMaterialExpressionVectorParameter* C = NewObject<UMaterialExpressionVectorParameter>(M);
		C->ParameterName = TEXT("Color"); C->DefaultValue = FLinearColor(0.6f, 0.1f, 0.35f);
		UMaterialExpressionScalarParameter* O = NewObject<UMaterialExpressionScalarParameter>(M);
		O->ParameterName = TEXT("Opacity"); O->DefaultValue = 0.5f;
		UMaterialExpressionMultiply* Mul = NewObject<UMaterialExpressionMultiply>(M);
		Mul->A.Connect(4, T);   // texture alpha
		Mul->B.Connect(0, O);
		for (UMaterialExpression* E : { (UMaterialExpression*)T, (UMaterialExpression*)C, (UMaterialExpression*)O, (UMaterialExpression*)Mul }) M->GetExpressionCollection().AddExpression(E);
		M->GetEditorOnlyData()->BaseColor.Connect(0, C);
		M->GetEditorOnlyData()->Opacity.Connect(0, Mul);
		M->PreEditChange(nullptr);
		M->PostEditChange();
		return M;
#else
		return nullptr;
#endif
	}

	/** Translucent blur texture for a spinning prop: faint disc, three curved blade smears, a denser tip ring. */
	UTexture2D* MakeBlurTexture()
	{
		const int32 S = 256;
		UTexture2D* Tex = UTexture2D::CreateTransient(S, S, PF_B8G8R8A8);
		if (!Tex) return nullptr;
		TArray<FColor> Px; Px.SetNumUninitialized(S * S);
		for (int32 y = 0; y < S; ++y)
			for (int32 x = 0; x < S; ++x)
			{
				const float dx = (x + 0.5f) / S * 2.f - 1.f, dy = (y + 0.5f) / S * 2.f - 1.f, r = FMath::Sqrt(dx * dx + dy * dy), a = FMath::Atan2(dy, dx);
				float Al = 0.f;
				if (r < 1.f && r > 0.13f)
				{
					const float Chord = 0.10f + 0.10f * FMath::Sin(PI * FMath::Min((r - 0.13f) / 0.87f * 1.25f, 1.f));
					const float Smear = FMath::Pow(0.5f + 0.5f * FMath::Cos(3.f * (a - 0.9f * r)), 10.f) * 0.2f;
					const float Ring = 0.22f * FMath::Exp(-FMath::Square((r - 0.965f) / 0.03f));
					Al = FMath::Clamp(Chord + Smear + Ring, 0.f, 1.f) * FMath::Clamp((1.f - r) / 0.02f, 0.f, 1.f);
				}
				Px[y * S + x] = FColor(255, 255, 255, uint8(Al * 255.f));
			}
		FTexture2DMipMap& Mip = Tex->GetPlatformData()->Mips[0];
		void* Data = Mip.BulkData.Lock(LOCK_READ_WRITE);
		FMemory::Memcpy(Data, Px.GetData(), Px.Num() * sizeof(FColor));
		Mip.BulkData.Unlock();
		Tex->SRGB = true;
		Tex->UpdateResource();
		return Tex;
	}
}

AFpvQuad::AFpvQuad()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;
	Body = CreateDefaultSubobject<USceneComponent>(TEXT("Body"));
	Body->SetupAttachment(Root);
	FpvCam = CreateDefaultSubobject<UCameraComponent>(TEXT("FpvCam"));
	FpvCam->SetupAttachment(Root);
	FpvCam->SetRelativeLocation(LensCm);
	ChaseCam = CreateDefaultSubobject<UCameraComponent>(TEXT("ChaseCam"));
	ChaseCam->SetupAttachment(Root);
	ChaseCam->SetUsingAbsoluteLocation(true); ChaseCam->SetUsingAbsoluteRotation(true);
	ChaseCam->SetFieldOfView(90.f);
	LosCam = CreateDefaultSubobject<UCameraComponent>(TEXT("LosCam"));
	LosCam->SetupAttachment(Root);
	LosCam->SetUsingAbsoluteLocation(true); LosCam->SetUsingAbsoluteRotation(true);
	LosCam->SetFieldOfView(70.f);
}

UStaticMeshComponent* AFpvQuad::Part(UStaticMesh* Mesh, const FVector& LocCm, const FRotator& Rot, const FVector& SizeCm, const FLinearColor& Color, float MeshSizeCm)
{
	UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this);
	C->SetStaticMesh(Mesh);
	C->SetupAttachment(Body);
	C->SetRelativeLocationAndRotation(LocCm, Rot);
	C->SetRelativeScale3D(SizeCm / MeshSizeCm);
	C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	C->SetCastShadow(true);
	static UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	if (Base) { UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(Base, this); M->SetVectorParameterValue(TEXT("Color"), Color); C->SetMaterial(0, M); }
	C->RegisterComponent();
	return C;
}

void AFpvQuad::BuildModel()
{
	// 5" freestyle quad (cm): X frame with 5 mm arms, 25 mm standoffs, top-mounted 6S pack, 2207 motors, tri-blade props,
	// FPV camera at the front of the frame, lens level with the props (they come into view at low uptilt)
	UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	UStaticMesh* Cyl = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	const FLinearColor Carbon(0.018f, 0.018f, 0.02f), Bell(0.35f, 0.04f, 0.03f), Stator(0.06f, 0.06f, 0.065f), Batt(0.55f, 0.42f, 0.02f), Grey(0.12f, 0.12f, 0.13f), Strap(0.5f, 0.02f, 0.02f);
	const FLinearColor PropCol(0.30f, 0.035f, 0.16f);  // pink, a common freestyle prop colour
	const float Arm = Sim.P.armX * 100.0;
	for (int32 s : { -1, 1 })
		Part(Cube, FVector(0, 0, ZBottom + 0.25f), FRotator(0, 45.f * s, 0), FVector(Arm * 2.83f + 3.f, 2.3f, 0.5f), Carbon);
	Part(Cube, FVector(0, 0, ZBottom + 0.25f), FRotator::ZeroRotator, FVector(12.f, 4.8f, 0.5f), Carbon);       // bottom plate
	Part(Cube, FVector(-0.5f, 0, ZBottom + 3.1f), FRotator::ZeroRotator, FVector(10.5f, 4.4f, 0.25f), Carbon);  // top plate
	for (float sx : { -1.f, 1.f }) for (float sy : { -1.f, 1.f })
		Part(Cyl, FVector(sx * 4.5f - 0.5f, sy * 1.8f, ZBottom + 1.75f), FRotator::ZeroRotator, FVector(0.5f, 0.5f, 2.5f), Grey);   // standoffs
	Part(Cube, FVector(-1.f, 0, ZBottom + 1.7f), FRotator::ZeroRotator, FVector(3.6f, 3.6f, 1.6f), Grey);      // FC / ESC stack
	Part(Cube, FVector(-0.8f, 0, ZBottom + 3.25f + 1.75f), FRotator::ZeroRotator, FVector(7.5f, 3.6f, 3.5f), Batt);   // battery
	Part(Cube, FVector(-0.8f, 0, ZBottom + 3.25f + 1.8f), FRotator::ZeroRotator, FVector(1.6f, 3.8f, 3.65f), Strap);
	CamBlock = Part(Cube, LensCm - FVector(1.1f, 0, 0), FRotator(CamUptiltDeg, 0, 0), FVector(1.9f, 1.9f, 1.9f), Grey);
	BlurTex = MakeBlurTexture();
	UMaterialInterface* BladeBase = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	UMaterialInterface* BlurBase = BlurTex ? MakeBlurMaterial(this, BlurTex) : nullptr;
	if (BlurBase) Keep.Add(BlurBase);
	for (int32 i = 0; i < 4; ++i)
	{
		const FVector L = ToUe(fpv::QuadSim::MotorPos(i, Sim.P));
		Part(Cyl, FVector(L.X, L.Y, ZBottom + 0.5f + 0.5f), FRotator::ZeroRotator, FVector(2.2f, 2.2f, 1.0f), Stator);
		Part(Cyl, FVector(L.X, L.Y, ZBottom + 0.5f + 1.45f), FRotator::ZeroRotator, FVector(2.8f, 2.8f, 1.3f), Bell);
		Part(Cyl, FVector(L.X, L.Y, ZProp + 0.35f), FRotator::ZeroRotator, FVector(0.9f, 0.9f, 0.9f), Grey);     // prop nut
		FProp& P = Props[i];
		P.Spin = NewObject<USceneComponent>(this);
		P.Spin->SetupAttachment(Body); P.Spin->SetRelativeLocation(FVector(L.X, L.Y, ZProp)); P.Spin->RegisterComponent();
		P.Blades = NewObject<UProceduralMeshComponent>(this);
		P.Blades->SetupAttachment(P.Spin); P.Blades->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		BuildBlades(P.Blades, fpv::QuadSim::SpinDir(i));
		if (BladeBase) { UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(BladeBase, this); M->SetVectorParameterValue(TEXT("Color"), PropCol); P.Blades->SetMaterial(0, M); }
		P.Blades->RegisterComponent();
		P.Disc = NewObject<UProceduralMeshComponent>(this);
		P.Disc->SetupAttachment(P.Spin); P.Disc->SetCollisionEnabled(ECollisionEnabled::NoCollision); P.Disc->SetCastShadow(false);
		const float R = PropRadiusCm;
		P.Disc->CreateMeshSection(0, { FVector(-R, -R, 0.15f), FVector(R, -R, 0.15f), FVector(R, R, 0.15f), FVector(-R, R, 0.15f) }, { 0, 2, 1, 0, 3, 2 },
			{ FVector::UpVector, FVector::UpVector, FVector::UpVector, FVector::UpVector }, { FVector2D(0, 0), FVector2D(1, 0), FVector2D(1, 1), FVector2D(0, 1) },
			TArray<FColor>(), TArray<FProcMeshTangent>(), false);
		if (BlurBase && BlurTex)
		{
			P.DiscMat = UMaterialInstanceDynamic::Create(BlurBase, this);
			P.DiscMat->SetTextureParameterValue(TEXT("Blur"), BlurTex);
			P.Disc->SetMaterial(0, P.DiscMat);
			Keep.Add(P.DiscMat);
		}
		P.Disc->RegisterComponent();
		Keep.Add(P.Spin); Keep.Add(P.Blades); Keep.Add(P.Disc);
	}
}

void AFpvQuad::BeginPlay()
{
	Super::BeginPlay();
	const FString Dir = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir());
	SettingsPath = FPaths::Combine(Dir, TEXT("fpv_settings.json"));
	Places = {   // a few to start with; add your own to "places" in fpv_settings.json
		{ TEXT("Yosemite Valley (El Capitan)"), 37.72150, -119.63800 },
		{ TEXT("New York (Central Park South)"), 40.76800, -73.98100 },
		{ TEXT("San Francisco (Crissy Field, Golden Gate)"), 37.80520, -122.46520 },
		{ TEXT("Grand Canyon (Mather Point)"), 36.06170, -112.10760 },
		{ TEXT("Lauterbrunnen, Switzerland"), 46.59370, 7.90860 },
		{ TEXT("Paris (Champ de Mars)"), 48.85560, 2.29860 },
		{ TEXT("Tokyo (Shibuya)"), 35.65950, 139.70050 },
		// landmarks for filming: the start point is the highest surface there, so a tower's coordinates start you on its top;
		// Alt (m) starts you that far up in the air, held still until you arm
		{ TEXT("Landmark: Burj Khalifa, Dubai (650 m, beside the spire)"), 25.197500, 55.274500, 650 },
		{ TEXT("Landmark: One World Trade Center, NYC"), 40.712743, -74.013379 },
		{ TEXT("Landmark: Empire State Building, NYC"), 40.748441, -73.985664 },
		{ TEXT("Landmark: Times Square, NYC (street canyons)"), 40.758000, -73.985500, 60 },
		{ TEXT("Landmark: Statue of Liberty, NYC"), 40.689247, -74.044502 },
		{ TEXT("Landmark: Willis Tower, Chicago"), 41.878876, -87.635915 },
		{ TEXT("Landmark: Chicago River (Wabash Ave bridge)"), 41.888600, -87.626000, 25 },
		{ TEXT("Landmark: Golden Gate Bridge (south tower)"), 37.815900, -122.478300 },
		{ TEXT("Landmark: Golden Gate Bridge (mid-span deck)"), 37.819900, -122.478300 },
		{ TEXT("Landmark: Eiffel Tower, Paris"), 48.858370, 2.294481 },
		{ TEXT("Landmark: Christ the Redeemer, Rio"), -22.951916, -43.210487 },
		{ TEXT("Landmark: Sydney Opera House"), -33.856784, 151.215297 },
		{ TEXT("Landmark: Sydney Harbour Bridge (arch)"), -33.852306, 151.210787 },
		{ TEXT("Landmark: El Capitan summit, Yosemite"), 37.734200, -119.637800 },
		{ TEXT("Landmark: Grand Canyon (over the gorge)"), 36.098900, -112.106800, 150 },
		{ TEXT("Landmark: Lauterbrunnen cliffs, Switzerland"), 46.596700, 7.907100, 80 },
	};
	FFileHelper::LoadFileToString(IonToken, *FPaths::Combine(Dir, TEXT("cesium_ion_token.txt")));
	if (IonToken.TrimStartAndEnd().IsEmpty()) IonToken = FPlatformMisc::GetEnvironmentVariable(TEXT("CESIUM_ION_TOKEN"));
	// a pasted token often wraps onto two lines: a token never contains whitespace, so drop all of it
	IonToken = IonToken.Replace(TEXT("\r"), TEXT("")).Replace(TEXT("\n"), TEXT("")).Replace(TEXT(" "), TEXT("")).Replace(TEXT("\t"), TEXT(""));
	Radio.Load(FPaths::Combine(Dir, TEXT("fpv_radio.json")));
	BuildModel();
	LoadSettings();
	BuildMenu();
	ResetToStart();
	ChasePos = GetActorLocation() - FVector(250, 0, -80);
	if (APlayerController* PC = GetWorld()->GetFirstPlayerController()) PC->ConsoleCommand(TEXT("r.SetNearClipPlane 0.5"));
	bTestFlight = FParse::Param(FCommandLine::Get(), TEXT("FpvTestFlight"));
	FParse::Value(FCommandLine::Get(), TEXT("FpvCam="), CamMode); CamMode = FMath::Clamp(CamMode, 0, NumCams - 1);
	bMenuOpen = FParse::Param(FCommandLine::Get(), TEXT("FpvMenu"));
	{ int32 RealAt = -1; if (FParse::Value(FCommandLine::Get(), TEXT("FpvReal="), RealAt) && Places.IsValidIndex(RealAt)) { bRealWorld = true; PlaceIndex = RealAt; } }   // -FpvReal=<location index>: start in the real world
}

// ------------------------------------------------------------------------------------------------ settings

void AFpvQuad::ApplySettings()
{
	fpv::QuadParams& P = Sim.P;
	const fpv::QuadParams D;
	P.inertia = D.inertia * (P.mass / D.mass);
	P.motorTauDown = P.motorTauUp * D.motorTauDown / D.motorTauUp;
	P.dragCdA = D.dragCdA * DragScale;
	PhysicsHz = FMath::Clamp(PhysicsHz, 500.0, 8000.0);
	FpvCam->SetRelativeRotation(FRotator(CamUptiltDeg, 0.f, 0.f));
	FpvCam->SetFieldOfView(FMath::Clamp(float(CamFovDeg), 60.f, 170.f));
	if (CamBlock) CamBlock->SetRelativeRotation(FRotator(CamUptiltDeg, 0.f, 0.f));
}

void AFpvQuad::ResetSettings()
{
	Sim.P = fpv::QuadParams(); Sim.C = fpv::ControllerParams();
	CamUptiltDeg = 25; CamFovDeg = 120; PhysicsHz = 2000; DragScale = 1; bPropsInView = true;
	ApplySettings();
}

void AFpvQuad::LoadSettings()
{
	ResetSettings();
	FString Text;
	if (!FFileHelper::LoadFileToString(Text, *SettingsPath)) { SaveSettings(); return; }   // first run: write the defaults
	TSharedPtr<FJsonObject> J;
	if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), J) || !J.IsValid()) { Say(TEXT("fpv_settings.json: parse error, using defaults")); return; }
	fpv::QuadParams& P = Sim.P; fpv::ControllerParams& C = Sim.C;
	auto ReadRate = [&](const TCHAR* K, fpv::Rates& R) { TSharedPtr<FJsonObject> O = Obj(Obj(J, TEXT("rates")), K); R.center = Num(O, TEXT("center_dps"), R.center); R.max = Num(O, TEXT("max_dps"), R.max); R.expo = Num(O, TEXT("expo"), R.expo); };
	auto ReadPid = [&](const TCHAR* K, fpv::Pid& R) { TSharedPtr<FJsonObject> O = Obj(Obj(J, TEXT("pid")), K); R.p = Num(O, TEXT("p"), R.p); R.i = Num(O, TEXT("i"), R.i); R.d = Num(O, TEXT("d"), R.d); R.ff = Num(O, TEXT("ff"), R.ff); };
	ReadRate(TEXT("roll"), C.roll); ReadRate(TEXT("pitch"), C.pitch); ReadRate(TEXT("yaw"), C.yaw);
	ReadPid(TEXT("roll"), C.pidRoll); ReadPid(TEXT("pitch"), C.pidPitch); ReadPid(TEXT("yaw"), C.pidYaw);
	TSharedPtr<FJsonObject> Cam = Obj(J, TEXT("camera"));
	CamUptiltDeg = Num(Cam, TEXT("uptilt_deg"), CamUptiltDeg); CamFovDeg = Num(Cam, TEXT("fov_deg"), CamFovDeg);
	if (Cam.IsValid()) Cam->TryGetBoolField(TEXT("props_in_view"), bPropsInView);
	TSharedPtr<FJsonObject> Q = Obj(J, TEXT("quad"));
	P.mass = Num(Q, TEXT("mass_kg"), P.mass);
	P.maxThrust = Num(Q, TEXT("max_thrust_per_motor_n"), P.maxThrust);
	P.propPitchSpeed = Num(Q, TEXT("prop_pitch_speed_ms"), P.propPitchSpeed);
	P.motorTauUp = Num(Q, TEXT("motor_tau_s"), P.motorTauUp);
	P.idle = Num(Q, TEXT("motor_idle"), P.idle);
	DragScale = Num(Q, TEXT("drag_scale"), DragScale);
	P.propwash = Num(Q, TEXT("propwash"), P.propwash);
	P.crashSpeed = Num(Q, TEXT("crash_speed_ms"), P.crashSpeed);
	P.turtlePower = Num(Q, TEXT("turtle_power"), P.turtlePower);
	TSharedPtr<FJsonObject> B = Obj(J, TEXT("battery"));
	P.batteryCells = Num(B, TEXT("cells"), Num(Q, TEXT("battery_cells"), P.batteryCells));
	P.batteryMah = Num(B, TEXT("mah"), Num(Q, TEXT("battery_mah"), P.batteryMah));
	if (B.IsValid()) B->TryGetBoolField(TEXT("infinite"), P.infiniteBattery);
	J->TryGetBoolField(TEXT("airmode"), C.airmode);
	C.throttleLimit = Num(J, TEXT("throttle_limit"), C.throttleLimit);
	PhysicsHz = Num(J, TEXT("physics_hz"), PhysicsHz);
	if (TSharedPtr<FJsonObject> Wo = Obj(J, TEXT("world")))
	{
		Wo->TryGetBoolField(TEXT("real"), bRealWorld);
		PlaceIndex = Num(Wo, TEXT("place"), PlaceIndex); TileDetail = Num(Wo, TEXT("detail"), TileDetail);
		const TArray<TSharedPtr<FJsonValue>>* Arr;
		if (Wo->TryGetArrayField(TEXT("places"), Arr) && Arr->Num())
		{
			Places.Reset();
			for (const TSharedPtr<FJsonValue>& V : *Arr) { TSharedPtr<FJsonObject> O = V->AsObject(); if (!O.IsValid()) continue; Places.Add({ O->GetStringField(TEXT("name")), Num(O, TEXT("lat"), 0), Num(O, TEXT("lon"), 0), Num(O, TEXT("alt"), 0) }); }
		}
	}
	PlaceIndex = FMath::Clamp(FMath::RoundToDouble(PlaceIndex), 0.0, double(FMath::Max(0, Places.Num() - 1)));
	ApplySettings();
}

void AFpvQuad::SaveSettings()
{
	const fpv::QuadParams& P = Sim.P; const fpv::ControllerParams& C = Sim.C;
	auto F = [](double v) { return FString::Printf(TEXT("%g"), v); };
	auto B = [](bool b) { return FString(b ? TEXT("true") : TEXT("false")); };
	auto Rate = [&](const fpv::Rates& R) { return FString::Printf(TEXT("{ \"center_dps\": %s, \"max_dps\": %s, \"expo\": %s }"), *F(R.center), *F(R.max), *F(R.expo)); };
	auto Pid = [&](const fpv::Pid& D) { return FString::Printf(TEXT("{ \"p\": %s, \"i\": %s, \"d\": %s, \"ff\": %s }"), *F(D.p), *F(D.i), *F(D.d), *F(D.ff)); };
	FString S = TEXT("{\n");
	S += FString::Printf(TEXT("  \"rates\": {\n    \"roll\": %s,\n    \"pitch\": %s,\n    \"yaw\": %s\n  },\n"), *Rate(C.roll), *Rate(C.pitch), *Rate(C.yaw));
	S += FString::Printf(TEXT("  \"pid\": {\n    \"roll\": %s,\n    \"pitch\": %s,\n    \"yaw\": %s\n  },\n"), *Pid(C.pidRoll), *Pid(C.pidPitch), *Pid(C.pidYaw));
	S += FString::Printf(TEXT("  \"camera\": { \"uptilt_deg\": %s, \"fov_deg\": %s, \"props_in_view\": %s },\n"), *F(CamUptiltDeg), *F(CamFovDeg), *B(bPropsInView));
	S += FString::Printf(TEXT("  \"quad\": {\n    \"mass_kg\": %s,\n    \"max_thrust_per_motor_n\": %s,\n    \"prop_pitch_speed_ms\": %s,\n    \"motor_tau_s\": %s,\n    \"motor_idle\": %s,\n    \"drag_scale\": %s,\n    \"propwash\": %s,\n    \"crash_speed_ms\": %s,\n    \"turtle_power\": %s\n  },\n"),
		*F(P.mass), *F(P.maxThrust), *F(P.propPitchSpeed), *F(P.motorTauUp), *F(P.idle), *F(DragScale), *F(P.propwash), *F(P.crashSpeed), *F(P.turtlePower));
	S += FString::Printf(TEXT("  \"battery\": { \"cells\": %s, \"mah\": %s, \"infinite\": %s },\n"), *F(P.batteryCells), *F(P.batteryMah), *B(P.infiniteBattery));
	S += FString::Printf(TEXT("  \"airmode\": %s,\n  \"throttle_limit\": %s,\n  \"physics_hz\": %s,\n"), *B(C.airmode), *F(C.throttleLimit), *F(PhysicsHz));
	FString Pl;
	for (int32 k = 0; k < Places.Num(); ++k)
	{
		const FString Alt = Places[k].Alt > 0 ? FString::Printf(TEXT(", \"alt\": %g"), Places[k].Alt) : FString();
		Pl += FString::Printf(TEXT("%s      { \"name\": \"%s\", \"lat\": %.6f, \"lon\": %.6f%s }"), k ? TEXT(",\n") : TEXT(""), *Places[k].Name, Places[k].Lat, Places[k].Lon, *Alt);
	}
	S += FString::Printf(TEXT("  \"world\": {\n    \"real\": %s,\n    \"place\": %s,\n    \"detail\": %s,\n    \"places\": [\n%s\n    ]\n  }\n}\n"), *B(bRealWorld), *F(PlaceIndex), *F(TileDetail), *Pl);
	FFileHelper::SaveStringToFile(S, *SettingsPath);
}

// ------------------------------------------------------------------------------------------------ menu

void AFpvQuad::BuildMenu()
{
	fpv::QuadParams& P = Sim.P; fpv::ControllerParams& C = Sim.C;
	Menu.Reset();
	auto Head = [&](const TCHAR* L) { FFpvMenuItem I; I.Kind = FFpvMenuItem::Heading; I.Label = L; Menu.Add(I); };
	auto Numb = [&](const TCHAR* L, double& V, double Scale, double Min, double Max, double Step, int32 Dec, const TCHAR* Unit)
	{
		FFpvMenuItem I; I.Kind = FFpvMenuItem::Number; I.Label = L; I.Value = &V; I.Scale = Scale; I.Min = Min; I.Max = Max; I.Step = Step; I.Decimals = Dec; I.Unit = Unit; Menu.Add(I);
	};
	auto Tog = [&](const TCHAR* L, bool& B) { FFpvMenuItem I; I.Kind = FFpvMenuItem::Toggle; I.Label = L; I.Flag = &B; Menu.Add(I); };
	auto Act = [&](const TCHAR* L, int32 Id) { FFpvMenuItem I; I.Kind = FFpvMenuItem::Action; I.Label = L; I.ActionId = Id; Menu.Add(I); };

	Head(TEXT("WORLD"));
	Tog(TEXT("Real world (Google 3D, needs internet)"), bRealWorld);
	{ FFpvMenuItem I; I.Kind = FFpvMenuItem::Choice; I.Label = TEXT("Location"); I.Value = &PlaceIndex; I.Min = 0; I.Max = FMath::Max(0, Places.Num() - 1); for (const FPlace& Pl : Places) I.Options.Add(Pl.Name); Menu.Add(I); }
	Numb(TEXT("Real-world detail (lower = sharper)"), TileDetail, 1, 2, 64, 1, 0, TEXT(""));
	Act(TEXT("Go (load the world above)"), ActLoadWorld);
	Head(TEXT("CAMERA"));
	Numb(TEXT("Camera uptilt"), CamUptiltDeg, 1, 0, 70, 1, 0, TEXT("deg"));
	Numb(TEXT("Field of view"), CamFovDeg, 1, 70, 160, 1, 0, TEXT("deg"));
	Tog(TEXT("Props in view"), bPropsInView);
	Head(TEXT("QUAD"));
	Numb(TEXT("All-up weight"), P.mass, 1000, 150, 2000, 10, 0, TEXT("g"));
	Numb(TEXT("Thrust per motor"), P.maxThrust, 1000 / fpv::kG, 200, 4000, 25, 0, TEXT("g"));
	Numb(TEXT("Prop pitch speed (top speed)"), P.propPitchSpeed, 3.6, 80, 400, 5, 0, TEXT("km/h"));
	Numb(TEXT("Motor response"), P.motorTauUp, 1000, 5, 80, 1, 0, TEXT("ms"));
	Numb(TEXT("Motor idle"), P.idle, 100, 0, 15, 0.5, 1, TEXT("%"));
	Numb(TEXT("Drag"), DragScale, 1, 0.2, 4, 0.05, 2, TEXT("x"));
	Numb(TEXT("Propwash"), P.propwash, 1, 0, 3, 0.1, 1, TEXT("x"));
	Numb(TEXT("Crash speed (0 = never)"), P.crashSpeed, 3.6, 0, 200, 5, 0, TEXT("km/h"));
	Head(TEXT("BATTERY"));
	Tog(TEXT("Infinite battery (always 4.2 V/cell)"), P.infiniteBattery);
	Numb(TEXT("Cells"), P.batteryCells, 1, 1, 8, 1, 0, TEXT("S"));
	Numb(TEXT("Capacity"), P.batteryMah, 1, 200, 8000, 50, 0, TEXT("mAh"));
	Head(TEXT("RATES (Betaflight Actual)"));
	Numb(TEXT("Roll centre"), C.roll.center, 1, 10, 1000, 5, 0, TEXT("deg/s")); Numb(TEXT("Roll max"), C.roll.max, 1, 100, 2000, 10, 0, TEXT("deg/s")); Numb(TEXT("Roll expo"), C.roll.expo, 1, 0, 1, 0.01, 2, TEXT(""));
	Numb(TEXT("Pitch centre"), C.pitch.center, 1, 10, 1000, 5, 0, TEXT("deg/s")); Numb(TEXT("Pitch max"), C.pitch.max, 1, 100, 2000, 10, 0, TEXT("deg/s")); Numb(TEXT("Pitch expo"), C.pitch.expo, 1, 0, 1, 0.01, 2, TEXT(""));
	Numb(TEXT("Yaw centre"), C.yaw.center, 1, 10, 1000, 5, 0, TEXT("deg/s")); Numb(TEXT("Yaw max"), C.yaw.max, 1, 100, 2000, 10, 0, TEXT("deg/s")); Numb(TEXT("Yaw expo"), C.yaw.expo, 1, 0, 1, 0.01, 2, TEXT(""));
	Head(TEXT("FLIGHT CONTROLLER"));
	Tog(TEXT("Airmode"), C.airmode);
	Numb(TEXT("Throttle limit"), C.throttleLimit, 100, 10, 100, 1, 0, TEXT("%"));
	Numb(TEXT("Turtle power"), P.turtlePower, 100, 10, 100, 5, 0, TEXT("%"));
	Numb(TEXT("Physics rate"), PhysicsHz, 1, 500, 8000, 250, 0, TEXT("Hz"));
	Head(TEXT(""));
	Act(TEXT("Save settings"), ActSave);
	Act(TEXT("Reload saved settings"), ActReload);
	Act(TEXT("Reset to defaults"), ActDefaults);
	Act(TEXT("Calibrate radio"), ActCalibrate);
	Act(TEXT("Set turtle switch"), ActTurtleSwitch);
	Act(TEXT("Set reset switch"), ActResetSwitch);
	Act(TEXT("Set camera switch"), ActCameraSwitch);
	Act(TEXT("Back to flying"), ActClose);
	Act(TEXT("Quit"), ActQuit);
	MenuSel = 1;
}

FString AFpvQuad::MenuValue(const FFpvMenuItem& It) const
{
	switch (It.Kind)
	{
	case FFpvMenuItem::Number: return FString::Printf(TEXT("%.*f %s"), It.Decimals, *It.Value * It.Scale, *It.Unit);
	case FFpvMenuItem::Toggle: return *It.Flag ? TEXT("ON") : TEXT("OFF");
	case FFpvMenuItem::Choice: return It.Options.IsValidIndex(int32(*It.Value)) ? It.Options[int32(*It.Value)] : FString(TEXT("-"));
	case FFpvMenuItem::Action:
		if (It.ActionId == ActTurtleSwitch) return Radio.SwitchName(FFpvRadio::Turtle);
		if (It.ActionId == ActResetSwitch) return Radio.SwitchName(FFpvRadio::Reset);
		if (It.ActionId == ActCameraSwitch) return Radio.SwitchName(FFpvRadio::Camera);
		return FString();
	default: return FString();
	}
}

void AFpvQuad::MenuMove(int32 Dir)
{
	const int32 N = Menu.Num();
	for (int32 k = 0; k < N; ++k)
	{
		MenuSel = (MenuSel + Dir + N) % N;
		if (Menu[MenuSel].Kind != FFpvMenuItem::Heading) return;
	}
}

void AFpvQuad::MenuStep(int32 Dir, bool bBig)
{
	if (!Menu.IsValidIndex(MenuSel)) return;
	FFpvMenuItem& It = Menu[MenuSel];
	if (It.Kind == FFpvMenuItem::Number)
	{
		const double Shown = FMath::Clamp(FMath::GridSnap(*It.Value * It.Scale + Dir * It.Step * (bBig ? 10 : 1), It.Step), It.Min, It.Max);
		*It.Value = Shown / It.Scale;
		ApplySettings();
	}
	else if (It.Kind == FFpvMenuItem::Toggle) { *It.Flag = !*It.Flag; ApplySettings(); }
	else if (It.Kind == FFpvMenuItem::Choice && It.Options.Num()) *It.Value = (int32(*It.Value) + Dir + It.Options.Num()) % It.Options.Num();
	else if (It.Kind == FFpvMenuItem::Action && Dir != 0)
	{
		switch (It.ActionId)
		{
		case ActSave: SaveSettings(); Say(TEXT("Settings saved to fpv_settings.json")); break;
		case ActReload: LoadSettings(); Say(TEXT("Saved settings reloaded")); break;
		case ActDefaults: ResetSettings(); Say(TEXT("Defaults restored (not saved yet)")); break;
		case ActCalibrate: bMenuOpen = false; Radio.StartCalibration(); Sim.armed = false; break;
		case ActTurtleSwitch: bMenuOpen = false; Radio.StartAssign(FFpvRadio::Turtle); break;
		case ActResetSwitch: bMenuOpen = false; Radio.StartAssign(FFpvRadio::Reset); break;
		case ActCameraSwitch: bMenuOpen = false; Radio.StartAssign(FFpvRadio::Camera); break;
		case ActLoadWorld: bMenuOpen = false; ApplyWorld(); break;
		case ActClose: bMenuOpen = false; break;
		case ActQuit: if (APlayerController* PC = GetWorld()->GetFirstPlayerController()) UKismetSystemLibrary::QuitGame(this, PC, EQuitPreference::Quit, false); break;
		}
	}
}

/** Menu navigation with the arrow keys (Shift = 10x steps, Enter = toggle / do it) or the radio sticks (pitch = up/down, roll = change). */
void AFpvQuad::HandleMenu(float Dt)
{
	APlayerController* PC = GetWorld()->GetFirstPlayerController();
	if (!PC) return;
	if (PC->WasInputKeyJustPressed(EKeys::Enter) || PC->WasInputKeyJustPressed(EKeys::SpaceBar)) MenuStep(1, false);
	const bool bBig = PC->IsInputKeyDown(EKeys::LeftShift) || PC->IsInputKeyDown(EKeys::RightShift);
	int32 V = (PC->IsInputKeyDown(EKeys::Down) ? 1 : 0) - (PC->IsInputKeyDown(EKeys::Up) ? 1 : 0);
	int32 H = (PC->IsInputKeyDown(EKeys::Right) ? 1 : 0) - (PC->IsInputKeyDown(EKeys::Left) ? 1 : 0);
	if (Radio.IsConnected() && Radio.IsCalibrated() && !V && !H)
	{
		const float Pi = Radio.Stick(FFpvRadio::Pitch), Ro = Radio.Stick(FFpvRadio::Roll);
		V = Pi > 0.6f ? -1 : Pi < -0.6f ? 1 : 0;
		H = Ro > 0.6f ? 1 : Ro < -0.6f ? -1 : 0;
	}
	const int32 Held = V * 3 + H;   // what is held right now; act on a change, then auto-repeat
	const double Now = GetWorld()->GetRealTimeSeconds();
	if (Held != MenuHeld) { MenuHeld = Held; MenuRepeatAt = Now + 0.35; }
	else if (!Held || Now < MenuRepeatAt) return;
	else MenuRepeatAt = Now + (V ? 0.1 : 0.06);
	if (V) MenuMove(V);
	else if (H) { if (Menu.IsValidIndex(MenuSel) && Menu[MenuSel].Kind == FFpvMenuItem::Action && H < 0) return; MenuStep(H, bBig); }
}

// ------------------------------------------------------------------------------------------------ flying

void AFpvQuad::ApplyWorld()
{
	if (bRealWorld && IonToken.IsEmpty())
	{
		bRealWorld = false;
		Say(TEXT("Real world needs a Cesium ion token in cesium_ion_token.txt (see README)"));
	}
	if (bRealWorld && Real && Places.IsValidIndex(int32(PlaceIndex)))
	{
		if (Field) { Field->SetActorHiddenInGame(true); Field->SetActorEnableCollision(false); }
		const FPlace& Pl = Places[int32(PlaceIndex)];
		Real->Load(Pl.Lat, Pl.Lon, IonToken, TileDetail);
		// park the quad (and so the camera) over the new start point: Cesium only streams what the camera can see,
		// so from wherever you were flying the ground there might never load
		StartCm = FVector(0, 0, 300.0); StartYawDeg = 0.f;
		ResetToStart();
		ChasePos = StartCm + FVector(-250, 0, 80);
		bWaitingForWorld = true;
		Sim.armed = false; bArmBlocked = true;
		return;
	}
	bRealWorld = false; bWaitingForWorld = false; AirStartM = 0;
	if (Real) Real->Unload();
	if (Field) { Field->SetActorHiddenInGame(false); Field->SetActorEnableCollision(true); StartCm = Field->StartCm; StartYawDeg = Field->StartYawDeg; }
	ResetToStart();
	ChasePos = StartCm + FVector(-250, 0, 80);
}

void AFpvQuad::ResetToStart()
{
	Sim.Reset(FromUe(StartCm + FVector(0, 0, 4.0)), -FMath::DegreesToRadians(StartYawDeg));
	bArmBlocked = true; FlightTime = 0; Accum = 0; KbThrottle = 0;
	ApplyPose();
	bAirHold = bRealWorld && AirStartM > 0;
}

void AFpvQuad::Say(const FString& S) { Notice = S; NoticeTime = GetWorld() ? GetWorld()->GetRealTimeSeconds() : 0; }

void AFpvQuad::HandleKeys(float Dt)
{
	APlayerController* PC = GetWorld()->GetFirstPlayerController();
	if (!PC) return;
	if (Radio.IsCalibrating())
	{
		if (PC->WasInputKeyJustPressed(EKeys::Enter) || PC->WasInputKeyJustPressed(EKeys::SpaceBar)) Radio.Next();
		if (PC->WasInputKeyJustPressed(EKeys::Escape)) { Radio.Cancel(); Say(TEXT("Calibration cancelled")); }
		return;
	}
	if (PC->WasInputKeyJustPressed(EKeys::Escape) || PC->WasInputKeyJustPressed(EKeys::M))   // not Tab: Alt+Tab would open it
	{
		bMenuOpen = !bMenuOpen; MenuHeld = 0;
		if (bMenuOpen && !Menu.IsValidIndex(MenuSel)) MenuSel = 1;
		return;
	}
	if (bMenuOpen) { HandleMenu(Dt); return; }
	if (PC->WasInputKeyJustPressed(EKeys::C)) { Radio.StartCalibration(); Sim.armed = false; }
	if (PC->WasInputKeyJustPressed(EKeys::R)) { ResetToStart(); Say(TEXT("Reset - arm switch OFF then ON to arm")); }
	if (PC->WasInputKeyJustPressed(EKeys::V)) CamMode = (CamMode + 1) % NumCams;
	if (PC->WasInputKeyJustPressed(EKeys::H)) bShowSticks = !bShowSticks;
	if (PC->WasInputKeyJustPressed(EKeys::O)) { bCleanView = !bCleanView; if (!bCleanView) Say(TEXT("OSD on")); }
	if (PC->WasInputKeyJustPressed(EKeys::T)) { bTurtleKey = !bTurtleKey; Say(bTurtleKey ? TEXT("Turtle mode ON - arm to flip over (T again for normal)") : TEXT("Turtle mode OFF")); }
	if (PC->WasInputKeyJustPressed(EKeys::F5)) { LoadSettings(); Say(TEXT("Settings reloaded")); }
	if (PC->WasInputKeyJustPressed(EKeys::K)) { bKeyboard = !bKeyboard; KbThrottle = 0; In.armed = false; Say(bKeyboard ? TEXT("Keyboard flying ON (Space arm, W/S throttle, A/D yaw, arrows)") : TEXT("Keyboard flying OFF")); }
	if (bKeyboard)
	{
		if (PC->WasInputKeyJustPressed(EKeys::SpaceBar)) { In.armed = !In.armed; if (In.armed) KbThrottle = 0; }
		KbThrottle = FMath::Clamp(KbThrottle + Dt * 0.7f * ((PC->IsInputKeyDown(EKeys::W) ? 1 : 0) - (PC->IsInputKeyDown(EKeys::S) ? 1 : 0)), 0.f, 1.f);
	}
}

void AFpvQuad::UpdateInputs(float Dt)
{
	if (bTestFlight)   // -FpvTestFlight: scripted sticks (take off, cruise, turn, flip, cut throttle), logged once a second
	{
		const double t = TestTime += Dt;
		In = fpv::Inputs(); In.armed = true;
		if (t < 1.0) In.throttle = 0;
		else if (t < 2.5) In.throttle = 0.6;
		else if (t < 4.0) { In.throttle = 0.35; In.pitch = 0.25; }
		else if (t < 9.0) { In.throttle = 0.42; In.yaw = 0.15; In.pitch = (Sim.PitchDeg() > -25 ? 0.15 : -0.15); }
		else if (t < 9.55) { In.throttle = 0.3; In.roll = 1.0; }
		else In.throttle = t < 11 ? 0.42 : 0.0;
		if (FMath::FloorToInt(t) != FMath::FloorToInt(t - Dt))
			UE_LOG(LogTemp, Display, TEXT("FPV test t=%.0f pos=(%.1f %.1f %.1f) m  %.0f km/h  pitch %.0f roll %.0f  armed %d crashed %d impact %.1f"), t, Sim.pos.x, Sim.pos.y, Sim.pos.z,
				fpv::Len(Sim.vel) * 3.6, Sim.PitchDeg(), Sim.RollDeg(), Sim.armed, Sim.crashed, Sim.crashImpact);
		return;
	}
	if (bKeyboard)
	{
		APlayerController* PC = GetWorld()->GetFirstPlayerController();
		auto K = [PC](const FKey& A, const FKey& B) { return PC ? float((PC->IsInputKeyDown(A) ? 1 : 0) - (PC->IsInputKeyDown(B) ? 1 : 0)) : 0.f; };
		In.roll = 0.45 * K(EKeys::Right, EKeys::Left); In.pitch = 0.45 * K(EKeys::Up, EKeys::Down); In.yaw = 0.5 * K(EKeys::D, EKeys::A);
		In.throttle = KbThrottle;
		In.turtle = bTurtleKey;
		if (!In.armed) bArmBlocked = false;
		return;
	}
	const bool bSwitch = Radio.ArmSwitch();
	if (!bSwitch) bArmBlocked = false;
	In.roll = Radio.Stick(FFpvRadio::Roll); In.pitch = Radio.Stick(FFpvRadio::Pitch); In.yaw = Radio.Stick(FFpvRadio::Yaw);
	In.throttle = Radio.Stick(FFpvRadio::Throttle);
	In.armed = bSwitch && !bArmBlocked && !Radio.IsCalibrating() && Radio.IsConnected();
	In.turtle = Radio.SwitchOn(FFpvRadio::Turtle) || bTurtleKey;
	// reset and camera switches act when flipped on
	const bool bReset = Radio.SwitchOn(FFpvRadio::Reset), bCam = Radio.SwitchOn(FFpvRadio::Camera);
	if (bReset && !bResetWas) { ResetToStart(); Say(TEXT("Reset - arm switch OFF then ON to arm")); }
	if (bCam && !bCamWas) CamMode = (CamMode + 1) % NumCams;
	bResetWas = bReset; bCamWas = bCam;
}

void AFpvQuad::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	const float Dt = FMath::Min(DeltaSeconds, 0.05f);
	Fps = FMath::Lerp(Fps, 1.f / FMath::Max(DeltaSeconds, 1e-4f), 0.05f);
	Radio.Poll(GetWorld()->GetRealTimeSeconds());
	if (!Radio.IsConnected() && !bKeyboard && !Radio.IsCalibrating()) In = fpv::Inputs();
	HandleKeys(Dt);

	if (bWaitingForWorld && Real && Real->IsActive() && Real->Ready())
	{
		bWaitingForWorld = false; StartCm = Real->StartCm; StartYawDeg = 0.f;
		AirStartM = Places.IsValidIndex(int32(PlaceIndex)) ? Places[int32(PlaceIndex)].Alt : 0.0;
		StartCm.Z += AirStartM * 100.0;
		ResetToStart();
		ChasePos = StartCm + FVector(-250, 0, 80);
		Say(FString::Printf(TEXT("%s - arm to fly"), Places.IsValidIndex(int32(PlaceIndex)) ? *Places[int32(PlaceIndex)].Name : TEXT("Real world")));
	}
	if (!bMenuOpen && !bWaitingForWorld)   // the menu pauses the sim, and so does a world still streaming in
	{
		UpdateInputs(Dt);
		// physics at a fixed rate; one broad-phase overlap per frame decides whether the contact traces are needed
		const double T0 = FPlatformTime::Seconds();
		FUeWorld W; W.World = GetWorld(); W.Params = FCollisionQueryParams(SCENE_QUERY_STAT(FpvQuad), true, this);
		const double Reach = 0.45 + fpv::Len(Sim.vel) * Dt * 1.5;
		W.bNear = bRealWorld || GetWorld()->OverlapBlockingTestByChannel(ToUe(Sim.pos), FQuat::Identity, ECC_Visibility, FCollisionShape::MakeSphere(Reach * 100.0), W.Params);
		const double H = 1.0 / PhysicsHz;
		Accum += Dt;
		const bool bWasCrashed = Sim.crashed;
		while (Accum >= H) { Sim.Step(H, In, &W); Accum -= H; }
		if (bAirHold)   // air start: hang in place until armed, then it is all physics
		{
			if (Sim.armed) bAirHold = false;
			else { Sim.pos = FromUe(StartCm + FVector(0, 0, 4.0)); Sim.vel = fpv::V3(); Sim.omega = fpv::V3(); }
		}
		PhysMs = FMath::Lerp(PhysMs, float((FPlatformTime::Seconds() - T0) * 1e3), 0.05f);
		if (Sim.crashed && !bWasCrashed) Say(FString::Printf(TEXT("CRASH  %.0f km/h impact - arm switch OFF, then ON (R resets)"), Sim.crashImpact * 3.6));
		if (Sim.armed) FlightTime += Dt;
		if (Sim.pos.z < FromUe(StartCm).z - (bRealWorld ? 300.0 : 200.0)) ResetToStart();   // fell out of the world (or through a tile)
		UpdateProps(Dt);
	}
	ApplyPose();
	UpdateCameras(Dt);
}

void AFpvQuad::ApplyPose()
{
	const FVector X = ToUeDir(Sim.Forward()), Z = ToUeDir(Sim.Up());
	SetActorLocationAndRotation(ToUe(Sim.pos), FRotationMatrix::MakeFromXZ(X, Z).ToQuat());
}

void AFpvQuad::UpdateProps(float Dt)
{
	// the blades turn at the real rpm (so on screen they alias against the frame rate, like a camera's shutter does);
	// once spinning they are drawn as the translucent blur disc, which fades in with rpm
	const FLinearColor Tint(0.6f, 0.1f, 0.35f);
	for (int32 i = 0; i < 4; ++i)
	{
		FProp& P = Props[i];
		if (!P.Spin) continue;
		const double W = Sim.motor[i];
		P.Angle = FMath::Fmod(P.Angle + (Sim.turtle ? -1.0 : 1.0) * W * Sim.P.maxRpm / 60.0 * 360.0 * Dt, 360.0);   // turtle runs them backwards
		P.Spin->SetRelativeRotation(FRotator(0.f, float(-fpv::QuadSim::SpinDir(i) * P.Angle), 0.f));
		const bool bSpinning = W > 0.015;
		P.Blades->SetVisibility(!bSpinning);
		P.Disc->SetVisibility(bSpinning);
		if (P.DiscMat) { P.DiscMat->SetVectorParameterValue(TEXT("Color"), Tint); P.DiscMat->SetScalarParameterValue(TEXT("Opacity"), float(FMath::Clamp(0.25 + W * 0.7, 0.0, 0.6))); }
	}
}

void AFpvQuad::UpdateCameras(float Dt)
{
	const FVector Pos = GetActorLocation();
	// chase: behind and above along the direction of travel (or the heading when slow), smoothed
	const FVector V = ToUe(Sim.vel);
	FVector Fwd = ToUeDir(Sim.Forward()); Fwd.Z = 0;
	if (V.Size2D() > 300.f) Fwd = FVector(V.X, V.Y, 0);
	Fwd = Fwd.GetSafeNormal(); if (Fwd.IsNearlyZero()) Fwd = FVector::ForwardVector;
	const FVector Want = Pos - Fwd * 260.f + FVector(0, 0, 90.f);
	ChasePos = FMath::VInterpTo(ChasePos, Want, Dt, 6.f);
	ChaseCam->SetWorldLocationAndRotation(ChasePos, (Pos - ChasePos).Rotation());
	LosCam->SetWorldLocationAndRotation(StartCm + FVector(-500.f, 300.f, 170.f), (Pos - (StartCm + FVector(-500.f, 300.f, 170.f))).Rotation());
	FpvCam->SetActive(CamMode == CamFpv); ChaseCam->SetActive(CamMode == CamChase); LosCam->SetActive(CamMode == CamLos);
	// in the FPV view the quad stays visible (props in the corners of the image) except the camera's own housing
	const bool bFpv = CamMode == CamFpv;
	Body->SetVisibility(!bFpv || bPropsInView, true);
	if (CamBlock) CamBlock->SetVisibility(!bFpv);
	for (const FProp& P : Props) if (P.Spin && Body->IsVisible()) { const bool bS = Sim.motor[&P - Props] > 0.015; P.Blades->SetVisibility(!bS); P.Disc->SetVisibility(bS); }
}
