#include "FpvWorld.h"
#include "ProceduralMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"

namespace
{
	uint32 Hash2(int32 a, int32 b) { uint32 h = uint32(a) * 374761393u + uint32(b) * 668265263u; h = (h ^ (h >> 13)) * 1274126177u; return h ^ (h >> 16); }
	float Rand01(int32 a, int32 b) { return (Hash2(a, b) & 0xFFFF) / 65535.f; }
	float Smooth(float t) { return t * t * (3.f - 2.f * t); }
	float SS(float e0, float e1, float x) { return Smooth(FMath::Clamp((x - e0) / (e1 - e0), 0.f, 1.f)); }
	float VNoise(float x, float y)
	{
		const int32 xi = FMath::FloorToInt(x), yi = FMath::FloorToInt(y);
		const float fx = Smooth(x - xi), fy = Smooth(y - yi);
		return FMath::Lerp(FMath::Lerp(Rand01(xi, yi), Rand01(xi + 1, yi), fx), FMath::Lerp(Rand01(xi, yi + 1), Rand01(xi + 1, yi + 1), fx), fy);
	}
	float Fbm(float x, float y) { float s = 0, a = 0.5f, f = 1.f; for (int i = 0; i < 4; ++i) { s += a * VNoise(x * f, y * f); f *= 2.f; a *= 0.5f; } return s; }

	// ---- layout, metres (Unreal x forward, y right). Start pad at the origin facing +x, toward the gate track.
	const FVector2D TrackC(70.f, 0.f); const float TrackA = 55.f, TrackB = 35.f;   // gate track: an oval
	const FVector2D BandoC(-130.f, 100.f);                                          // parking structure corner
	const FVector2D TowerC(200.f, -170.f);
	const FVector2D ForestC(-40.f, -130.f);                                         // dense tree patch for gaps
	const FVector2D BoxesC(20.f, 60.f);                                             // shipping containers

	float HeightM(float x, float y)
	{
		const float r = FMath::Sqrt(x * x + y * y);
		const float Hills = (Fbm(x / 180.f + 3.f, y / 180.f + 11.f) - 0.45f) * 45.f * SS(220.f, 650.f, r) + 6.f * SS(260.f, 700.f, r);
		const float Mount = SS(950.f, 1350.f, r) * (70.f + 160.f * Fbm(x / 260.f + 7.f, y / 260.f + 3.f));
		return Hills + Mount;
	}
	bool Reserved(float x, float y, float m)   // keep trees and rocks out of the flying features
	{
		const float e = FMath::Sqrt(FMath::Square((x - TrackC.X) / (TrackA + m)) + FMath::Square((y - TrackC.Y) / (TrackB + m)));
		if (e < 1.25f) return true;
		if (FMath::Sqrt(x * x + y * y) < 30.f + m) return true;
		if (x > BandoC.X - 10.f - m && x < BandoC.X + 50.f + m && y > BandoC.Y - 10.f - m && y < BandoC.Y + 34.f + m) return true;
		if (FVector2D(x, y).Equals(TowerC, 25.f + m) || FVector2D(x - BoxesC.X, y - BoxesC.Y).Size() < 22.f + m) return true;
		return false;
	}
}

float AFpvWorld::GroundZ(float Xcm, float Ycm) { return HeightM(Xcm / 100.f, Ycm / 100.f) * 100.f; }

AFpvWorld::AFpvWorld()
{
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;
}

UInstancedStaticMeshComponent* AFpvWorld::Ism(UStaticMesh* Mesh, const FLinearColor& Color, bool bCollide, bool bShadow)
{
	UInstancedStaticMeshComponent* I = NewObject<UInstancedStaticMeshComponent>(this);
	I->SetStaticMesh(Mesh);
	I->SetupAttachment(Root);
	I->SetMobility(EComponentMobility::Static);
	if (bCollide) I->SetCollisionProfileName(TEXT("BlockAll")); else I->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	I->SetCastShadow(bShadow);
	static UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	if (Base) { UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(Base, this); M->SetVectorParameterValue(TEXT("Color"), Color); I->SetMaterial(0, M); }
	I->RegisterComponent();
	return I;
}

void AFpvWorld::Box(UInstancedStaticMeshComponent* I, const FVector& CentreCm, const FVector& SizeCm, const FRotator& Rot)
{
	I->AddInstance(FTransform(Rot, CentreCm, SizeCm / 100.f), true);
}

void AFpvWorld::Build()
{
	Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	Cyl = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	Cone = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cone.Cone"));
	const double T0 = FPlatformTime::Seconds();
	BuildTerrain();
	BuildTrack();
	BuildBando(FVector(BandoC.X * 100.f, BandoC.Y * 100.f, GroundZ(BandoC.X * 100.f, BandoC.Y * 100.f)));
	BuildTower(FVector(TowerC.X * 100.f, TowerC.Y * 100.f, GroundZ(TowerC.X * 100.f, TowerC.Y * 100.f)), 60.f);
	BuildScatter();
	// start pad
	UInstancedStaticMeshComponent* Pad = Ism(Cube, FLinearColor(0.25f, 0.25f, 0.27f), true);
	Box(Pad, FVector(0, 0, -4.f), FVector(150.f, 150.f, 10.f));
	StartCm = FVector(0, 0, 1.f); StartYawDeg = 0.f;
	UE_LOG(LogTemp, Display, TEXT("FPV world built in %.1f s"), FPlatformTime::Seconds() - T0);
}

void AFpvWorld::BuildTerrain()
{
	const float Half = 1500.f, Step = 7.5f;   // metres
	const int32 N = FMath::RoundToInt(2 * Half / Step) + 1;
	TArray<FVector> V; TArray<int32> T; TArray<FVector> Nr; TArray<FLinearColor> Col;
	V.Reserve(N * N); Nr.Reserve(N * N); Col.Reserve(N * N); T.Reserve((N - 1) * (N - 1) * 6);
	for (int32 j = 0; j < N; ++j)
		for (int32 i = 0; i < N; ++i)
		{
			const float x = -Half + i * Step, y = -Half + j * Step, h = HeightM(x, y), r = FMath::Sqrt(x * x + y * y);
			V.Add(FVector(x, y, h) * 100.f);
			const float dx = HeightM(x + 1.f, y) - HeightM(x - 1.f, y), dy = HeightM(x, y + 1.f) - HeightM(x, y - 1.f);
			const FVector Normal = FVector(-dx, -dy, 2.f).GetSafeNormal();
			Nr.Add(Normal);
			// grass with large and small scale variation; per-vertex speckle gives a speed / height cue
			const float n1 = Fbm(x / 30.f, y / 30.f), n2 = Rand01(i, j);
			FLinearColor C = FMath::Lerp(FLinearColor(0.018f, 0.045f, 0.012f), FLinearColor(0.06f, 0.10f, 0.025f), n1);
			C *= 0.7f + 0.6f * n2;
			if (r < 200.f && (FMath::FloorToInt(x / 10.f) & 1)) C *= 1.18f;                    // mown stripes on the home field
			if (r > 260.f && r < 950.f)                                                          // patchwork fields
			{
				const int32 cx = FMath::FloorToInt(x / 130.f), cy = FMath::FloorToInt(y / 110.f);
				const float k = Rand01(cx + 91, cy - 17);
				const FLinearColor Crop = k < 0.3f ? FLinearColor(0.16f, 0.13f, 0.045f) : k < 0.5f ? FLinearColor(0.07f, 0.045f, 0.025f) : k < 0.7f ? FLinearColor(0.03f, 0.08f, 0.012f) : C;
				C = Crop * (0.85f + 0.3f * n2);
			}
			const float e = FMath::Sqrt(FMath::Square((x - TrackC.X) / TrackA) + FMath::Square((y - TrackC.Y) / TrackB));
			if (FMath::Abs(e - 1.f) * 45.f < 2.5f) C = FLinearColor(0.09f, 0.06f, 0.035f) * (0.8f + 0.4f * n2);   // dirt racing line
			if (h > 40.f || Normal.Z < 0.8f) C = FMath::Lerp(C, FLinearColor(0.09f, 0.085f, 0.08f) * (0.8f + 0.4f * n2), SS(40.f, 90.f, h) + (1.f - SS(0.65f, 0.85f, Normal.Z)));
			if (h > 170.f) C = FMath::Lerp(C, FLinearColor(0.8f, 0.82f, 0.85f), SS(170.f, 200.f, h));
			Col.Add(C);
		}
	for (int32 j = 0; j + 1 < N; ++j)
		for (int32 i = 0; i + 1 < N; ++i)
		{
			const int32 a = j * N + i, b = a + 1, c = a + N, d = c + 1;
			T.Append({ a, c, b, b, c, d });
		}
	UProceduralMeshComponent* G = NewObject<UProceduralMeshComponent>(this);
	G->SetupAttachment(Root);
	G->bUseAsyncCooking = false;
	G->CreateMeshSection_LinearColor(0, V, T, Nr, TArray<FVector2D>(), Col, TArray<FProcMeshTangent>(), true);
	G->SetCollisionProfileName(TEXT("BlockAll"));
	if (UMaterialInterface* VM = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"))) G->SetMaterial(0, VM);
	G->RegisterComponent();
}

void AFpvWorld::BuildTrack()
{
	UInstancedStaticMeshComponent* Frame = Ism(Cube, FLinearColor(1.f, 0.22f, 0.02f), true);
	UInstancedStaticMeshComponent* Stilt = Ism(Cube, FLinearColor(0.12f, 0.12f, 0.13f), true);
	const int32 NumGates = 12;
	const float W = 200.f, Hgt = 160.f, Bar = 12.f;   // cm: opening and tube size
	for (int32 g = 0; g < NumGates; ++g)
	{
		const float t = 2 * PI * g / NumGates;
		const FVector2D P(TrackC.X + TrackA * FMath::Cos(t), TrackC.Y + TrackB * FMath::Sin(t));
		const FVector2D Tan(-TrackA * FMath::Sin(t), TrackB * FMath::Cos(t));
		const FRotator Rot(0.f, FMath::RadiansToDegrees(FMath::Atan2(Tan.Y, Tan.X)), 0.f);
		const float Lift = (g % 4 == 2) ? 300.f : 0.f;   // every 4th gate sits on stilts
		const FVector Base(P.X * 100.f, P.Y * 100.f, GroundZ(P.X * 100.f, P.Y * 100.f) + Lift);
		auto L = [&](float side, float up) { return Base + Rot.RotateVector(FVector(0.f, side, up)); };
		for (float s : { -1.f, 1.f })
		{
			Box(Frame, L(s * (W + Bar) * 0.5f, (Hgt + Bar) * 0.5f), FVector(Bar, Bar, Hgt + 2 * Bar), Rot);
			if (Lift > 0) Box(Stilt, L(s * (W + Bar) * 0.5f, -Lift * 0.5f), FVector(Bar * 0.8f, Bar * 0.8f, Lift), Rot);
		}
		Box(Frame, L(0.f, Hgt + Bar * 0.5f), FVector(Bar, W + 2 * Bar, Bar), Rot);
		if (Lift > 0) Box(Frame, L(0.f, -Bar * 0.5f), FVector(Bar, W + 2 * Bar, Bar), Rot);
	}
	// a ladder: two gates stacked, and a flag line to weave
	const FVector2D Lad(TrackC.X, TrackC.Y);
	for (int32 k = 0; k < 2; ++k)
	{
		const FVector Base(Lad.X * 100.f, Lad.Y * 100.f, GroundZ(Lad.X * 100.f, Lad.Y * 100.f) + k * (Hgt + Bar));
		for (float s : { -1.f, 1.f }) Box(Frame, Base + FVector(0, s * (W + Bar) * 0.5f, (Hgt + Bar) * 0.5f), FVector(Bar, Bar, Hgt + 2 * Bar));
		Box(Frame, Base + FVector(0, 0, Hgt + Bar * 0.5f), FVector(Bar, W + 2 * Bar, Bar));
	}
	UInstancedStaticMeshComponent* Flags = Ism(Cyl, FLinearColor(0.9f, 0.9f, 0.95f), true);
	for (int32 k = 0; k < 6; ++k)
	{
		const float x = TrackC.X - 30.f + k * 12.f, y = TrackC.Y + ((k & 1) ? 8.f : -8.f);
		Box(Flags, FVector(x * 100.f, y * 100.f, GroundZ(x * 100.f, y * 100.f) + 150.f), FVector(8.f, 8.f, 300.f));
	}
}

void AFpvWorld::BuildBando(const FVector& O)
{
	UInstancedStaticMeshComponent* Concrete = Ism(Cube, FLinearColor(0.30f, 0.29f, 0.27f), true);
	UInstancedStaticMeshComponent* Paint = Ism(Cube, FLinearColor(0.55f, 0.08f, 0.12f), true);
	UInstancedStaticMeshComponent* Paint2 = Ism(Cube, FLinearColor(0.05f, 0.25f, 0.55f), true);
	const float Lx = 4000.f, Ly = 2400.f, Floor = 320.f, Slab = 30.f, Pil = 50.f;
	const int32 Levels = 4;
	auto Slab3 = [&](float x0, float x1, float y0, float y1, float z) { if (x1 > x0 && y1 > y0) Box(Concrete, O + FVector((x0 + x1) * 0.5f, (y0 + y1) * 0.5f, z), FVector(x1 - x0, y1 - y0, Slab)); };
	for (int32 k = 1; k <= Levels; ++k)
	{
		const float z = k * Floor;
		const float h0 = (k & 1) ? 800.f : 2400.f, h1 = h0 + 800.f;   // an 8 x 8 m hole, alternating ends, to drop through
		Slab3(0, Lx, 0, 800.f, z); Slab3(0, Lx, 1600.f, Ly, z);
		Slab3(0, h0, 800.f, 1600.f, z); Slab3(h1, Lx, 800.f, 1600.f, z);
		// parapets with gaps every other bay
		for (int32 b = 0; b < 5; ++b)
			if ((b + k) & 1)
				for (float y : { 0.f, Ly }) Box(b == 2 ? Paint : Concrete, O + FVector(b * 800.f + 400.f, y, z + Slab * 0.5f + 50.f), FVector(800.f, 20.f, 100.f));
	}
	for (int32 i = 0; i <= 5; ++i)
		for (int32 j = 0; j <= 3; ++j)
			Box(Concrete, O + FVector(i * 800.f, j * 800.f, Levels * Floor * 0.5f), FVector(Pil, Pil, Levels * Floor));
	// stair core and some painted walls
	Box(Concrete, O + FVector(Lx - 200.f, Ly - 200.f, (Levels * Floor + 150.f) * 0.5f), FVector(400.f, 400.f, Levels * Floor + 150.f));
	Box(Paint2, O + FVector(1200.f, 0.f, Floor * 0.5f), FVector(780.f, 25.f, Floor - 20.f));
	Box(Paint, O + FVector(0.f, 1200.f, Floor * 1.5f), FVector(25.f, 780.f, Floor - 20.f));
}

void AFpvWorld::BuildTower(const FVector& B, float HeightM)
{
	UInstancedStaticMeshComponent* Red = Ism(Cube, FLinearColor(0.6f, 0.05f, 0.03f), true);
	UInstancedStaticMeshComponent* White = Ism(Cube, FLinearColor(0.8f, 0.8f, 0.8f), true);
	const float S = 400.f, Hc = HeightM * 100.f, Seg = 400.f, T = 18.f;
	for (float sx : { -1.f, 1.f }) for (float sy : { -1.f, 1.f }) Box(Red, B + FVector(sx * S * 0.5f, sy * S * 0.5f, Hc * 0.5f), FVector(T * 1.5f, T * 1.5f, Hc));
	const float Diag = FMath::Sqrt(S * S + Seg * Seg), Ang = FMath::RadiansToDegrees(FMath::Atan2(Seg, S));
	for (int32 k = 0; k * Seg < Hc; ++k)
	{
		UInstancedStaticMeshComponent* M = (k / 2) & 1 ? White : Red;
		const float z = k * Seg;
		for (int32 side = 0; side < 4; ++side)
		{
			const float Yaw = side * 90.f;
			const FRotator R(0.f, Yaw, 0.f);
			const FVector Out = R.RotateVector(FVector(S * 0.5f, 0.f, 0.f));
			Box(M, B + Out + FVector(0, 0, z + Seg), FVector(T, S, T), R);                                     // horizontal
			Box(M, B + Out + FVector(0, 0, z + Seg * 0.5f), FVector(T, Diag, T), FRotator(0.f, Yaw, (side & 1) ? Ang : -Ang)); // diagonal brace
		}
	}
}

void AFpvWorld::BuildScatter()
{
	UInstancedStaticMeshComponent* Trunk = Ism(Cyl, FLinearColor(0.09f, 0.06f, 0.035f), true);
	UInstancedStaticMeshComponent* Pine = Ism(Cone, FLinearColor(0.02f, 0.07f, 0.025f), true);
	UInstancedStaticMeshComponent* Leafy = Ism(Cone, FLinearColor(0.06f, 0.13f, 0.03f), true);
	UInstancedStaticMeshComponent* Rock = Ism(Cube, FLinearColor(0.2f, 0.19f, 0.18f), true);
	UInstancedStaticMeshComponent* Clump = Ism(Cone, FLinearColor(0.02f, 0.05f, 0.012f), false, false);
	auto Tree = [&](float x, float y, float s)
	{
		const FVector G(x * 100.f, y * 100.f, GroundZ(x * 100.f, y * 100.f) - 20.f);
		const float H = 700.f * s, Wd = 380.f * s;
		Box(Trunk, G + FVector(0, 0, H * 0.25f), FVector(30.f * s, 30.f * s, H * 0.5f));
		UInstancedStaticMeshComponent* C = Rand01(int32(x * 7), int32(y * 13)) < 0.6f ? Pine : Leafy;
		Box(C, G + FVector(0, 0, H * 0.55f), FVector(Wd, Wd, H * 0.7f));
		Box(C, G + FVector(0, 0, H * 0.85f), FVector(Wd * 0.7f, Wd * 0.7f, H * 0.45f));
	};
	int32 Seed = 1;
	auto R = [&Seed]() { return Rand01(Seed++, 77); };
	for (int32 n = 0; n < 1400; )   // scattered trees in tree lines and clusters
	{
		const float a = R() * 2 * PI, r = 60.f + R() * 900.f;
		const float x = r * FMath::Cos(a), y = r * FMath::Sin(a);
		if (Reserved(x, y, 6.f) || Fbm(x / 70.f, y / 70.f) < 0.5f) { if (R() > 0.995f) ++n; continue; }
		Tree(x, y, 0.7f + 0.8f * R()); ++n;
	}
	for (int32 n = 0; n < 260; ++n)   // dense patch for gap shots
	{
		const float x = ForestC.X + (R() - 0.5f) * 120.f, y = ForestC.Y + (R() - 0.5f) * 80.f;
		if (!Reserved(x, y, 2.f)) Tree(x, y, 0.8f + 0.6f * R());
	}
	for (int32 n = 0; n < 1500; ++n)
	{
		const float a = R() * 2 * PI, r = 20.f + R() * 600.f, x = r * FMath::Cos(a), y = r * FMath::Sin(a);
		if (Reserved(x, y, 2.f)) continue;
		const float s = 40.f + 120.f * R() * R();
		Box(Rock, FVector(x * 100.f, y * 100.f, GroundZ(x * 100.f, y * 100.f) + s * 0.2f), FVector(s, s * (0.6f + 0.6f * R()), s * 0.6f), FRotator(R() * 20.f, R() * 360.f, R() * 20.f));
	}
	for (int32 n = 0; n < 12000; ++n)   // grass tufts: no collision, they only give the eye something close to the ground
	{
		const float a = R() * 2 * PI, r = 6.f + 450.f * FMath::Sqrt(R()), x = r * FMath::Cos(a), y = r * FMath::Sin(a);
		const float s = 20.f + 40.f * R(), h = 15.f + 35.f * R();
		Box(Clump, FVector(x * 100.f, y * 100.f, GroundZ(x * 100.f, y * 100.f) + h * 0.5f - 3.f), FVector(s, s, h), FRotator(0.f, R() * 360.f, 0.f));
	}
	// shipping containers, some stacked
	const FLinearColor Cc[] = { { 0.5f, 0.12f, 0.05f }, { 0.05f, 0.18f, 0.4f }, { 0.15f, 0.3f, 0.12f }, { 0.55f, 0.4f, 0.05f } };
	for (int32 k = 0; k < 7; ++k)
	{
		UInstancedStaticMeshComponent* C = Ism(Cube, Cc[k % 4], true);
		const bool bTop = k >= 5;                  // 5 and 6 sit on 0 and 2
		const int32 Slot = bTop ? (k - 5) * 2 : k;
		const float x = BoxesC.X + (Slot % 3) * 3.2f, y = BoxesC.Y + (Slot / 3) * 9.f, Z = GroundZ(x * 100.f, y * 100.f);
		Box(C, FVector(x * 100.f, y * 100.f, Z + 130.f + (bTop ? 260.f : 0.f)), FVector(244.f, 610.f, 260.f), FRotator(0.f, bTop ? 8.f : 0.f, 0.f));
	}
}
