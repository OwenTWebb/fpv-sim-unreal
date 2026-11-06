#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "FpvHUD.generated.h"

/** Betaflight-style OSD: battery, timer, speed, altitude, throttle, horizon, sticks, warnings, and the radio calibration screen. */
UCLASS()
class FPVSIM_API AFpvHUD : public AHUD
{
	GENERATED_BODY()

public:
	virtual void DrawHUD() override;

private:
	void Txt(const FString& S, float X, float Y, const FLinearColor& C, float Scale = 1.f, bool bCentre = false);
	float U = 1.f;   // pixels per 1080p pixel
};
