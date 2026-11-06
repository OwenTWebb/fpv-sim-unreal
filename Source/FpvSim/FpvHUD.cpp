#include "FpvHUD.h"
#include "FpvQuad.h"
#include "FpvRealWorld.h"
#include "CanvasItem.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"

void AFpvHUD::Txt(const FString& S, float X, float Y, const FLinearColor& C, float Scale, bool bCentre)
{
	FCanvasTextItem T(FVector2D(X, Y), FText::FromString(S), GEngine->GetMediumFont(), C);
	T.Scale = FVector2D(FMath::Max(Scale, 0.9f) * U * 1.4f);
	T.bOutlined = true; T.OutlineColor = FLinearColor(0, 0, 0, 0.9f);
	T.bCentreX = bCentre;
	Canvas->DrawItem(T);
}

void AFpvHUD::DrawHUD()
{
	Super::DrawHUD();
	AFpvQuad* Q = nullptr;
	for (TActorIterator<AFpvQuad> It(GetWorld()); It; ++It) { Q = *It; break; }
	if (!Q || !Canvas) return;
	const float W = Canvas->SizeX, H = Canvas->SizeY, Cx = W * 0.5f, Cy = H * 0.5f;
	U = H / 1080.f;
	const fpv::QuadSim& S = Q->Sim;
	const FLinearColor White(1, 1, 1), Warn(1.f, 0.85f, 0.2f), Red(1.f, 0.3f, 0.25f), Green(0.4f, 1.f, 0.5f), Dim(0.8f, 0.85f, 0.9f, 0.8f);
	const double Now = GetWorld()->GetRealTimeSeconds();
	const bool bBlink = FMath::Fmod(Now, 0.8) < 0.5;

	// ---- radio calibration screen
	FFpvRadio& R = Q->Radio;
	if (R.IsCalibrating())
	{
		DrawRect(FLinearColor(0, 0, 0, 0.7f), Cx - 520 * U, Cy - 300 * U, 1040 * U, 640 * U);
		Txt(TEXT("RADIO CALIBRATION"), Cx, Cy - 285 * U, Warn, 1.4f, true);
		Txt(R.IsConnected() ? R.DeviceName : FString(TEXT("no radio connected")), Cx, Cy - 245 * U, Dim, 0.8f, true);
		Txt(R.Prompt(), Cx, Cy - 200 * U, White, 1.1f, true);
		Txt(TEXT("then press ENTER        (ESC cancels)"), Cx, Cy - 160 * U, Dim, 0.85f, true);
		for (int32 i = 0; i < FFpvRadio::NumAxes; ++i)   // live channels, so you can see every switch being read
		{
			const float y = Cy - 115 * U + i * 32 * U;
			Txt(FString::Printf(TEXT("ch %d"), i + 1), Cx - 420 * U, y, Dim, 0.8f);
			DrawRect(FLinearColor(1, 1, 1, 0.15f), Cx - 300 * U, y + 4 * U, 600 * U, 18 * U);
			DrawRect(Green, Cx - 300 * U + R.Raw[i] * 594 * U, y + 2 * U, 6 * U, 22 * U);
		}
		Txt(TEXT("buttons"), Cx - 420 * U, Cy + 150 * U, Dim, 0.8f);
		for (int32 b = 0; b < 32; ++b)
			DrawRect((R.Buttons >> b) & 1u ? Green : FLinearColor(1, 1, 1, 0.15f), Cx - 300 * U + b * 18.75f * U, Cy + 156 * U, 14 * U, 14 * U);
		Txt(FString::Printf(TEXT("arm %s   turtle %s   reset %s   camera %s"), *R.SwitchName(FFpvRadio::Arm), *R.SwitchName(FFpvRadio::Turtle), *R.SwitchName(FFpvRadio::Reset), *R.SwitchName(FFpvRadio::Camera)),
			Cx, Cy + 195 * U, Dim, 0.8f, true);
		if (!R.CalMessage.IsEmpty()) Txt(R.CalMessage, Cx, Cy + 240 * U, Warn, 0.9f, true);
		return;
	}

	// ---- real world still streaming in
	if (Q->bWaitingForWorld && !Q->bMenuOpen)
	{
		const FString WS = Q->Real ? Q->Real->Status : FString();
		DrawRect(FLinearColor(0, 0, 0, 0.6f), Cx - 420 * U, Cy - 60 * U, 840 * U, 120 * U);
		Txt(Q->Places.IsValidIndex(int32(Q->PlaceIndex)) ? Q->Places[int32(Q->PlaceIndex)].Name : FString(TEXT("Real world")), Cx, Cy - 45 * U, Warn, 1.2f, true);
		Txt(WS, Cx, Cy, White, 0.95f, true);
		return;
	}

	// ---- settings menu (the sim is paused behind it)
	if (Q->bMenuOpen)
	{
		const float RowH = 34 * U, PW = 820 * U, X0 = Cx - PW * 0.5f, Top = 60 * U;
		const int32 N = Q->Menu.Num(), Rows = FMath::Max(5, int32((H - Top - 170 * U) / RowH));
		const int32 First = FMath::Clamp(Q->MenuSel - Rows / 2, 0, FMath::Max(0, N - Rows));
		DrawRect(FLinearColor(0, 0, 0, 0.72f), X0 - 20 * U, Top - 50 * U, PW + 40 * U, Rows * RowH + 150 * U);
		Txt(TEXT("SETTINGS   (sim paused)"), Cx, Top - 40 * U, Warn, 1.2f, true);
		for (int32 k = 0; k < Rows && First + k < N; ++k)
		{
			const FFpvMenuItem& It = Q->Menu[First + k];
			const float y = Top + k * RowH;
			if (It.Kind == FFpvMenuItem::Heading) { Txt(It.Label, X0, y + 4 * U, Warn, 0.85f); continue; }
			const bool bSel = First + k == Q->MenuSel;
			if (bSel) DrawRect(FLinearColor(0.3f, 0.6f, 1.f, 0.35f), X0 - 8 * U, y, PW + 16 * U, RowH - 2 * U);
			Txt(It.Label, X0 + 24 * U, y + 3 * U, It.Kind == FFpvMenuItem::Action ? Green : White);
			const FString V = Q->MenuValue(It);
			if (!V.IsEmpty())
			{
				float TW = 0, TH = 0; GetTextSize(V, TW, TH, GEngine->GetMediumFont(), 1.f * U * 1.4f);
				Txt(bSel && It.Kind == FFpvMenuItem::Number ? TEXT("<") : TEXT(""), X0 + PW - TW - 40 * U, y + 3 * U, Dim);
				Txt(V, X0 + PW - TW - 16 * U, y + 3 * U, It.Kind == FFpvMenuItem::Toggle ? (*It.Flag ? Green : Dim) : White);
				if (bSel && It.Kind == FFpvMenuItem::Number) Txt(TEXT(">"), X0 + PW - 6 * U, y + 3 * U, Dim);
			}
		}
		if (First > 0) Txt(TEXT("..."), Cx, Top - 14 * U, Dim, 0.9f, true);
		if (First + Rows < N) Txt(TEXT("..."), Cx, Top + Rows * RowH - 6 * U, Dim, 0.9f, true);
		Txt(TEXT("Up/Down select    Left/Right change (Shift x10)    Enter toggle / do    Esc or M back"), Cx, Top + Rows * RowH + 26 * U, Dim, 0.9f, true);
		Txt(TEXT("radio: pitch stick = select, roll stick = change.   Changes apply at once; Save keeps them."), Cx, Top + Rows * RowH + 58 * U, Dim, 0.9f, true);
		if (Now - Q->NoticeTime < 4.0) Txt(Q->Notice, Cx, Top + Rows * RowH + 92 * U, Warn, 1.0f, true);
		return;
	}

	// ---- clean recording view: nothing but the data credits (Google / Cesium draw their own on-screen credits)
	if (Q->bCleanView)
	{
		return;
	}
	if (Q->bAirHold) Txt(TEXT("AIR START - holding position until you arm"), Cx, H * 0.5f + 160 * U, Warn, 0.9f, true);

	// ---- crosshair and artificial horizon (FPV view only)
	if (Q->CamMode == AFpvQuad::CamFpv)
	{
		DrawLine(Cx - 14 * U, Cy, Cx + 14 * U, Cy, White, 2.f); DrawLine(Cx, Cy - 8 * U, Cx, Cy + 8 * U, White, 2.f);
		// horizon: where the true horizon sits in the camera image (pinhole projection), clamped to the middle of the screen
		const float Roll = FMath::DegreesToRadians(S.RollDeg());
		const float Focal = Cx / FMath::Tan(FMath::DegreesToRadians(Q->CamFovDeg * 0.5f));
		const float Elev = FMath::DegreesToRadians(FMath::Clamp(float(S.PitchDeg()) + Q->CamUptiltDeg, -60.f, 60.f));
		const float Off = FMath::Clamp(Focal * FMath::Tan(Elev), -H * 0.3f, H * 0.3f);
		const FVector2D Dir(FMath::Cos(Roll), -FMath::Sin(Roll)), Nrm(-Dir.Y, Dir.X);
		const FVector2D Ctr = FVector2D(Cx, Cy) + Nrm * Off;
		const float Len = 130 * U;
		DrawLine(Ctr.X - Dir.X * Len, Ctr.Y - Dir.Y * Len, Ctr.X - Dir.X * Len * 0.3f, Ctr.Y - Dir.Y * Len * 0.3f, Dim, 2.f);
		DrawLine(Ctr.X + Dir.X * Len * 0.3f, Ctr.Y + Dir.Y * Len * 0.3f, Ctr.X + Dir.X * Len, Ctr.Y + Dir.Y * Len, Dim, 2.f);
	}

	// ---- corners
	const float M = 40 * U;
	const double Cell = S.voltage / S.P.batteryCells;
	const FLinearColor BattC = Cell < 3.3 ? Red : Cell < 3.5 ? Warn : White;
	Txt(FString::Printf(TEXT("%.1fV  %.2fV"), S.voltage, Cell), M, M, BattC);
	Txt(FString::Printf(TEXT("%3.0fA   %4.0fmAh"), S.current, S.mahUsed), M, M + 32 * U, White);
	const int32 Secs = int32(Q->FlightTime);
	Txt(FString::Printf(TEXT("%d:%02d"), Secs / 60, Secs % 60), W - M - 90 * U, M, White);
	Txt(FString::Printf(TEXT("%.0f fps  phys %.2f ms"), Q->Fps, Q->PhysMs), W - M - 260 * U, M + 32 * U, Dim, 0.7f);
	const double Spd = fpv::Len(S.vel) * 3.6;
	const double Alt = S.pos.z - Q->StartCm.Z / 100.0 + Q->AirStartM;   // above the ground at the start point
	Txt(FString::Printf(TEXT("%3.0f KM/H"), Spd), M, H - M - 64 * U, White);
	Txt(FString::Printf(TEXT("ALT %4.1f M"), Alt), M, H - M - 32 * U, White);
	Txt(FString::Printf(TEXT("THR %3.0f%%"), Q->In.throttle * 100.0), W - M - 150 * U, H - M - 32 * U, White);
	if (Q->CamMode != AFpvQuad::CamFpv) Txt(Q->CamMode == AFpvQuad::CamChase ? TEXT("CHASE CAM [V]") : TEXT("LINE OF SIGHT [V]"), Cx, M, Dim, 0.9f, true);

	// ---- status / warnings (bottom centre); ARMED shows briefly, warnings blink
	FString Status; FLinearColor SC = White; bool bWarn = false;
	if (S.crashed) { Status = TEXT("CRASH - ARM SWITCH OFF"); SC = Red; bWarn = true; }
	else if (!R.IsConnected() && !Q->bKeyboard) { Status = TEXT("NO RADIO - plug in the handset (USB joystick mode), or K for keyboard"); SC = Warn; bWarn = true; }
	else if (!R.IsCalibrated() && !Q->bKeyboard) { Status = TEXT("RADIO NOT CALIBRATED - press C"); SC = Warn; bWarn = true; }
	else if (S.armed && S.turtle) { Status = TEXT("TURTLE - push the stick away from the side to lift; disarm once it is over"); SC = Warn; }
	else if (!S.armed && Q->In.turtle) { Status = TEXT("TURTLE MODE - arm to flip over"); SC = Warn; bWarn = true; }
	else if (S.armed) { Status = Q->FlightTime < 1.5 ? TEXT("ARMED") : TEXT(""); SC = Green; }
	else if (Q->In.armed && Q->In.throttle > 0.05) { Status = TEXT("THROTTLE NOT LOW - can't arm"); SC = Warn; bWarn = true; }
	else if (!Q->bKeyboard && R.ArmSwitch() && Q->bArmBlocked) { Status = TEXT("ARM SWITCH OFF, then ON"); SC = Warn; bWarn = true; }
	else Status = Q->bKeyboard ? TEXT("DISARMED - Space to arm") : TEXT("DISARMED");
	if (!Status.IsEmpty() && (!bWarn || bBlink)) Txt(Status, Cx, H - M - 110 * U, SC, 1.1f, true);
	if (Cell < 3.3 && S.armed && bBlink) Txt(TEXT("LAND NOW - BATTERY"), Cx, Cy + 120 * U, Red, 1.1f, true);


	// ---- notices and help
	if (Now - Q->NoticeTime < 4.0) Txt(Q->Notice, Cx, Cy - 200 * U, Warn, 1.0f, true);
	if (Now < 20.0) Txt(TEXT("Esc or M settings   C calibrate radio   R reset   T turtle   V camera   H sticks   O clean view   K keyboard"), Cx, H - M - 150 * U, Dim, 0.75f, true);

	// ---- stick overlay (mode 2: left = yaw / throttle, right = roll / pitch)
	if (Q->bShowSticks)
	{
		const float B = 70 * U, Y0 = H - M - B;
		auto Box = [&](float X0, float sx, float sy)
		{
			DrawRect(FLinearColor(0, 0, 0, 0.35f), X0, Y0, B, B);
			DrawRect(White, X0 + (sx * 0.5f + 0.5f) * B - 4 * U, Y0 + (0.5f - sy * 0.5f) * B - 4 * U, 8 * U, 8 * U);
		};
		Box(Cx - B - 10 * U, float(Q->In.yaw), float(Q->In.throttle * 2.0 - 1.0));
		Box(Cx + 10 * U, float(Q->In.roll), float(Q->In.pitch));
	}
}
