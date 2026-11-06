#include "FpvRadio.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#define DIRECTINPUT_VERSION 0x0800
#include <windows.h>
#include <dinput.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace
{
	enum ECalStep { CalNeutral, CalSweep, CalThrottle, CalYaw, CalPitch, CalRoll, CalArm, CalTurtle, CalAssignRef, CalAssign };
	const TCHAR* FuncName[] = { TEXT("throttle"), TEXT("yaw"), TEXT("pitch"), TEXT("roll"), TEXT("arm"), TEXT("turtle"), TEXT("reset"), TEXT("camera") };
	const TCHAR* Backend = TEXT("dinput");   // axis numbering differs from the old winmm reader: older files need a recalibration

#if PLATFORM_WINDOWS
	BOOL CALLBACK EnumJoystick(LPCDIDEVICEINSTANCEW Inst, LPVOID Ctx)
	{
		static_cast<TArray<DIDEVICEINSTANCEW>*>(Ctx)->Add(*Inst);
		return DIENUM_CONTINUE;
	}
#endif
}

FFpvRadio::FFpvRadio()
{
	for (int32 i = 0; i < NumAxes; ++i) { Min[i] = 0.f; Max[i] = 1.f; Center[i] = 0.5f; SeenMin[i] = 1.f; SeenMax[i] = 0.f; }
}

FFpvRadio::~FFpvRadio()
{
	Close();
#if PLATFORM_WINDOWS
	if (DInput) static_cast<IDirectInput8W*>(DInput)->Release();
	DInput = nullptr;
#endif
}

void FFpvRadio::Close()
{
#if PLATFORM_WINDOWS
	if (IDirectInputDevice8W* D = static_cast<IDirectInputDevice8W*>(Device)) { D->Unacquire(); D->Release(); }
#endif
	Device = nullptr; bConnected = false;
}

bool FFpvRadio::Open()
{
#if PLATFORM_WINDOWS
	if (!DInput && FAILED(DirectInput8Create(GetModuleHandleW(nullptr), DIRECTINPUT_VERSION, IID_IDirectInput8W, &DInput, nullptr))) { DInput = nullptr; return false; }
	IDirectInput8W* DI = static_cast<IDirectInput8W*>(DInput);
	TArray<DIDEVICEINSTANCEW> Found;
	DI->EnumDevices(DI8DEVCLASS_GAMECTRL, EnumJoystick, &Found, DIEDFL_ATTACHEDONLY);
	if (Found.Num() == 0) return false;
	// prefer something that looks like a radio over a gamepad that happens to be plugged in too
	int32 Pick = 0;
	for (int32 i = 0; i < Found.Num(); ++i)
	{
		const FString N = Found[i].tszProductName;
		for (const TCHAR* K : { TEXT("EdgeTX"), TEXT("OpenTX"), TEXT("Radiomaster"), TEXT("RadioMaster"), TEXT("ELRS"), TEXT("Jumper"), TEXT("FrSky"), TEXT("TBS"), TEXT("Betafpv"), TEXT("BETAFPV") })
			if (N.Contains(K)) { Pick = i; break; }
	}
	IDirectInputDevice8W* D = nullptr;
	if (FAILED(DI->CreateDevice(Found[Pick].guidInstance, &D, nullptr)) || !D) return false;
	D->SetDataFormat(&c_dfDIJoystick2);
	D->SetCooperativeLevel(GetDesktopWindow(), DISCL_BACKGROUND | DISCL_NONEXCLUSIVE);
	DIPROPRANGE R = {};
	R.diph.dwSize = sizeof(DIPROPRANGE); R.diph.dwHeaderSize = sizeof(DIPROPHEADER); R.diph.dwHow = DIPH_DEVICE; R.diph.dwObj = 0;
	R.lMin = 0; R.lMax = 65535;
	D->SetProperty(DIPROP_RANGE, &R.diph);
	D->Acquire();
	Device = D;
	DeviceName = Found[Pick].tszProductName;
	DIDEVCAPS Caps = {}; Caps.dwSize = sizeof(Caps); D->GetCapabilities(&Caps);
	UE_LOG(LogTemp, Display, TEXT("FPV radio: '%s' (%u axes, %u buttons; %d game controllers found)"), *DeviceName, Caps.dwAxes, Caps.dwButtons, Found.Num());
	return true;
#else
	return false;
#endif
}

bool FFpvRadio::Poll(double Now)
{
#if PLATFORM_WINDOWS
	if (!Device)
	{
		bConnected = false;
		if (Now < NextScan) return false;
		NextScan = Now + 2.0;   // enumerating devices is slow-ish: not every frame
		if (!Open()) return false;
	}
	IDirectInputDevice8W* D = static_cast<IDirectInputDevice8W*>(Device);
	DIJOYSTATE2 St = {};
	D->Poll();
	if (FAILED(D->GetDeviceState(sizeof(St), &St)))
	{
		if (FAILED(D->Acquire()) || FAILED(D->GetDeviceState(sizeof(St), &St))) { Close(); NextScan = Now + 1.0; return false; }
	}
	const LONG Pos[NumAxes] = { St.lX, St.lY, St.lZ, St.lRx, St.lRy, St.lRz, St.rglSlider[0], St.rglSlider[1] };
	for (int32 i = 0; i < NumAxes; ++i)
	{
		Raw[i] = FMath::Clamp(float(Pos[i]) / 65535.f, 0.f, 1.f);
		if (IsCalibrating() && CalStep >= CalSweep && CalStep < CalAssignRef) { SeenMin[i] = FMath::Min(SeenMin[i], Raw[i]); SeenMax[i] = FMath::Max(SeenMax[i], Raw[i]); }
	}
	Buttons = 0;
	for (int32 b = 0; b < 32; ++b) if (St.rgbButtons[b] & 0x80) Buttons |= 1u << b;
	bConnected = true;
	return true;
#else
	return false;
#endif
}

float FFpvRadio::Stick(EFunc F) const
{
	if (!bConnected || !bCalibrated || F >= Arm) return 0.f;
	const FMap& M = Map[F];
	if (M.Axis < 0) return 0.f;
	const int32 a = M.Axis;
	const float v = Raw[a];
	if (F == Throttle)
	{
		float t = (v - Min[a]) / FMath::Max(Max[a] - Min[a], 1e-3f);
		if (M.bInvert) t = 1.f - t;
		return FMath::Clamp(t, 0.f, 1.f);
	}
	float s = v >= Center[a] ? (v - Center[a]) / FMath::Max(Max[a] - Center[a], 1e-3f) : (v - Center[a]) / FMath::Max(Center[a] - Min[a], 1e-3f);
	if (M.bInvert) s = -s;
	s = FMath::Abs(s) < Deadband ? 0.f : (s - FMath::Sign(s) * Deadband) / (1.f - Deadband);
	return FMath::Clamp(s, -1.f, 1.f);
}

bool FFpvRadio::SwitchOn(EFunc F) const
{
	if (!bConnected || !bCalibrated) return false;
	const FMap& M = Map[F];
	if (M.Button >= 0) return ((Buttons >> M.Button) & 1u) != (M.bInvert ? 1u : 0u);
	if (M.Axis < 0) return false;
	return M.bInvert ? Raw[M.Axis] < M.Threshold : Raw[M.Axis] > M.Threshold;
}

FString FFpvRadio::SwitchName(EFunc F) const
{
	if (Map[F].Axis >= 0) return FString::Printf(TEXT("ch %d"), Map[F].Axis + 1);
	if (Map[F].Button >= 0) return FString::Printf(TEXT("button %d"), Map[F].Button + 1);
	return TEXT("-");
}

void FFpvRadio::StartCalibration()
{
	CalStep = CalNeutral; CalMessage.Reset();
	for (int32 i = 0; i < NumAxes; ++i) { SeenMin[i] = 1.f; SeenMax[i] = 0.f; }
	for (int32 f = Throttle; f <= Turtle; ++f) Map[f] = FMap();   // re-found below; reset / camera switches are kept
}

void FFpvRadio::StartAssign(EFunc F)
{
	AssignFunc = F; CalStep = CalAssignRef; CalMessage.Reset();
}

FString FFpvRadio::Prompt() const
{
	switch (CalStep)
	{
	case CalNeutral:   return TEXT("Centre roll, pitch and yaw. Throttle DOWN. All switches OFF.");
	case CalSweep:     return TEXT("Move both sticks around their full range a few times (corners too).");
	case CalThrottle:  return TEXT("Hold THROTTLE fully UP.");
	case CalYaw:       return TEXT("Hold YAW fully RIGHT (throttle can stay anywhere).");
	case CalPitch:     return TEXT("Hold PITCH fully FORWARD (stick up).");
	case CalRoll:      return TEXT("Hold ROLL fully RIGHT.");
	case CalArm:       return TEXT("Flip the ARM switch ON.");
	case CalTurtle:    return TEXT("Flip the TURTLE switch ON  (no turtle switch? just press Enter).");
	case CalAssignRef: return FString::Printf(TEXT("Put the %s switch in its OFF position."), *FString(FuncName[AssignFunc]).ToUpper());
	case CalAssign:    return FString::Printf(TEXT("Flip the %s switch ON  (Enter with nothing changed = no %s switch)."), *FString(FuncName[AssignFunc]).ToUpper(), FuncName[AssignFunc]);
	default:           return FString();
	}
}

/** The axis that moved most from the neutral snapshot becomes F (skipping axes already assigned). */
bool FFpvRadio::DetectAxis(EFunc F)
{
	int32 Best = -1; float BestD = 0.f;
	for (int32 i = 0; i < NumAxes; ++i)
	{
		bool bUsed = false;
		for (int32 f = 0; f < NumFuncs; ++f) if (f != F && Map[f].Axis == i) bUsed = true;
		const float D = FMath::Abs(Raw[i] - Ref[i]);
		if (!bUsed && D > BestD) { BestD = D; Best = i; }
	}
	if (Best < 0 || BestD < 0.25f) { CalMessage = FString::Printf(TEXT("No stick movement seen for %s - hold it at the end and press Enter again."), FuncName[F]); return false; }
	Map[F].Axis = Best; Map[F].Button = -1;
	// throttle is measured from the bottom, so "moved down" means reversed; the others are reversed when the stick reads low at full right/forward
	Map[F].bInvert = Raw[Best] < Ref[Best];
	CalMessage = FString::Printf(TEXT("%s = ch %d%s"), FuncName[F], Best + 1, Map[F].bInvert ? TEXT(" (reversed)") : TEXT(""));
	return true;
}

/** A switch (axis or button) that changed from the reference snapshot, skipping anything assigned to another function. */
bool FFpvRadio::DetectSwitch(EFunc F)
{
	int32 Best = -1; float BestD = 0.f;
	for (int32 i = 0; i < NumAxes; ++i)
	{
		bool bUsed = false;
		for (int32 f = 0; f < NumFuncs; ++f) if (f != F && Map[f].Axis == i) bUsed = true;
		const float D = FMath::Abs(Raw[i] - Ref[i]);
		if (!bUsed && D > BestD) { BestD = D; Best = i; }
	}
	uint32 Changed = Buttons ^ RefButtons;
	for (int32 f = 0; f < NumFuncs; ++f) if (f != F && Map[f].Button >= 0) Changed &= ~(1u << Map[f].Button);
	FMap& M = Map[F];
	if (Best >= 0 && BestD >= 0.25f) { M = FMap(); M.Axis = Best; M.bInvert = Raw[Best] < Ref[Best]; M.Threshold = (Raw[Best] + Ref[Best]) * 0.5f; CalMessage = FString::Printf(TEXT("%s = ch %d"), FuncName[F], Best + 1); return true; }
	if (Changed) { M = FMap(); M.Button = FMath::CountTrailingZeros(Changed); M.bInvert = ((RefButtons >> M.Button) & 1u) != 0; CalMessage = FString::Printf(TEXT("%s = button %d"), FuncName[F], M.Button + 1); return true; }
	return false;
}

void FFpvRadio::Next()
{
	if (!IsCalibrating()) return;
	if (!bConnected) { CalMessage = TEXT("No radio connected."); return; }
	switch (CalStep)
	{
	case CalNeutral:
		for (int32 i = 0; i < NumAxes; ++i) { Ref[i] = CenterSnap[i] = Raw[i]; SeenMin[i] = SeenMax[i] = Raw[i]; }
		RefButtons = Buttons; CalMessage.Reset(); ++CalStep; return;
	case CalSweep: CalMessage.Reset(); ++CalStep; return;
	case CalThrottle: if (DetectAxis(Throttle)) ++CalStep; return;
	case CalYaw: if (DetectAxis(Yaw)) ++CalStep; return;
	case CalPitch: if (DetectAxis(Pitch)) ++CalStep; return;
	case CalRoll: if (DetectAxis(Roll)) ++CalStep; return;
	case CalArm:
		if (!DetectSwitch(Arm)) { CalMessage = TEXT("No switch change seen - flip the arm switch and press Enter again."); return; }
		// the arm switch stays on for the next step: take it as the new reference
		if (Map[Arm].Axis >= 0) Ref[Map[Arm].Axis] = Raw[Map[Arm].Axis];
		RefButtons = Buttons; ++CalStep; return;
	case CalTurtle:
		if (!DetectSwitch(Turtle)) { Map[Turtle] = FMap(); CalMessage = TEXT("No turtle switch (T on the keyboard toggles turtle mode)."); }
		Finish(); return;
	case CalAssignRef:
		for (int32 i = 0; i < NumAxes; ++i) Ref[i] = Raw[i];
		RefButtons = Buttons; CalMessage.Reset(); CalStep = CalAssign; return;
	case CalAssign:
		if (!DetectSwitch(AssignFunc)) { Map[AssignFunc] = FMap(); CalMessage = FString::Printf(TEXT("No %s switch."), FuncName[AssignFunc]); }
		CalStep = -1; Save(); return;
	default: CalStep = -1; return;
	}
}

void FFpvRadio::Finish()
{
	// end points from the sweep; centres from the neutral snapshot
	for (int32 i = 0; i < NumAxes; ++i)
	{
		Min[i] = SeenMax[i] - SeenMin[i] > 0.2f ? SeenMin[i] : 0.f;
		Max[i] = SeenMax[i] - SeenMin[i] > 0.2f ? SeenMax[i] : 1.f;
		Center[i] = FMath::Clamp(CenterSnap[i], Min[i] + 0.05f, Max[i] - 0.05f);
	}
	bCalibrated = true; CalStep = -1;
	CalMessage = CalMessage.IsEmpty() ? FString(TEXT("Calibration saved.")) : CalMessage + TEXT("   Calibration saved.");
	Save();
}

void FFpvRadio::Load(const FString& Path)
{
	ConfigPath = Path;
	FString Text;
	if (!FFileHelper::LoadFileToString(Text, *Path)) return;
	TSharedPtr<FJsonObject> J;
	if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), J) || !J.IsValid()) return;
	FString B;
	if (!J->TryGetStringField(TEXT("backend"), B) || B != Backend) { UE_LOG(LogTemp, Display, TEXT("FPV radio: calibration from the old reader, please recalibrate")); return; }
	const TArray<TSharedPtr<FJsonValue>>* Arr;
	auto ReadArr = [&](const TCHAR* Key, float* Out) { if (J->TryGetArrayField(Key, Arr)) for (int32 i = 0; i < NumAxes && i < Arr->Num(); ++i) Out[i] = (*Arr)[i]->AsNumber(); };
	ReadArr(TEXT("min"), Min); ReadArr(TEXT("max"), Max); ReadArr(TEXT("center"), Center);
	const TSharedPtr<FJsonObject>* Maps;
	if (J->TryGetObjectField(TEXT("map"), Maps))
		for (int32 f = 0; f < NumFuncs; ++f)
		{
			const TSharedPtr<FJsonObject>* M;
			if (!(*Maps)->TryGetObjectField(FuncName[f], M)) continue;
			Map[f].Axis = (*M)->GetIntegerField(TEXT("axis")); Map[f].Button = (*M)->GetIntegerField(TEXT("button"));
			Map[f].bInvert = (*M)->GetBoolField(TEXT("invert")); Map[f].Threshold = (*M)->GetNumberField(TEXT("threshold"));
		}
	J->TryGetNumberField(TEXT("deadband"), Deadband);
	bCalibrated = J->GetBoolField(TEXT("calibrated"));
}

void FFpvRadio::Save() const
{
	TSharedRef<FJsonObject> J = MakeShared<FJsonObject>();
	J->SetStringField(TEXT("backend"), Backend);
	auto WriteArr = [&](const TCHAR* Key, const float* In) { TArray<TSharedPtr<FJsonValue>> A; for (int32 i = 0; i < NumAxes; ++i) A.Add(MakeShared<FJsonValueNumber>(In[i])); J->SetArrayField(Key, A); };
	WriteArr(TEXT("min"), Min); WriteArr(TEXT("max"), Max); WriteArr(TEXT("center"), Center);
	TSharedRef<FJsonObject> Maps = MakeShared<FJsonObject>();
	for (int32 f = 0; f < NumFuncs; ++f)
	{
		TSharedRef<FJsonObject> M = MakeShared<FJsonObject>();
		M->SetNumberField(TEXT("axis"), Map[f].Axis); M->SetNumberField(TEXT("button"), Map[f].Button);
		M->SetBoolField(TEXT("invert"), Map[f].bInvert); M->SetNumberField(TEXT("threshold"), Map[f].Threshold);
		Maps->SetObjectField(FuncName[f], M);
	}
	J->SetObjectField(TEXT("map"), Maps);
	J->SetNumberField(TEXT("deadband"), Deadband);
	J->SetBoolField(TEXT("calibrated"), bCalibrated);
	J->SetStringField(TEXT("device"), DeviceName);
	FString Out;
	FJsonSerializer::Serialize(J, TJsonWriterFactory<>::Create(&Out));
	FFileHelper::SaveStringToFile(Out, *ConfigPath);
}
