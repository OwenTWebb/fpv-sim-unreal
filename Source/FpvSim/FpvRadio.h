// The radio: an ELRS / EdgeTX handset plugged in over USB in "USB Joystick" mode, read with DirectInput (no drivers):
// all 8 axes (channels 1-8) and up to 32 buttons, so every switch the radio sends can be used. A guided calibration
// finds which axis is which and the stick end points; switches for arm / turtle / reset / camera are assigned by flipping
// them. The result is saved to fpv_radio.json next to the .uproject.
#pragma once

#include "CoreMinimal.h"

class FFpvRadio
{
public:
	enum EFunc { Throttle, Yaw, Pitch, Roll, Arm, Turtle, Reset, Camera, NumFuncs };
	static constexpr int32 NumAxes = 8;

	FFpvRadio();
	~FFpvRadio();
	/** Reads the handset (call once per frame). Returns false when no joystick is connected. */
	bool Poll(double Now);
	bool IsConnected() const { return bConnected; }
	bool IsCalibrated() const { return bCalibrated; }

	/** Roll / pitch / yaw -1..1 (right, forward, right), throttle 0..1. Zero when not connected or not calibrated. */
	float Stick(EFunc F) const;
	/** A switch function (Arm, Turtle, Reset, Camera): true when that switch is on. False when it is not assigned. */
	bool SwitchOn(EFunc F) const;
	bool ArmSwitch() const { return SwitchOn(Arm); }
	bool HasSwitch(EFunc F) const { return Map[F].Axis >= 0 || Map[F].Button >= 0; }
	FString SwitchName(EFunc F) const;   // "ch 6", "button 3" or "-"

	// guided calibration: Start, then Next() on each Enter press
	void StartCalibration();
	/** Assigns one switch function (keeps the rest of the calibration). */
	void StartAssign(EFunc F);
	void Next();
	void Cancel() { CalStep = -1; CalMessage.Reset(); }
	bool IsCalibrating() const { return CalStep >= 0; }
	FString Prompt() const;
	FString CalMessage;     // feedback for the last step ("roll: axis 1", "no movement seen", ...)

	void Load(const FString& Path);
	void Save() const;

	FString DeviceName;
	float Raw[NumAxes] = {};      // 0..1, uncalibrated
	uint32 Buttons = 0;
	float Deadband = 0.01f;

private:
	struct FMap { int32 Axis = -1; bool bInvert = false; int32 Button = -1; float Threshold = 0.5f; };
	FMap Map[NumFuncs];
	float Min[NumAxes], Max[NumAxes], Center[NumAxes];
	float Ref[NumAxes] = {}, CenterSnap[NumAxes] = {}, SeenMin[NumAxes], SeenMax[NumAxes];
	uint32 RefButtons = 0;
	int32 CalStep = -1;
	EFunc AssignFunc = Turtle;
	bool bConnected = false, bCalibrated = false;
	double NextScan = 0;
	FString ConfigPath;
	void* DInput = nullptr;   // IDirectInput8W*
	void* Device = nullptr;   // IDirectInputDevice8W*

	bool Open();
	void Close();
	bool DetectAxis(EFunc F);
	bool DetectSwitch(EFunc F);
	void Finish();
};
