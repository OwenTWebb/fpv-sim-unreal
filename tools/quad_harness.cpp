// Headless checks and tuning numbers for Source/FpvSim/QuadSim.h (no Unreal needed).
// Build + run: powershell -File tools/run_harness.ps1
#include "../Source/FpvSim/QuadSim.h"
#include <cstdio>
#include <cstring>
#include <functional>

using namespace fpv;

struct FlatGround : IWorld
{
	bool Trace(const V3& a, const V3& b, V3& hit, V3& n) override
	{
		if (a.z < 0 || b.z >= 0) return false;
		const double t = a.z / (a.z - b.z);
		hit = a + (b - a) * t; n = { 0, 0, 1 }; return true;
	}
	bool AnythingNear(const V3& p, double r) override { return p.z < r; }
};

static const double DT = 1.0 / 2000.0;
static FlatGround G;

static double StickFor(const Rates& r, double degs)   // inverse of the rate curve
{
	double lo = -1, hi = 1;
	for (int i = 0; i < 50; ++i) { const double m = (lo + hi) / 2; (r.Apply(m) < degs ? lo : hi) = m; }
	return (lo + hi) / 2;
}

// level-ish "autopilot" that flies the acro quad like a pilot would: target pitch/roll angles and climb rate
struct Pilot
{
	double pitchDeg = 0, rollDeg = 0, vz = 0, thrFixed = -1, thr = 0.3, thrI = 0.35;
	Inputs Fly(QuadSim& q, double dt)
	{
		Inputs in; in.armed = true;
		const double ep = pitchDeg - q.PitchDeg(), er = rollDeg - q.RollDeg();
		in.pitch = StickFor(q.C.pitch, -ep * 8);   // nose up target -> pull back (negative stick)
		in.roll = StickFor(q.C.roll, er * 8);
		if (thrFixed >= 0) thr = thrFixed;
		else { thrI = std::clamp(thrI + (vz - q.vel.z) * 0.4 * dt, 0.0, 1.0); thr = std::clamp(thrI + (vz - q.vel.z) * 0.06, 0.0, 1.0); }
		in.throttle = thr;
		return in;
	}
};

static QuadSim Hovering(double alt = 20)
{
	QuadSim q; q.Reset({ 0, 0, alt }, 0); q.voltage = 23.6;
	Pilot p; p.thr = 0.3;
	Inputs arm; arm.armed = true; arm.throttle = 0; q.Step(DT, arm, &G);
	for (int i = 0; i < 2000 * 6; ++i) q.Step(DT, p.Fly(q, DT), &G);
	return q;
}

struct Resp { double t50, t90, over, settle; };
static ControllerParams gC;
static double gThr = -1;   // >= 0: run the step at this throttle instead of hover
static Resp StepResponse(const char* name, int axis, double degs, bool print = true)
{
	QuadSim q = Hovering(); q.C = gC;
	Pilot p; p.thr = 0;
	{   // recover the hover throttle
		QuadSim h = Hovering(); Pilot hp; hp.thr = 0.3;
		for (int i = 0; i < 2000 * 4; ++i) h.Step(DT, hp.Fly(h, DT), &G);
		p.thr = hp.thrI;
	}
	Inputs in; in.armed = true; in.throttle = gThr >= 0 ? gThr : p.thr;
	const Rates& R = axis == 0 ? q.C.roll : axis == 1 ? q.C.pitch : q.C.yaw;
	const double s = StickFor(R, degs);
	double peak = 0, t90 = -1, t50 = -1, settle = 0;
	const double target = degs * kDeg;
	for (int i = 0; i < int(0.35 / DT); ++i)
	{
		if (axis == 0) in.roll = s; else if (axis == 1) in.pitch = s; else in.yaw = s;
		q.Step(DT, in, &G);
		double w = axis == 0 ? q.omega.x : axis == 1 ? q.omega.y : -q.omega.z;
		peak = std::max(peak, w);
		const double t = (i + 1) * DT;
		if (t50 < 0 && w >= 0.5 * target) t50 = t;
		if (t90 < 0 && w >= 0.9 * target) t90 = t;
		if (std::fabs(w - target) > 0.05 * target) settle = t;
	}
	const Resp r{ t50 * 1e3, t90 * 1e3, (peak / target - 1) * 100, settle * 1e3 };
	if (print) std::printf("  %-6s %4.0f deg/s: t50 %5.1f ms  t90 %5.1f ms  overshoot %5.1f %%  settle(5%%) %5.1f ms\n", name, degs, r.t50, r.t90, r.over, r.settle);
	return r;
}

static double Score(const Resp& r) { return (r.t90 < 0 ? 1000 : r.t90) + 0.25 * r.settle + 8 * std::max(0.0, r.over - 4); }

/** Grid search of one axis' PID + feedforward on the step response from hover. */
static void Tune(int axis)
{
	Pid* pid = axis == 0 ? &gC.pidRoll : axis == 1 ? &gC.pidPitch : &gC.pidYaw;
	Pid best = *pid; double bs = 1e9;
	for (double p : { 0.025, 0.032, 0.040, 0.050, 0.065, 0.08, 0.1 })
	for (double d : { 0.0, 0.0002, 0.0004, 0.0006, 0.0009 })
	for (double ff : { 0.0, 0.0002, 0.0004, 0.0007, 0.001 })
	for (double i : { 0.1, 0.25, 0.5 })
	{
		if (axis == 2 && d > 0) continue;
		*pid = { p, i, d, ff };
		const Resp a = StepResponse("", axis, 250, false), b = StepResponse("", axis, axis == 2 ? 450 : 600, false);
		const double sc = Score(a) + Score(b);
		if (sc < bs) { bs = sc; best = *pid; }
	}
	*pid = best;
	std::printf("best axis %d: p %.4f i %.3f d %.5f ff %.5f\n", axis, best.p, best.i, best.d, best.ff);
}

int main(int argc, char** argv)
{
	if (argc > 1 && !std::strcmp(argv[1], "tune")) { Tune(0); Tune(1); Tune(2); }
	std::printf("== FPV quad harness (dt %.2f ms)\n", DT * 1e3);
	{   // 1. drop onto the ground, disarmed: must come to rest, upright, no jitter
		QuadSim q; q.Reset({ 0, 0, 0.6 }, 0.3);
		Inputs in;
		double maxW = 0;
		for (int i = 0; i < 2000 * 3; ++i) { q.Step(DT, in, &G); if (i > 2000 * 2) maxW = std::max(maxW, Len(q.omega)); }
		std::printf("rest:     z %.4f m  |v| %.4f m/s  |w| max(last 1 s) %.4f rad/s  roll %.1f pitch %.1f  onGround %d\n", q.pos.z, Len(q.vel), maxW, q.RollDeg(), q.PitchDeg(), q.onGround);
	}
	{   // 2. armed at idle on the ground: must not hop or spin
		QuadSim q; q.Reset({ 0, 0, 0.03 }, 0);
		Inputs in; in.armed = true;
		for (int i = 0; i < 2000 * 3; ++i) q.Step(DT, in, &G);
		std::printf("idle:     armed %d  z %.4f  |w| %.4f  yaw %.1f deg  motors %.3f\n", q.armed, q.pos.z, Len(q.omega), q.HeadingDeg(), q.motor[0]);
	}
	{   // 3. hover throttle, current
		QuadSim q = Hovering(); Pilot p; p.thr = 0.3;
		for (int i = 0; i < 2000 * 4; ++i) q.Step(DT, p.Fly(q, DT), &G);
		std::printf("hover:    z %.2f vz %.3f pitch %.1f roll %.1f\n", q.pos.z, q.vel.z, q.PitchDeg(), q.RollDeg());
		std::printf("hover:    throttle %.3f  mean thrust/motor %.2f N  current %.1f A  voltage %.1f V  T/W %.1f\n", p.thr, (q.thrust[0] + q.thrust[1] + q.thrust[2] + q.thrust[3]) / 4, q.current, q.voltage,
			4 * q.P.maxThrust / (q.P.mass * kG));
	}
	std::printf("rate step responses (from hover):\n");
	StepResponse("roll", 0, 300); StepResponse("roll", 0, 600);
	StepResponse("pitch", 1, 300); StepResponse("pitch", 1, 600);
	StepResponse("yaw", 2, 200); StepResponse("yaw", 2, 450);
	for (double t : { 0.0, 0.12, 0.7, 1.0 })
	{
		gThr = t; std::printf("  at throttle %.2f:\n", t);
		StepResponse("roll", 0, 600); StepResponse("pitch", 1, 600); StepResponse("yaw", 2, 450);
	}
	gThr = -1;
	{   // 4. full-stick roll: peak rate vs the 670 deg/s set point
		QuadSim q = Hovering(); Inputs in; in.armed = true; in.throttle = 0.35; in.roll = 1;
		double peak = 0, ang = 0;
		for (int i = 0; i < int(0.6 / DT); ++i) { q.Step(DT, in, &G); peak = std::max(peak, q.omega.x); ang += q.omega.x * DT; }
		std::printf("flip:     full roll stick 0.6 s: peak %.0f deg/s, rotated %.0f deg, alt lost %.1f m\n", peak / kDeg, ang / kDeg, 20 - q.pos.z);
	}
	{   // 5. speed runs: hold a nose-down angle at full throttle
		for (double pitch : { -45.0, -60.0, -70.0, -78.0 })
		{
			QuadSim q = Hovering(60); Pilot p; p.pitchDeg = pitch; p.thrFixed = 1.0;
			double t = 0; for (; t < 10; t += DT) q.Step(DT, p.Fly(q, DT), &G);
			std::printf("speed:    pitch %3.0f full throttle: %5.1f km/h  climb %5.1f m/s  %5.0f A  %4.1f V\n", pitch, std::sqrt(q.vel.x * q.vel.x + q.vel.y * q.vel.y) * 3.6, q.vel.z, q.current, q.voltage);
		}
	}
	{   // 6. punch out from hover
		QuadSim q = Hovering(); Pilot p; p.thrFixed = 1;
		for (double t = 0; t < 2; t += DT) q.Step(DT, p.Fly(q, DT), &G);
		std::printf("punch:    2 s full throttle: climb %.1f m/s, +%.1f m\n", q.vel.z, q.pos.z - 20);
	}
	{   // 7. armed, throttle cut at 30 m: hits the ground hard -> crash, disarmed
		QuadSim q = Hovering(30); Inputs in; in.armed = true; in.throttle = 0;
		for (double t = 0; t < 4; t += DT) q.Step(DT, in, &G);
		std::printf("crash:    fell from 30 m: crashed %d armed %d impact %.1f m/s  z %.3f\n", q.crashed, q.armed, q.crashImpact, q.pos.z);
	}
	{   // 8. gentle landing at 1.5 m/s: no crash, settles
		QuadSim q = Hovering(3); Pilot p; p.vz = -1.5;
		for (double t = 0; t < 4; t += DT) q.Step(DT, p.Fly(q, DT), &G);
		Inputs in; in.armed = true; for (double t = 0; t < 1; t += DT) q.Step(DT, in, &G);
		std::printf("landing:  crashed %d  z %.3f  roll %.1f pitch %.1f  |w| %.3f\n", q.crashed, q.pos.z, q.RollDeg(), q.PitchDeg(), Len(q.omega));
	}
	{   // 9. battery: 3 minutes of mixed flying at ~45 % throttle
		QuadSim q = Hovering(50); Pilot p; p.thrFixed = 0.45; p.pitchDeg = -35;
		for (double t = 0; t < 180; t += DT) { if (q.pos.z < 20) p.thrFixed = 0.6; if (q.pos.z > 80) p.thrFixed = 0.35; q.Step(DT, p.Fly(q, DT), &G); }
		std::printf("battery:  3 min at ~45 %% thr: %.0f mAh used, %.2f V/cell under load\n", q.mahUsed, q.voltage / q.P.batteryCells);
	}
	for (int axis = 0; axis < 2; ++axis)   // 10. turtle: upside down on the ground, arm in turtle mode, push one stick until it flips
	{
		QuadSim q; q.Reset({ 0, 0, 0.06 }, 0.4);
		q.att = q.att * Quat::FromAxisAngle({ 1, 0, 0 }, kPi);   // upside down
		Inputs in;
		for (double t = 0; t < 1; t += DT) q.Step(DT, in, &G);   // settle on its back
		in.armed = true; in.turtle = true;
		double t = 0, upAt = -1;
		for (; t < 3; t += DT)
		{
			if (axis == 0) in.roll = 1; else if (axis == 1) in.pitch = 1; else in.roll = in.pitch = 0.7;
			q.Step(DT, in, &G);
			if (upAt < 0 && q.Up().z > 0.9) upAt = t;   // stick held the whole time: the fade must still leave it upright
		}
		std::printf("turtle:   %-8s stick: armed %d turtle %d  upright after %.2f s  (up.z %.2f, z %.3f, |w| %.2f, %.2f m from start)\n", axis == 0 ? "roll" : axis == 1 ? "pitch" : "diagonal",
			q.armed, q.turtle, upAt, q.Up().z, q.pos.z, Len(q.omega), std::sqrt(q.pos.x * q.pos.x + q.pos.y * q.pos.y));
	}
	{   // 11. turtle needs the switch on at arming; normal arm with it off
		QuadSim q; q.Reset({ 0, 0, 0.03 }, 0); Inputs in; in.armed = true;
		q.Step(DT, in, &G); in.turtle = true; q.Step(DT, in, &G);
		std::printf("turtle:   switch flipped after arming -> turtle %d (expect 0)\n", q.turtle);
	}
	return 0;
}
