// FPV quad physics + Betaflight-style acro rate controller. Plain C++ (no Unreal types), so the same code runs in the
// game and in the headless tuning harness (tools/quad_harness.cpp).
//
// Frames (right-handed, SI units): world X forward/north, Y left/west, Z up. Body FLU: x forward, y left, z up.
// Body rates: p > 0 rolls right, q > 0 pitches nose down, r > 0 yaws left.
// Motor order follows Betaflight: 0 rear-right, 1 front-right, 2 rear-left, 3 front-left (props-in: 0 and 3 spin CW from above).
#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace fpv
{
struct V3
{
	double x = 0, y = 0, z = 0;
	V3() = default;
	V3(double a, double b, double c) : x(a), y(b), z(c) {}
	V3 operator+(const V3& o) const { return { x + o.x, y + o.y, z + o.z }; }
	V3 operator-(const V3& o) const { return { x - o.x, y - o.y, z - o.z }; }
	V3 operator-() const { return { -x, -y, -z }; }
	V3 operator*(double s) const { return { x * s, y * s, z * s }; }
	V3& operator+=(const V3& o) { x += o.x; y += o.y; z += o.z; return *this; }
	V3& operator-=(const V3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
};
inline double Dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 Cross(const V3& a, const V3& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
inline double Len(const V3& a) { return std::sqrt(Dot(a, a)); }
inline V3 Mul(const V3& a, const V3& b) { return { a.x * b.x, a.y * b.y, a.z * b.z }; }

struct Quat   // body -> world
{
	double w = 1, x = 0, y = 0, z = 0;
	Quat operator*(const Quat& o) const
	{
		return { w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
		         w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w };
	}
	V3 Rotate(const V3& v) const   // body -> world
	{
		const V3 u(x, y, z); const V3 t = Cross(u, v) * 2.0;
		return v + t * w + Cross(u, t);
	}
	V3 Unrotate(const V3& v) const { return Quat{ w, -x, -y, -z }.Rotate(v); }   // world -> body
	void Normalize() { const double n = std::sqrt(w * w + x * x + y * y + z * z); w /= n; x /= n; y /= n; z /= n; }
	static Quat FromAxisAngle(const V3& axis, double ang)
	{
		const double l = Len(axis); if (l < 1e-12) return {};
		const double s = std::sin(ang * 0.5) / l; return { std::cos(ang * 0.5), axis.x * s, axis.y * s, axis.z * s };
	}
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kG = 9.80665;

/** Betaflight "Actual" rates: stick -1..1 -> deg/s. */
struct Rates
{
	double center = 200, max = 670, expo = 0.54;
	double Apply(double s) const
	{
		s = std::clamp(s, -1.0, 1.0); const double a = std::fabs(s);
		const double expof = a * (std::pow(s, 5) * expo + s * (1.0 - expo));
		return s * center + std::max(0.0, max - center) * expof;
	}
};

struct Pid { double p, i, d, ff; };

struct QuadParams
{
	// airframe: 5" freestyle quad on 6S, with battery and action camera ~0.65 kg
	double mass = 0.65;                      // kg
	V3 inertia{ 0.0018, 0.0020, 0.0034 };    // kg m^2 (body x, y, z)
	double armX = 0.080, armY = 0.080;       // motor position from the CG, m (226 mm wheelbase)
	double maxThrust = 13.5;                 // N per motor, static, full throttle (2207 1900 kV, 5x4.3x3)
	double maxRpm = 32000;
	double propPitchSpeed = 62;              // m/s: axial inflow at which a full-rpm prop stops pushing (pitch x rpm)
	double torqueCoef = 0.012;               // yaw drag torque per newton of thrust, m
	double rotorInertia = 6e-6;              // kg m^2, prop + bell (gives the yaw kick on throttle changes)
	double motorTauUp = 0.022, motorTauDown = 0.030;   // s, rpm time constants
	double idle = 0.055;                     // armed idle, fraction of full rpm
	V3 dragCdA{ 0.0090, 0.0115, 0.032 };     // m^2 front / side / top
	double rotorDrag = 0.07;                 // N per m/s of in-plane airspeed at full rpm (blade flapping / H-force)
	double angularDamping = 4e-4;            // N m s
	double airDensity = 1.225;               // kg/m^3
	double propwash = 1.0;                   // 0 = off; scales the shake when descending into your own wash
	// battery: open-circuit 6S voltage sags with load and drops with use (thrust ~ voltage^2)
	double batteryCells = 6, batteryMah = 1300, batteryResistance = 0.018;   // ohm per pack
	bool infiniteBattery = false;            // pack never drains or sags: always 4.2 V/cell
	// turtle mode (Betaflight "flip over after crash"): motors run backwards, chosen by stick direction
	double turtlePower = 0.6;                // stick-to-motor scale, 0..1 (like Betaflight's crash-flip power limit)
	double turtleEfficiency = 0.55;          // a prop spun backwards gives about half its thrust
	// contacts: the hull is a box; its 8 corners are spring-damper contact points (plus a swept centre point)
	V3 hullMin{ -0.115, -0.115, -0.028 }, hullMax{ 0.115, 0.115, 0.04 };
	double contactK = 20000, contactC = 60, friction = 0.7, restitution = 0.15;
	double crashSpeed = 11;                  // m/s normal impact speed that disarms (<= 0: never)
};

struct ControllerParams
{
	Rates roll{ 200, 670, 0.54 }, pitch{ 200, 670, 0.54 }, yaw{ 200, 500, 0.40 };
	// u = thrust fraction per motor (differential); see Controller::Update
	Pid pidRoll{ 0.060, 0.15, 0.00070, 0.0003 }, pidPitch{ 0.065, 0.15, 0.00075, 0.0003 }, pidYaw{ 0.080, 0.40, 0.0, 0.0005 };   // tuned with tools/quad_harness.cpp
	double iLimit = 0.15;
	double gyroLowpassHz = 120, dLowpassHz = 90, ffLowpassHz = 30;   // gyro filtering is what limits the usable gain, like on the real FC
	bool airmode = true;
	double throttleLimit = 1.0;              // scale on the throttle stick
};

struct Inputs
{
	double roll = 0, pitch = 0, yaw = 0;     // -1..1 (right / forward / right)
	double throttle = 0;                     // 0..1
	bool armed = false;
	bool turtle = false;                     // turtle switch: arming with it on arms in turtle mode
};

/** Collision queries from the host (Unreal line traces, or a flat ground in the harness). */
struct IWorld
{
	virtual ~IWorld() = default;
	/** First blocking hit on the segment from -> to. Normal points out of the surface. */
	virtual bool Trace(const V3& from, const V3& to, V3& hit, V3& normal) = 0;
	/** Cheap broad phase: anything within radius r of p? Return true if unsure. */
	virtual bool AnythingNear(const V3& p, double r) { (void)p; (void)r; return true; }
};

class QuadSim
{
public:
	QuadParams P;
	ControllerParams C;

	// state
	V3 pos, vel;           // world, m, m/s
	Quat att;              // body -> world
	V3 omega;              // body rates, rad/s
	double motor[4] = {};  // rpm fraction 0..1
	double motorCmd[4] = {};
	double thrust[4] = {};
	bool armed = false, crashed = false, onGround = false;
	bool turtle = false;   // armed in turtle mode (latched at arming, like Betaflight)
	double lastImpact = 0, crashImpact = 0;   // m/s: last normal impact speed, and the one that caused the crash
	double mahUsed = 0, voltage = 0, current = 0;
	double time = 0;

	QuadSim() { Reset({}, 0); }

	void Reset(const V3& p, double yawRad)
	{
		pos = p; vel = {}; omega = {}; att = Quat::FromAxisAngle({ 0, 0, 1 }, yawRad);
		for (int i = 0; i < 4; ++i) motor[i] = motorCmd[i] = thrust[i] = 0;
		armed = crashed = onGround = turtle = false; lastImpact = crashImpact = 0; mahUsed = 0; current = 0; voltage = P.batteryCells * 4.2; time = 0;
		iTerm = {}; gyro = {}; prevGyro = {}; dFilt = {}; ffFilt = {}; prevSet = {}; airmodeActive = false; noise = {}; noiseState = 12345;
	}

	static V3 MotorPos(int i, const QuadParams& P)
	{
		static const double sx[4] = { -1, 1, -1, 1 }, sy[4] = { -1, -1, 1, 1 };   // RR, FR, RL, FL
		return { sx[i] * P.armX, sy[i] * P.armY, 0.0 };
	}
	static double SpinDir(int i) { static const double s[4] = { -1, 1, 1, -1 }; return s[i]; }   // +1 = CCW from above

	/** Advances by dt (call with a small fixed step, e.g. 1/2000 s). */
	void Step(double dt, const Inputs& in, IWorld* world)
	{
		time += dt;
		// ---- arming: needs throttle low; a crash stays disarmed until the arm switch is cycled
		if (!in.armed) { armed = false; crashed = false; turtle = false; }
		else if (!armed && !crashed && in.throttle < 0.05) { armed = true; turtle = in.turtle; }

		Controller(dt, in);
		Motors(dt);

		// ---- forces and torques in the body frame
		const V3 vBody = att.Unrotate(vel);
		V3 F, T;
		const double vSag = voltage / (P.batteryCells * 3.85);   // rpm scales with voltage; maxThrust is at 3.85 V/cell under load
		double sumW = 0;
		for (int i = 0; i < 4; ++i)
		{
			const V3 r = MotorPos(i, P);
			const double vAx = Dot(vBody + Cross(omega, r), { 0, 0, 1 });   // inflow through the disc (climb, and roll/pitch rate)
			const double w = motor[i] * vSag;
			double t = P.maxThrust * w * (w - vAx / P.propPitchSpeed);
			t = std::clamp(t, 0.0, 1.35 * P.maxThrust * w * w);
			const double dir = turtle ? -1.0 : 1.0;   // turtle: reversed rotation, thrust pushes down out of the top of the props
			if (turtle) t = P.turtleEfficiency * P.maxThrust * w * w;
			thrust[i] = t; sumW += w;
			F += V3(0, 0, dir * t);
			T += Cross(r, V3(0, 0, dir * t));
			const double s = SpinDir(i) * dir;
			T += V3(0, 0, -s * P.torqueCoef * t - s * P.rotorInertia * motorAccel[i] * P.maxRpm * (2 * kPi / 60));
		}
		// body drag (quadratic, per axis) and rotor H-force (linear in in-plane speed, grows with rpm)
		const double sp = Len(vBody);
		F -= Mul(P.dragCdA, vBody) * (0.5 * P.airDensity * sp);
		F -= V3(vBody.x, vBody.y, 0) * (P.rotorDrag * sumW / 4.0);
		T -= omega * P.angularDamping;
		// propwash: descending through your own wake shakes the quad
		if (P.propwash > 0)
		{
			const double sink = std::clamp((-vBody.z - 1.5) / 6.0, 0.0, 1.0) * std::clamp(1.0 - (std::fabs(vBody.x) + std::fabs(vBody.y)) / 12.0, 0.0, 1.0);
			const double k = std::exp(-dt * 2 * kPi * 12.0);
			noise = noise * k + V3(Rand(), Rand(), Rand() * 0.4) * (1 - k);
			T += noise * (0.9 * P.propwash * sink * (sumW / 4.0));
		}

		// ---- to world, contacts, integrate
		V3 Fw = att.Rotate(F) + V3(0, 0, -kG * P.mass);
		V3 Tw = att.Rotate(T);
		onGround = false;
		if (world) Contacts(world, Fw, Tw);

		vel += Fw * (dt / P.mass);
		const V3 prev = pos;
		pos += vel * dt;
		V3 Tb = att.Unrotate(Tw);
		const V3 Iw = Mul(P.inertia, omega);
		Tb -= Cross(omega, Iw);
		omega += V3(Tb.x / P.inertia.x, Tb.y / P.inertia.y, Tb.z / P.inertia.z) * dt;
		att = att * Quat::FromAxisAngle(omega, Len(omega) * dt);
		att.Normalize();

		if (world && world->AnythingNear(pos, Len(pos - prev) + 0.3)) SweepCentre(world, prev);
		Battery(dt);
	}

	// attitude helpers (for the OSD)
	V3 Forward() const { return att.Rotate({ 1, 0, 0 }); }
	V3 Up() const { return att.Rotate({ 0, 0, 1 }); }
	double PitchDeg() const { const V3 f = Forward(); return std::atan2(f.z, std::sqrt(f.x * f.x + f.y * f.y)) / kDeg; }   // nose up +
	double RollDeg() const { const V3 l = att.Rotate({ 0, 1, 0 }), u = Up(); return std::atan2(l.z, u.z) / kDeg; }       // right wing down +
	double HeadingDeg() const { const V3 f = Forward(); double h = std::atan2(-f.y, f.x) / kDeg; return h < 0 ? h + 360 : h; }

	V3 rateSetpoint;       // rad/s, last controller setpoint (for the harness / OSD)

private:
	V3 iTerm, gyro, prevGyro, dFilt, ffFilt, prevSet, noise;
	bool airmodeActive = false;
	uint32_t noiseState = 12345;
	double motorAccel[4] = {};

	double Rand() { noiseState = noiseState * 1664525u + 1013904223u; return (noiseState >> 8) / double(1 << 24) * 2.0 - 1.0; }

	void Controller(double dt, const Inputs& in)
	{
		if (!armed) { for (double& m : motorCmd) m = 0; iTerm = {}; airmodeActive = false; return; }
		if (turtle)
		{
			// each motor runs when the stick points away from it: running a side's motors backwards pushes that side
			// down, which rolls / pitches the quad over the opposite edge. Yaw spins one diagonal pair. No PID, no idle.
			iTerm = {}; rateSetpoint = {};
			const double rr = in.roll, pp = in.pitch, yy = in.yaw;
			const double mag = std::max({ std::fabs(rr), std::fabs(pp), std::fabs(yy) });
			// fade the motors out as it rolls past vertical, so it falls the rest of the way onto its feet instead of tumbling over
			const double fade = std::clamp((-0.15 - Up().z) / 0.5, 0.0, 1.0);
			for (int i = 0; i < 4; ++i)
			{
				const V3 r = MotorPos(i, P);
				const double mr = r.y > 0 ? 1 : -1, mp = r.x < 0 ? 1 : -1, my = -SpinDir(i);
				const double wgt = std::clamp(-(mr * rr + mp * pp) + (std::fabs(rr) + std::fabs(pp) < 0.2 ? my * yy : 0.0), 0.0, 1.0);
				motorCmd[i] = mag < 0.15 ? 0.0 : std::clamp(wgt * P.turtlePower * fade, 0.0, 1.0);
			}
			return;
		}
		const double thr = std::clamp(in.throttle, 0.0, 1.0) * C.throttleLimit;
		if (thr > 0.25) airmodeActive = true;
		const V3 set(C.roll.Apply(in.roll) * kDeg, C.pitch.Apply(in.pitch) * kDeg, -C.yaw.Apply(in.yaw) * kDeg);
		rateSetpoint = set;
		gyro += (omega - gyro) * (1.0 - std::exp(-dt * 2 * kPi * C.gyroLowpassHz));
		const V3 err = set - gyro;
		const double aD = 1.0 - std::exp(-dt * 2 * kPi * C.dLowpassHz), aF = 1.0 - std::exp(-dt * 2 * kPi * C.ffLowpassHz);
		dFilt += ((gyro - prevGyro) * (1.0 / dt) - dFilt) * aD; prevGyro = gyro;
		ffFilt += ((set - prevSet) * (1.0 / dt) - ffFilt) * aF; prevSet = set;
		// no integrator wind-up while sitting on the ground at low throttle (Betaflight resets I at zero throttle without airmode)
		const bool iActive = airmodeActive || thr > 0.05;
		const Pid* pid[3] = { &C.pidRoll, &C.pidPitch, &C.pidYaw };
		double u[3];
		const double e[3] = { err.x, err.y, err.z }, d[3] = { dFilt.x, dFilt.y, dFilt.z }, ff[3] = { ffFilt.x, ffFilt.y, ffFilt.z };
		double* it[3] = { &iTerm.x, &iTerm.y, &iTerm.z };
		for (int a = 0; a < 3; ++a)
		{
			if (iActive) *it[a] = std::clamp(*it[a] + pid[a]->i * e[a] * dt, -C.iLimit, C.iLimit); else *it[a] = 0;
			u[a] = pid[a]->p * e[a] + *it[a] - pid[a]->d * d[a] + pid[a]->ff * ff[a];
		}
		// u is a thrust fraction per motor; the mixer works in rpm-command space, so linearise around the current
		// throttle (thrust ~ rpm^2: d rpm = d thrust / (2 rpm)). This keeps the loop gain flat across throttle.
		const double wRef = std::max(0.30, P.idle + (1 - P.idle) * thr);
		double mix[4], lo = 1e9, hi = -1e9;
		for (int i = 0; i < 4; ++i)
		{
			const V3 r = MotorPos(i, P);
			const double tr = (r.y > 0 ? 1 : -1) * u[0] + (r.x < 0 ? 1 : -1) * u[1] - SpinDir(i) * u[2];
			mix[i] = tr / (2 * wRef * (1 - P.idle));
			lo = std::min(lo, mix[i]); hi = std::max(hi, mix[i]);
		}
		// airmode: shrink the mix if it needs more than the full range, then shift throttle to fit it in
		const double range = hi - lo;
		if (range > 1.0) { for (double& m : mix) m /= range; lo /= range; hi /= range; }
		double base = thr;
		if (C.airmode && airmodeActive) base = std::clamp(base, -lo, 1.0 - hi);
		for (int i = 0; i < 4; ++i)
			motorCmd[i] = P.idle + (1 - P.idle) * std::clamp(base + mix[i], 0.0, 1.0);
	}

	void Motors(double dt)
	{
		for (int i = 0; i < 4; ++i)
		{
			const double tau = motorCmd[i] > motor[i] ? P.motorTauUp : P.motorTauDown;
			const double prev = motor[i];
			motor[i] += (motorCmd[i] - motor[i]) * (1.0 - std::exp(-dt / tau));
			motorAccel[i] = (motor[i] - prev) / dt;
		}
	}

	void Battery(double dt)
	{
		// electrical power from ideal induced power P = T^1.5 / sqrt(2 rho A), at ~40 % overall (prop figure of merit x motor/ESC)
		const double A = kPi * 0.0635 * 0.0635;   // one 5" disc
		double pw = 0; for (double t : thrust) pw += std::pow(t, 1.5) / std::sqrt(2 * P.airDensity * A);
		pw = pw / 0.40 + (armed ? 4.0 : 0.5);
		const double used = std::clamp(mahUsed / P.batteryMah, 0.0, 1.0);
		const double ocv = P.batteryCells * (4.2 - 0.55 * used - 0.35 * std::pow(used, 6));
		current = pw / std::max(ocv, 1.0);
		if (P.infiniteBattery) { voltage = P.batteryCells * 4.2; return; }
		voltage = ocv - current * P.batteryResistance;
		mahUsed += current * dt / 3.6;
	}

	void Contacts(IWorld* world, V3& Fw, V3& Tw)
	{
		const double rMax = Len(P.hullMax - P.hullMin) * 0.5 + 0.05;
		if (!world->AnythingNear(pos, rMax)) return;
		for (int k = 0; k < 8; ++k)
		{
			const V3 b((k & 1) ? P.hullMax.x : P.hullMin.x, (k & 2) ? P.hullMax.y : P.hullMin.y, (k & 4) ? P.hullMax.z : P.hullMin.z);
			const V3 r = att.Rotate(b), pt = pos + r;
			V3 hit, n;
			if (!world->Trace(pos, pt, hit, n)) continue;
			const double depth = Dot(hit - pt, n);
			if (depth <= 0) continue;
			const V3 vp = vel + att.Rotate(Cross(omega, b));
			const double vn = Dot(vp, n);
			if (vn < -P.crashSpeed && P.crashSpeed > 0 && armed) { crashed = true; armed = false; crashImpact = -vn; }
			if (vn < -0.5) lastImpact = -vn;
			const double fn = std::max(0.0, P.contactK * depth - P.contactC * vn);
			const V3 vt = vp - n * vn; const double vtl = Len(vt);
			V3 f = n * fn;
			if (vtl > 1e-6) f -= vt * (std::min(P.friction * fn, 40.0 * vtl) / vtl);
			Fw += f; Tw += Cross(r, f);
			onGround = true;
		}
	}

	void SweepCentre(IWorld* world, const V3& prev)
	{
		V3 hit, n;
		if (!world->Trace(prev, pos, hit, n)) return;
		const double vn = Dot(vel, n);
		if (vn < 0)
		{
			if (P.crashSpeed > 0 && -vn > P.crashSpeed && armed) { crashed = true; armed = false; crashImpact = -vn; }
			lastImpact = -vn;
			const V3 vt = vel - n * vn;
			vel = vt * 0.6 - n * (vn * P.restitution);
			omega = omega * 0.5;
		}
		pos = hit + n * 0.01;
	}
};
}  // namespace fpv
