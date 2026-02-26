#pragma once

#include <modules/simulation/simulator_sih/aero.hpp>
#include <matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

/**
 * Lightweight wrapper around the SIH AeroSeg-based aerodynamics.
 * Provides total aerodynamic force/moment for a fixed-wing using wing/tail/fin/fuselage segments.
 * Frames: body FRD, inputs are normalized [-1,1] for roll/pitch/yaw deflections, and thrust [N] along +X.
 */
class FwMpcAero
{
public:
	FwMpcAero() = default;

	void set_damping(float kdv, float kdw)
	{
		_kdv = kdv;
		_kdw = kdw;
	}

	/**
	 * Compute aerodynamic force/moment in body frame.
	 * @param v_B body velocity [m/s] (FRD)
	 * @param w_B body rates [rad/s]
	 * @param altitude_m altitude above MSL [m] (used for air density)
	 * @param u normalized control inputs [roll, pitch, yaw, thrust_N]
	 * @param force_B aerodynamic force [N] in body frame
	 * @param moment_B aerodynamic moment [Nm] in body frame
	 */
	void compute(const matrix::Vector3f &v_B, const matrix::Vector3f &w_B, float altitude_m,
		     const matrix::Vector4f &u, matrix::Vector3f &force_B, matrix::Vector3f &moment_B)
	{
		const float roll_def = math::constrain(u(0), -1.f, 1.f) * FLAP_MAX;
		const float pitch_def = math::constrain(u(1), -1.f, 1.f) * FLAP_MAX;
		const float yaw_def = math::constrain(u(2), -1.f, 1.f) * FLAP_MAX;
		const float thrust = math::max(u(3), 0.f);

		_wing_l.update_aero(v_B, w_B, altitude_m, roll_def);
		_wing_r.update_aero(v_B, w_B, altitude_m, -roll_def);

		_tailplane.update_aero(v_B, w_B, altitude_m, -pitch_def, thrust);
		_fin.update_aero(v_B, w_B, altitude_m, yaw_def, thrust);
		_fuselage.update_aero(v_B, w_B, altitude_m);

		force_B = _wing_l.get_Fa() + _wing_r.get_Fa() + _tailplane.get_Fa() + _fin.get_Fa() + _fuselage.get_Fa()
			  - _kdv * v_B;
		moment_B = _wing_l.get_Ma() + _wing_r.get_Ma() + _tailplane.get_Ma() + _fin.get_Ma() + _fuselage.get_Ma()
			   - _kdw * w_B;
	}

private:
	// Matched to Tools/simulation/gz/models/advanced_plane/model.sdf reference geometry.
	static constexpr float SPAN = 1.48f;       // [m]
	static constexpr float MAC = 0.22f;        // [m]
	static constexpr float WING_AR = 6.5f;     // [-]
	static constexpr float WING_AC_X = -0.12f; // [m] from AdvancedLiftDrag cp
	static constexpr float RP = 0.10f;         // prop radius [m]
	static constexpr float FLAP_MAX = 0.78f;   // [rad] servo joint limits in gz advanced_plane

	// Tail/fin placement from control surface joints in model.sdf (elevator/rudder at x=-0.5 m).
	static constexpr float TAIL_SPAN = 0.52f;  // [m]
	static constexpr float TAIL_MAC = 0.12f;   // [m]
	static constexpr float TAIL_AR = 4.3f;     // [-]
	static constexpr float TAIL_CF = 0.06f;    // [m]
	static constexpr float TAIL_X = -0.50f;    // [m]
	static constexpr float FIN_SPAN = 0.30f;   // [m]
	static constexpr float FIN_MAC = 0.18f;    // [m]
	static constexpr float FIN_AR = 1.7f;      // [-]
	static constexpr float FIN_CF = 0.12f;     // [m]
	static constexpr float FIN_X = -0.50f;     // [m]
	static constexpr float FIN_Z_FRD = -0.05f; // [m] rudder z=+0.05 in gz (up), converted to FRD

	float _kdv{1.0f};         // linear drag (N/(m/s))
	float _kdw{0.025f};       // angular damper (Nm/(rad/s))

	AeroSeg _wing_l{SPAN / 2.0f, MAC, -1.74f, matrix::Vector3f(WING_AC_X, -SPAN / 4.0f, 0.0f), 3.0f,
			WING_AR, MAC / 3.0f};
	AeroSeg _wing_r{SPAN / 2.0f, MAC, -1.74f, matrix::Vector3f(WING_AC_X, SPAN / 4.0f, 0.0f), -3.0f,
			WING_AR, MAC / 3.0f};
	AeroSeg _tailplane{TAIL_SPAN, TAIL_MAC, 0.0f, matrix::Vector3f(TAIL_X, 0.0f, 0.0f), 0.0f, TAIL_AR, TAIL_CF, RP};
	AeroSeg _fin{FIN_SPAN, FIN_MAC, 0.0f, matrix::Vector3f(FIN_X, 0.0f, FIN_Z_FRD), -90.0f, FIN_AR, FIN_CF, RP};
	AeroSeg _fuselage{0.47f, 0.11f, 0.0f, matrix::Vector3f(0.0f, 0.0f, 0.0f), -90.0f};
};
