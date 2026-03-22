#include "FwMpcController.hpp"

#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

#include <algorithm>
#include <cmath>
#include <vector>

using matrix::Matrix;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::Vector4f;

namespace
{
template<typename Mat>
void denseToCSC(const Mat &M, int rows, int cols, bool upper_only,
		std::vector<OSQPFloat> &data, std::vector<OSQPInt> &indices, std::vector<OSQPInt> &indptr)
{
	data.clear();
	indices.clear();
	indptr.clear();
	indptr.reserve(cols + 1);

	const float eps = 1e-6f;

	for (int j = 0; j < cols; j++) {
		indptr.push_back((OSQPInt)data.size());

		for (int i = 0; i < rows; i++) {
			if (upper_only && i > j) {
				continue;
			}

			const float v = M(i, j);

			if (fabsf(v) > eps) {
				data.push_back(v);
				indices.push_back((OSQPInt)i);
			}
		}
	}

	indptr.push_back((OSQPInt)data.size());
}

matrix::Dcmf rotation_matrix(float phi, float theta, float psi)
{
	const float cP = cosf(phi);
	const float sP = sinf(phi);
	const float cT = cosf(theta);
	const float sT = sinf(theta);
	const float cS = cosf(psi);
	const float sS = sinf(psi);

	matrix::Dcmf R;
	R(0, 0) = cT * cS;
	R(0, 1) = sP * sT * cS - cP * sS;
	R(0, 2) = cP * sT * cS + sP * sS;
	R(1, 0) = cT * sS;
	R(1, 1) = sP * sT * sS + cP * cS;
	R(1, 2) = cP * sT * sS - sP * cS;
	R(2, 0) = -sT;
	R(2, 1) = sP * cT;
	R(2, 2) = cP * cT;
	return R;
}

bool obstacle_is_ahead(const Vector2f &rel_xy, const Vector2f &vel_xy, float clearance)
{
	const float speed_xy = vel_xy.norm();

	if (speed_xy <= 3.f) {
		return true;
	}

	return rel_xy.dot(vel_xy / speed_xy) > -math::max(clearance, 0.f);
}
} // namespace

bool FwMpcController::configure(float Ts, int horizon)
{
	if (horizon < 1 || horizon > kMaxHorizon) {
		PX4_ERR("MPC horizon %d exceeds limit %d", horizon, kMaxHorizon);
		return false;
	}

	_Ts = Ts;
	_N = horizon;
	_ubar.setZero();
	_xbar.setZero();
	_fallback_ubar.setZero();
	_fallback_xapply.setZero();
	_last_solved_xbar.setZero();
	_time_since_last_solve_s = 0.f;
	_last_qp_status = 0;
	_warm_start_z.setZero();
	_warm_start_n_vars = 0;
	_have_warm_start = false;
	_have_fallback_trajectory = false;
	return true;
}

void FwMpcController::set_obstacles(const std::vector<Obstacle> &obs)
{
	_n_obstacles = std::min((int)obs.size(), kMaxObstacles);

	for (int i = 0; i < _n_obstacles; i++) {
		_obstacles[i] = obs[i];
	}
}

void FwMpcController::clear_obstacles()
{
	_n_obstacles = 0;
}

FwMpcController::StateVec FwMpcController::initTrim(float V_trim, float z0_up, const matrix::Vector3f &goal_up)
{
	_V_trim = V_trim;
	const float v0 = 0.f;

	const Vector3f dp = goal_up - Vector3f{0.f, 0.f, z0_up};
	const float psi0 = atan2f(dp(1), dp(0));
	float alpha_trim = _alpha_trim;
	float theta_trim = _theta_trim;
	float thrust_trim = _T_trim;
	const float alpha_seed = PX4_ISFINITE(_alpha_trim) ? _alpha_trim : 0.08f;
	const float thrust_seed = PX4_ISFINITE(_T_trim) ? _T_trim : 0.5f;

	if (!solve_model_steady_reference(V_trim, z0_up, 0.f, psi0, 0.f,
					  alpha_seed,
					  thrust_seed,
					  alpha_trim, theta_trim, thrust_trim)) {
		const float mass = _model.mass();
		const float qdyn = 0.5f * _model.rho() * V_trim * V_trim * _model.wing_area();
		const float CLreq = mass * _model.gravity() / math::max(qdyn, 1e-3f);
		alpha_trim = (CLreq - _model.CL0()) / _model.CL_alpha();
		const float CD = _model.CD0() + _model.induced_drag_k() * CLreq * CLreq;
		thrust_trim = 0.5f * _model.rho() * V_trim * V_trim * _model.wing_area() * CD;
		theta_trim = alpha_trim;
	}

	_alpha_trim = alpha_trim;
	_theta_trim = theta_trim;
	_T_trim = thrust_trim;

	const float u_trimmed = V_trim * cosf(_alpha_trim);
	const float w_trimmed = V_trim * sinf(_alpha_trim);

	StateVec x0{};
	x0(0) = u_trimmed;
	x0(1) = v0;
	x0(2) = w_trimmed;
	x0(3) = 0.f;
	x0(4) = 0.f;
	x0(5) = 0.f;
	x0(6) = 0.f;
	x0(7) = _theta_trim;
	x0(8) = psi0;
	x0(9) = 0.f;
	x0(10) = 0.f;
	x0(11) = z0_up;

	const ControlVec u_trim{0.f, 0.f, 0.f, _T_trim};

	for (int k = 0; k < _N; k++) {
		_ubar.col(k) = u_trim;
	}

	_xbar.col(0) = x0;

	for (int k = 0; k < _N; k++) {
		const StateVec xk = _xbar.col(k);
		const ControlVec uk = _ubar.col(k);
		_xbar.col(k + 1) = fd_step(xk, uk);
	}

	_have_fallback_trajectory = false;
	_fallback_ubar = _ubar;
	_fallback_xapply.setZero();
	_last_solved_xbar = _xbar;
	_time_since_last_solve_s = 0.f;

	return x0;
}

void FwMpcController::set_vehicle_params(float mass, const matrix::Vector3f &inertia_diag, float kdv, float kdw)
{
	_model.set_mass(mass);
	_model.set_inertia_diag(inertia_diag);
	_model.set_damping(kdv, kdw);
}

bool FwMpcController::solve_model_steady_reference(float V_target, float z_up, float phi_ref, float psi_ref,
		float gamma_ref, float alpha_seed, float thrust_seed,
		float &alpha_ref, float &theta_ref, float &thrust_ref) const
{
	const float V = math::max(V_target, 3.f);
	float alpha = math::constrain(alpha_seed, -0.35f, 0.35f);
	float thrust = math::constrain(thrust_seed, _limits.u_min(3), _limits.u_max(3));
	const float hdot_des = V * sinf(gamma_ref);
	const float alpha_eps = 1e-3f;
	const float thrust_eps = 0.25f;
	bool converged = false;

	auto evaluate_reference = [&](float alpha_eval, float thrust_eval, float &vdot, float &hdot) {
		StateVec x{};
		x(0) = V * cosf(alpha_eval);
		x(1) = 0.f;
		x(2) = V * sinf(alpha_eval);
		x(3) = 0.f;
		x(4) = 0.f;
		x(5) = 0.f;
		x(6) = phi_ref;
		x(7) = alpha_eval + gamma_ref;
		x(8) = psi_ref;
		x(9) = 0.f;
		x(10) = 0.f;
		x(11) = z_up;

		ControlVec u{};
		u(0) = 0.f;
		u(1) = 0.f;
		u(2) = 0.f;
		u(3) = thrust_eval;

		const StateVec dx = _model.dynamics(x, u);
		const Vector3f vel_body{x(0), x(1), x(2)};
		const Vector3f acc_body{dx(0), dx(1), dx(2)};
		vdot = vel_body.dot(acc_body) / V;
		hdot = dx(11);
	};

	for (int iter = 0; iter < 6; iter++) {
		float vdot = 0.f;
		float hdot = 0.f;
		evaluate_reference(alpha, thrust, vdot, hdot);
		const float f_v = vdot;
		const float f_h = hdot - hdot_des;

		if (fabsf(f_v) < 0.05f && fabsf(f_h) < 0.15f) {
			converged = true;
			break;
		}

		float vdot_alpha = 0.f;
		float hdot_alpha = 0.f;
		float vdot_thrust = 0.f;
		float hdot_thrust = 0.f;
		evaluate_reference(math::constrain(alpha + alpha_eps, -0.35f, 0.35f), thrust, vdot_alpha, hdot_alpha);
		evaluate_reference(alpha, math::constrain(thrust + thrust_eps, _limits.u_min(3), _limits.u_max(3)),
				   vdot_thrust, hdot_thrust);

		const float j11 = (vdot_alpha - vdot) / alpha_eps;
		const float j12 = (vdot_thrust - vdot) / thrust_eps;
		const float j21 = (hdot_alpha - hdot) / alpha_eps;
		const float j22 = (hdot_thrust - hdot) / thrust_eps;
		const float det = j11 * j22 - j12 * j21;

		if (!PX4_ISFINITE(det) || fabsf(det) < 1e-5f) {
			break;
		}

		float d_alpha = ((-f_v) * j22 + j12 * f_h) / det;
		float d_thrust = (j21 * f_v - j11 * f_h) / det;

		d_alpha = math::constrain(d_alpha, -0.05f, 0.05f);
		d_thrust = math::constrain(d_thrust, -2.0f, 2.0f);

		alpha = math::constrain(alpha + 0.8f * d_alpha, -0.35f, 0.35f);
		thrust = math::constrain(thrust + 0.8f * d_thrust, _limits.u_min(3), _limits.u_max(3));
	}

	alpha_ref = alpha;
	theta_ref = alpha + gamma_ref;
	thrust_ref = thrust;
	return converged;
}

bool FwMpcController::step(const StateVec &x_now, const matrix::Vector3f &goal_up, float V_cruise, bool is_last,
			   float obstacle_attention_distance, float dt_real_s, ControlVec &u_apply, StateVec &x_next)
{
	(void)is_last; // currently unused
	const float dt_step_s = math::constrain(dt_real_s, 0.f, 0.5f);

	if (_have_fallback_trajectory) {
		_time_since_last_solve_s += dt_step_s;

	} else {
		_time_since_last_solve_s = 0.f;
	}

	matrix::Matrix<float, 3, kMaxHorizon> x_ref_seq{};

	for (int k = 0; k < _N; k++) {
		x_ref_seq(0, k) = goal_up(0);
		x_ref_seq(1, k) = goal_up(1);
		x_ref_seq(2, k) = goal_up(2);
	}

	const Weights base_weights = _weights;
	const matrix::Matrix<float, kControlSize, kMaxHorizon> base_ubar = _ubar;
	bool solved = false;
	int solved_tier = -1;
	float last_attempt_slack_max = 0.f;

	const Vector3f uvw_now{x_now(0), x_now(1), x_now(2)};
	const matrix::Dcmf R_nb = rotation_matrix(x_now(6), x_now(7), x_now(8));
	const Vector3f vel_ned = R_nb * uvw_now;
	const Vector2f vel_xy{vel_ned(0), vel_ned(1)};
	const Vector3f p_now{x_now(9), x_now(10), x_now(11)};

	auto nearest_obstacle_distance = [&](bool include_planning_margin) {
		if (_n_obstacles <= 0) {
			return INFINITY;
		}

		float nearest = INFINITY;

		for (int j = 0; j < _n_obstacles; j++) {
			const Obstacle &obs = _obstacles[j];
			const float planning_margin = include_planning_margin ? math::max(obs.planning_margin, 0.f) : 0.f;
			const float radius = obs.R + obs.margin + planning_margin;
			const Vector2f obs_xy{obs.c(0), obs.c(1)};
			const Vector2f rel_xy = obs_xy - Vector2f{p_now(0), p_now(1)};

			if (!obstacle_is_ahead(rel_xy, vel_xy, radius)) {
				continue;
			}

			const float horizontal_distance_to_surface = rel_xy.norm() - radius;
			float obstacle_distance = horizontal_distance_to_surface;

			if (PX4_ISFINITE(obs.height) && obs.height > 0.f) {
				const float half_height = 0.5f * obs.height + obs.margin;
				const float vertical_distance_to_surface = fabsf(p_now(2) - obs.c(2)) - half_height;

				if (vertical_distance_to_surface > 0.f) {
					continue;
				}

				obstacle_distance = math::max(horizontal_distance_to_surface, vertical_distance_to_surface);
			}

			nearest = math::min(nearest, obstacle_distance);
		}

		return nearest;
	};

	const float current_soft_distance = nearest_obstacle_distance(true);
	const float current_hard_distance = nearest_obstacle_distance(false);
	const bool real_soft_threat = PX4_ISFINITE(current_soft_distance)
				      && current_soft_distance < math::max(base_weights.obstacle_proximity_distance, 12.f);
	const bool real_hard_threat = PX4_ISFINITE(current_hard_distance)
				      && current_hard_distance < math::max(0.5f * base_weights.obstacle_proximity_distance, 6.f);
	const bool very_close_hard_threat = PX4_ISFINITE(current_hard_distance) && current_hard_distance < 4.f;
	int nIt = (_options.mode == Mode::LMPC) ? 1 : 2;

	if (real_soft_threat) {
		nIt = math::max(nIt, (_options.mode == Mode::LMPC) ? 2 : 3);
	}

	if (real_hard_threat) {
		nIt = math::max(nIt, 3);
	}

	if (very_close_hard_threat) {
		nIt = math::max(nIt, 4);
	}

	for (int tier = 0; tier < 4; tier++) {
		if (tier >= 2 && !real_soft_threat && last_attempt_slack_max < 0.05f) {
			break;
		}

		if (tier >= 3 && !very_close_hard_threat && (!real_hard_threat || last_attempt_slack_max < 0.20f)) {
			break;
		}

		_weights = weights_for_solve_tier(base_weights, tier);
		_ubar = base_ubar;
		bool tier_solved = false;

		for (int it = 0; it < nIt; it++) {
			_xbar.col(0) = x_now;

			for (int k = 0; k < _N; k++) {
				const StateVec xk = _xbar.col(k);
				const ControlVec uk = _ubar.col(k);
				_xbar.col(k + 1) = fd_step(xk, uk);
			}

			matrix::Vector<float, kMaxHorizon> theta_ref_seq{};
			matrix::Vector<float, kMaxHorizon> T_ref_seq{};

			if (_options.recompute_ff) {
				ff_refs_from_nominal(_xbar, x_ref_seq, V_cruise, theta_ref_seq, T_ref_seq);

			} else {
				for (int k = 0; k < _N; k++) {
					theta_ref_seq(k) = _theta_trim;
					T_ref_seq(k) = _T_trim;
				}
			}

			matrix::Vector<float, kMaxHorizon> psi_ref_seq{};
			heading_refs(_xbar, x_ref_seq, psi_ref_seq);

			std::array<StateMat, kMaxHorizon> Ak{};
			std::array<matrix::Matrix<float, kStateSize, kControlSize>, kMaxHorizon> Bk{};

			for (int k = 0; k < _N; k++) {
				const StateVec xk = _xbar.col(k);
				const ControlVec uk = _ubar.col(k);
				lin_fd(xk, uk, Ak[k], Bk[k]);
			}

			matrix::Vector<float, kMaxVars> z{};
			int n_vars = 0;
			int n_constraints = 0;

			if (buildQP(_xbar, _ubar, x_ref_seq, theta_ref_seq, T_ref_seq, psi_ref_seq, Ak, Bk, _N,
				    obstacle_attention_distance, n_vars, n_constraints)) {
				tier_solved = solveQP(z, n_vars, n_constraints);

			} else {
				_last_qp_debug = {};
				_last_qp_debug.solve_success = false;
				_last_qp_debug.solve_tier_used = tier;
				tier_solved = false;
			}

			_last_qp_debug.solve_tier_used = tier;
			const int Nz_dx = _N * kStateSize;

			for (int k = 0; k < _N; k++) {
				ControlVec du{};

				for (int i = 0; i < kControlSize; i++) {
					du(i) = z(Nz_dx + k * kControlSize + i);
				}

				ControlVec u_new = _ubar.col(k) + du;

				for (int i = 0; i < kControlSize; i++) {
					u_new(i) = math::constrain(u_new(i), _limits.u_min(i), _limits.u_max(i));
				}

				_ubar.col(k) = u_new;
			}
		}

		if (tier_solved) {
			solved = true;
			solved_tier = tier;
			break;
		}

		last_attempt_slack_max = PX4_ISFINITE(_last_qp_debug.active_slack_max) ? _last_qp_debug.active_slack_max : 0.f;
	}

	const bool use_fallback_trajectory = !solved && _have_fallback_trajectory;

	if (!solved && use_fallback_trajectory) {
		// Reuse the tail of the last successful plan instead of reusing a failed local update.
		_ubar = _fallback_ubar;

	} else if (!solved) {
		_ubar = base_ubar;
	}

	_weights = base_weights;
	if (solved_tier >= 0) {
		_last_qp_debug.solve_tier_used = solved_tier;
	}

	matrix::Matrix<float, kStateSize, kMaxHorizon + 1> selected_xbar{};
	matrix::Matrix<float, kControlSize, kMaxHorizon> solved_ubar{};
	ControlVec u_selected{};

	if (solved) {
		rolloutStateHorizon(x_now, _ubar, selected_xbar);
		solved_ubar = _ubar;
		_last_solved_xbar = selected_xbar;
		x_next = selected_xbar.col(1);
		u_selected = _ubar.col(0);

	} else if (use_fallback_trajectory) {
		const float fallback_age_s = math::max(_time_since_last_solve_s, 0.f);
		const int control_idx = math::constrain(static_cast<int>(floorf(fallback_age_s / math::max(_Ts, 1e-3f))),
						       0, _N - 1);
		const float lookahead_age_s = math::min(fallback_age_s + _Ts, static_cast<float>(_N) * _Ts);

		for (int i = 0; i < kControlSize; i++) {
			u_selected(i) = _fallback_ubar(i, control_idx);
		}

		if (!sampleStateHorizon(_last_solved_xbar, lookahead_age_s, x_next)) {
			x_next = fd_step(x_now, u_selected);
		}

	} else {
		rolloutStateHorizon(x_now, _ubar, selected_xbar);
		x_next = selected_xbar.col(1);
		u_selected = _ubar.col(0);
	}

	u_apply = u_selected;

	for (int i = 0; i < kControlSize; i++) {
		u_apply(i) = math::constrain(u_apply(i), _limits.u_min(i), _limits.u_max(i));
	}

	// Shift horizon
	if (_N > 1) {
		for (int k = 0; k < _N - 1; k++) {
			_ubar.col(k) = _ubar.col(k + 1);
		}

		_ubar.col(_N - 1) = _ubar.col(_N - 2);
	}

	rolloutStateHorizon(x_next, _ubar, _xbar);

	if (solved) {
		_fallback_ubar = solved_ubar;
		rolloutAppliedStateSequence(x_now, _fallback_ubar, _fallback_xapply);
		_have_fallback_trajectory = true;
		_time_since_last_solve_s = 0.f;
	}

	return solved || use_fallback_trajectory;
}

void FwMpcController::rolloutStateHorizon(const StateVec &x0,
		const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar,
		matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar) const
{
	xbar.col(0) = x0;

	for (int k = 0; k < _N; k++) {
		const StateVec xk = xbar.col(k);
		ControlVec uk{};

		for (int i = 0; i < kControlSize; i++) {
			uk(i) = ubar(i, k);
		}

		xbar.col(k + 1) = fd_step(xk, uk);
	}
}

void FwMpcController::rolloutAppliedStateSequence(const StateVec &x0,
		const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar,
		matrix::Matrix<float, kStateSize, kMaxHorizon> &xapply) const
{
	StateVec xk = x0;

	for (int k = 0; k < _N; k++) {
		ControlVec uk{};

		for (int i = 0; i < kControlSize; i++) {
			uk(i) = ubar(i, k);
		}

		xk = fd_step(xk, uk);
		xapply.col(k) = xk;
	}
}

bool FwMpcController::sampleStateHorizon(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
		float age_s, StateVec &x_sampled) const
{
	if (_N < 1 || _Ts <= 1e-3f) {
		return false;
	}

	const float clamped_age_s = math::constrain(age_s, 0.f, static_cast<float>(_N) * _Ts);
	const float step_pos = clamped_age_s / _Ts;
	const int idx0 = math::constrain(static_cast<int>(floorf(step_pos)), 0, _N);
	const int idx1 = math::min(idx0 + 1, _N);
	const float alpha = math::constrain(step_pos - static_cast<float>(idx0), 0.f, 1.f);
	const StateVec x0 = xbar.col(idx0);
	const StateVec x1 = xbar.col(idx1);

	for (int i = 0; i < kStateSize; i++) {
		x_sampled(i) = x0(i) + alpha * (x1(i) - x0(i));
	}

	for (int i = 6; i <= 8; i++) {
		x_sampled(i) = matrix::wrap_pi(x0(i) + alpha * matrix::wrap_pi(x1(i) - x0(i)));
	}

	return true;
}

FwMpcController::Weights FwMpcController::weights_for_solve_tier(const Weights &base_weights, int tier) const
{
	Weights tier_weights = base_weights;

	switch (tier) {
	case 0:
		break;

	case 1:
		tier_weights.Rdu *= 0.25f;
		tier_weights.Ru_abs *= 0.25f;
		tier_weights.Rrate_diag *= 0.35f;
		break;

	case 2:
		tier_weights.Rdu *= 0.15f;
		tier_weights.Ru_abs *= 0.15f;
		tier_weights.Rrate_diag *= 0.25f;
		tier_weights.avoidance_tracking_scale_min = math::min(tier_weights.avoidance_tracking_scale_min, 0.15f);
		tier_weights.avoidance_terminal_scale_min = math::min(tier_weights.avoidance_terminal_scale_min, 0.05f);
		tier_weights.obstacle_proximity_weight *= 1.25f;
		break;

	default:
		// Tier 3 should be a tighter safety-focused fallback, not a numerically wild mode.
		tier_weights.Rdu *= 0.35f;
		tier_weights.Ru_abs *= 0.35f;
		tier_weights.Rrate_diag *= 0.45f;
		tier_weights.avoidance_tracking_scale_min = math::min(tier_weights.avoidance_tracking_scale_min, 0.18f);
		tier_weights.avoidance_terminal_scale_min = math::min(tier_weights.avoidance_terminal_scale_min, 0.08f);
		tier_weights.avoidance_control_scale_min = math::min(tier_weights.avoidance_control_scale_min, 0.28f);
		tier_weights.obstacle_proximity_weight *= 1.15f;
		break;
	}

	return tier_weights;
}

FwMpcController::StateVec FwMpcController::fd_step(const StateVec &x0, const ControlVec &u) const
{
	return _model.rk4_step(x0, u, _Ts);
}

void FwMpcController::lin_fd(const StateVec &x, const ControlVec &u, StateMat &A,
			     matrix::Matrix<float, kStateSize, kControlSize> &B) const
{
	const StateVec f0 = fd_step(x, u);
	const float epsx = 1e-4f;
	const float epsu = 1e-4f;

	for (int i = 0; i < kStateSize; i++) {
		StateVec dx = x;
		dx(i) += epsx;
		const StateVec f_plus = fd_step(dx, u);

		for (int r = 0; r < kStateSize; r++) {
			A(r, i) = (f_plus(r) - f0(r)) / epsx;
		}
	}

	for (int j = 0; j < kControlSize; j++) {
		ControlVec du = u;
		du(j) += epsu;
		const StateVec f_plus = fd_step(x, du);

		for (int r = 0; r < kStateSize; r++) {
			B(r, j) = (f_plus(r) - f0(r)) / epsu;
		}
	}
}

void FwMpcController::ff_refs_from_nominal(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
		const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq, float Vc,
		matrix::Vector<float, kMaxHorizon> &theta_ref_seq,
		matrix::Vector<float, kMaxHorizon> &T_ref_seq) const
{
	float alpha_seed = _alpha_trim;
	float thrust_seed = _T_trim;

	for (int k = 0; k < _N; k++) {
		const int state_idx = math::min(k + 1, _N);
		const Vector3f vel{xbar(0, state_idx), xbar(1, state_idx), xbar(2, state_idx)};
		const float V = math::max(vel.norm(), math::max(Vc, 3.f));
		const float phi = xbar(6, state_idx);
		const float psi = xbar(8, state_idx);
		const float z_now = xbar(11, state_idx);
		const float z_targ = x_ref_seq(2, k);

		float vz_des = (z_targ - z_now) / _Ts;
		vz_des = math::constrain(vz_des, -0.2f * Vc, 0.2f * Vc);
		const float gamma_ref = asinf(math::constrain(vz_des / V, -0.25f, 0.25f));
		float alpha_ref = alpha_seed;
		float theta_ref = _theta_trim + gamma_ref;
		float thrust_ref = thrust_seed;

		if (!solve_model_steady_reference(V, z_now, phi, psi, gamma_ref, alpha_seed, thrust_seed,
						  alpha_ref, theta_ref, thrust_ref)) {
			theta_ref = _theta_trim + gamma_ref;
			thrust_ref = thrust_seed;
		}

		alpha_seed = alpha_ref;
		thrust_seed = thrust_ref;
		theta_ref_seq(k) = theta_ref;
		T_ref_seq(k) = thrust_ref;
	}
}

void FwMpcController::heading_refs(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
				   const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
				   matrix::Vector<float, kMaxHorizon> &psi_ref) const
{
	for (int k = 0; k < _N; k++) {
		const int state_idx = math::min(k + 1, _N);
		const Vector3f pos{xbar(9, state_idx), xbar(10, state_idx), xbar(11, state_idx)};
		const Vector3f dp = Vector3f{x_ref_seq(0, k), x_ref_seq(1, k), x_ref_seq(2, k)} - pos;
		psi_ref(k) = atan2f(dp(1), dp(0));
	}
}

float FwMpcController::angDiff(float a, float b) const
{
	return matrix::wrap_pi(a - b);
}

void FwMpcController::computeActiveObstacles(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar, int N,
		float obstacle_attention_distance, std::array<bool, kMaxObstacles> &active_obstacles) const
{
	if (_n_obstacles <= 0) {
		return;
	}

	const float attention_distance = math::max(obstacle_attention_distance, 0.f);
	const float proximity_distance = math::max(_weights.obstacle_proximity_distance, 0.f);
	const float relevance_distance = math::max(attention_distance, proximity_distance);

	for (int j = 0; j < _n_obstacles; j++) {
		float min_distance = INFINITY;

		for (int k = 0; k <= N; k++) {
			const Vector3f pbar{xbar(9, k), xbar(10, k), xbar(11, k)};

			if (PX4_ISFINITE(_obstacles[j].height) && _obstacles[j].height > 0.f) {
				const float half_height_buffered = 0.5f * _obstacles[j].height + _obstacles[j].margin;
				const float vertical_distance_to_surface = fabsf(pbar(2) - _obstacles[j].c(2)) - half_height_buffered;

				if (vertical_distance_to_surface > relevance_distance) {
					continue;
				}
			}

			const float Rbuf = _obstacles[j].R + _obstacles[j].margin + _obstacles[j].planning_margin;
			const Vector2f dvec_xy{pbar(0) - _obstacles[j].c(0), pbar(1) - _obstacles[j].c(1)};
			const float horizontal_distance_to_surface = dvec_xy.norm() - Rbuf;
			min_distance = math::min(min_distance, horizontal_distance_to_surface);
		}

		active_obstacles[j] = PX4_ISFINITE(min_distance) && min_distance <= relevance_distance;
	}
}

bool FwMpcController::buildQP(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
				      const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar,
				      const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
				      const matrix::Vector<float, kMaxHorizon> &theta_ref_seq,
				      const matrix::Vector<float, kMaxHorizon> &T_ref_seq,
				      const matrix::Vector<float, kMaxHorizon> &psi_ref_seq,
				      const std::array<StateMat, kMaxHorizon> &Ak,
				      const std::array<matrix::Matrix<float, kStateSize, kControlSize>, kMaxHorizon> &Bk,
				      int N, float obstacle_attention_distance, int &n_vars, int &n_constraints)
{
	const int n = kStateSize;
	const int m = kControlSize;
	const int Nz_dx = N * n;
	const int Nz_du = N * m;
	const int Nz_slack = N * _n_obstacles;
	std::array<bool, kMaxObstacles> active_obstacles{};
	active_obstacles.fill(false);
	computeActiveObstacles(xbar, N, obstacle_attention_distance, active_obstacles);
	n_vars = Nz_dx + Nz_du + Nz_slack;

	if (n_vars > kMaxVars) {
		PX4_ERR("QP var buffer overflow (%d > %d)", n_vars, kMaxVars);
		return false;
	}

	_H.setZero();
	_f.setZero();
	_A.setZero();

	for (int i = 0; i < kMaxConstraints; i++) {
		_l(i) = -OSQP_INFTY;
		_u(i) = OSQP_INFTY;
	}

	// Equality constraints for linearized dynamics
	int row = 0;

	for (int k = 0; k < N; k++) {
		const int idx_dxk = k * n;

		for (int i = 0; i < n; i++) {
			_A(row + i, idx_dxk + i) = 1.f;
			_l(row + i) = 0.f;
			_u(row + i) = 0.f;
		}

		if (k == 0) {
			const int idx_du0 = Nz_dx;

			for (int i = 0; i < n; i++) {
				for (int j = 0; j < m; j++) {
					_A(row + i, idx_du0 + j) = -Bk[0](i, j);
				}
			}

		} else {
			const int idx_dxkm1 = (k - 1) * n;
			const int idx_dukm1 = Nz_dx + (k - 1) * m;

			for (int i = 0; i < n; i++) {
				for (int j = 0; j < n; j++) {
					_A(row + i, idx_dxkm1 + j) = -Ak[k - 1](i, j);
				}

				for (int j = 0; j < m; j++) {
					_A(row + i, idx_dukm1 + j) = -Bk[k - 1](i, j);
				}
			}
		}

		row += n;
	}

	int row_offset = row;

	// Selectors
	Matrix<float, 3, n> Spos{};
	Spos(0, 9) = 1.f;
	Spos(1, 10) = 1.f;
	Spos(2, 11) = 1.f;
	Matrix<float, 3, n> Sang{};
	Sang(0, 6) = 1.f;
	Sang(1, 7) = 1.f;
	Sang(2, 8) = 1.f;

	const Matrix<float, n, 3> Spos_T = Spos.transpose();
	const Matrix<float, n, 3> Sang_T = Sang.transpose();

	Matrix<float, n, n> epsI{};
	epsI.setZero();

	for (int i = 0; i < n; i++) {
		epsI(i, i) = 2.f * 1e-5f;
	}

	const Matrix<float, n, n> Qpos = 2.f * (Spos_T * (_weights.Qp * Spos));
	const Matrix<float, n, n> Qang = 2.f * (Sang_T * (_weights.Qang * Sang));
	const float obs_distance = math::max(_weights.obstacle_proximity_distance, 0.f);
	const float tracking_scale_min = math::constrain(_weights.avoidance_tracking_scale_min, 0.05f, 1.f);
	const float terminal_scale_min = math::constrain(_weights.avoidance_terminal_scale_min, 0.02f, 1.f);
	const float control_scale_min = math::constrain(_weights.avoidance_control_scale_min, 0.05f, 1.f);
	std::array<float, kMaxHorizon> stage_avoidance_gain{};
	stage_avoidance_gain.fill(0.f);
	float max_horizon_avoidance_gain = 0.f;

	auto stage_obstacle_urgency = [&](const Vector3f &pbar) {
		if (_n_obstacles <= 0 || obs_distance <= 0.f) {
			return 0.f;
		}

		float max_urgency = 0.f;

		for (int j = 0; j < _n_obstacles; j++) {
			if (!active_obstacles[j]) {
				continue;
			}

			if (PX4_ISFINITE(_obstacles[j].height) && _obstacles[j].height > 0.f) {
				const float half_height_buffered = 0.5f * _obstacles[j].height + _obstacles[j].margin;
				const float vertical_distance_to_surface = fabsf(pbar(2) - _obstacles[j].c(2)) - half_height_buffered;

				if (vertical_distance_to_surface > 0.f) {
					continue;
				}
			}

			const float Rbuf = _obstacles[j].R + _obstacles[j].margin + _obstacles[j].planning_margin;
			Vector2f dvec_xy{pbar(0) - _obstacles[j].c(0), pbar(1) - _obstacles[j].c(1)};
			float d_xy = dvec_xy.norm();

			if (d_xy < 1e-6f) {
				d_xy = 1e-6f;
			}

			const float proximity = (Rbuf + obs_distance) - d_xy;

			if (proximity <= 0.f) {
				continue;
			}

			max_urgency = math::max(max_urgency, math::constrain(proximity / math::max(obs_distance, 1e-3f), 0.f, 1.f));
		}

		return max_urgency;
	};

	// Stage costs
	for (int k = 0; k < N; k++) {
		const int idx_dxk = k * n;
		const int idx_duk = Nz_dx + k * m;

		const StateVec xk = xbar.col(k + 1);
			const Vector3f ref_k{x_ref_seq(0, k), x_ref_seq(1, k), x_ref_seq(2, k)};
			const Vector3f epos = (Spos * xk) - ref_k;
			const Vector3f eang{xk(6), xk(7) - theta_ref_seq(k), angDiff(xk(8), psi_ref_seq(k))};
			const Vector3f pbar{xk(9), xk(10), xk(11)};
			const float obstacle_urgency = stage_obstacle_urgency(pbar);
			const float avoidance_gain = sqrtf(math::constrain(obstacle_urgency, 0.f, 1.f));
			stage_avoidance_gain[k] = avoidance_gain;
			const float tracking_scale = 1.f - avoidance_gain * (1.f - tracking_scale_min);
			const float control_scale = 1.f - avoidance_gain * (1.f - control_scale_min);
			max_horizon_avoidance_gain = math::max(max_horizon_avoidance_gain, avoidance_gain);

		const Matrix<float, n, n> Hdx = tracking_scale * (Qpos + Qang) + epsI;
		const matrix::Vector<float, kStateSize> fdx = 2.f * tracking_scale
						      * (Spos_T * (_weights.Qp * epos) + Sang_T * (_weights.Qang * eang));

		for (int i = 0; i < n; i++) {
			_f(idx_dxk + i) += fdx(i);

			for (int j = 0; j < n; j++) {
				_H(idx_dxk + i, idx_dxk + j) += Hdx(i, j);
			}
		}

			// Keep a light absolute-control regularization, but do not heavily penalize
			// the first move itself. Oscillation suppression is handled below with the
			// second-difference smoothness penalty.
			const Matrix<float, m, m> Hdu = 2.f * control_scale * (0.15f * _weights.Rdu + _weights.Ru_abs);

		for (int i = 0; i < m; i++) {
			for (int j = 0; j < m; j++) {
				_H(idx_duk + i, idx_duk + j) += Hdu(i, j);
			}
		}

		const ControlVec u_ref{0.f, 0.f, 0.f, T_ref_seq(k)};
		const ControlVec fdu = 2.f * control_scale * (_weights.Ru_abs * (ubar.col(k) - u_ref));

		for (int i = 0; i < m; i++) {
			_f(idx_duk + i) += fdu(i);
		}

		// Soft proximity cost around obstacles to encourage earlier lateral avoidance.
			const float obs_weight = math::max(_weights.obstacle_proximity_weight, 0.f) * (1.f + 2.f * avoidance_gain);

		if (obs_weight > 0.f && obs_distance > 0.f && _n_obstacles > 0) {
			for (int j = 0; j < _n_obstacles; j++) {
				if (!active_obstacles[j]) {
					continue;
				}

				if (PX4_ISFINITE(_obstacles[j].height) && _obstacles[j].height > 0.f) {
					const float half_height_buffered = 0.5f * _obstacles[j].height + _obstacles[j].margin;
					const float vertical_distance_to_surface = fabsf(pbar(2) - _obstacles[j].c(2)) - half_height_buffered;

					// Same height gating as obstacle constraints.
					if (vertical_distance_to_surface > 0.f) {
						continue;
					}
				}

				const float Rbuf = _obstacles[j].R + _obstacles[j].margin + _obstacles[j].planning_margin;
				Vector2f dvec_xy{pbar(0) - _obstacles[j].c(0), pbar(1) - _obstacles[j].c(1)};
				float d_xy = dvec_xy.norm();

				if (d_xy < 1e-6f) {
					d_xy = 1e-6f;
					dvec_xy = Vector2f{1.f, 0.f};
				}

				const float proximity = (Rbuf + obs_distance) - d_xy;

				if (proximity <= 0.f) {
					continue;
				}

				const Vector2f grad_xy = -(dvec_xy / d_xy);
				const float two_w = 2.f * obs_weight;

				_f(idx_dxk + 9) += two_w * proximity * grad_xy(0);
				_f(idx_dxk + 10) += two_w * proximity * grad_xy(1);
				_H(idx_dxk + 9, idx_dxk + 9) += two_w * grad_xy(0) * grad_xy(0);
				_H(idx_dxk + 9, idx_dxk + 10) += two_w * grad_xy(0) * grad_xy(1);
				_H(idx_dxk + 10, idx_dxk + 9) += two_w * grad_xy(1) * grad_xy(0);
				_H(idx_dxk + 10, idx_dxk + 10) += two_w * grad_xy(1) * grad_xy(1);
			}
		}
	}

	// Terminal position cost on last dx block
	{
			const int idx_dxN = (N - 1) * n;
			const StateVec xN = xbar.col(N);
			const Vector3f ref_N{x_ref_seq(0, N - 1), x_ref_seq(1, N - 1), x_ref_seq(2, N - 1)};
			const Vector3f eN = (Spos * xN) - ref_N;
			const float terminal_scale = 1.f - max_horizon_avoidance_gain * (1.f - terminal_scale_min);
			const Matrix<float, n, n> Hterm = 2.f * terminal_scale * _weights.Qterm * (Spos_T * (_weights.Qp * Spos));
			const matrix::Vector<float, kStateSize> fterm = 2.f * terminal_scale * _weights.Qterm * (Spos_T * (_weights.Qp * eN));

		for (int i = 0; i < n; i++) {
			_f(idx_dxN + i) += fterm(i);

			for (int j = 0; j < n; j++) {
				_H(idx_dxN + i, idx_dxN + j) += Hterm(i, j);
			}
		}
	}

		// Smoothness along horizon: penalize second differences so monotonic steering
		// into and out of the turn is allowed, while oscillatory left-right maneuvers
		// are discouraged.
		if (_limits.use_stage_smoothness) {
			for (int k = 0; k < N - 2; k++) {
				const int idx_du0 = Nz_dx + k * m;
				const int idx_du1 = Nz_dx + (k + 1) * m;
				const int idx_du2 = Nz_dx + (k + 2) * m;
				const float segment_avoidance_gain = math::max(stage_avoidance_gain[k],
						       math::max(stage_avoidance_gain[k + 1], stage_avoidance_gain[k + 2]));
				const float smoothness_scale = 1.f - segment_avoidance_gain * (1.f - control_scale_min);

				for (int i = 0; i < m; i++) {
					const float w = smoothness_scale * _weights.Rrate_diag(i);

				if (w <= 0.f) {
					continue;
				}

					const float ddu_bar = ubar(i, k + 2) - 2.f * ubar(i, k + 1) + ubar(i, k);
					const float two_w = 2.f * w;

					_H(idx_du0 + i, idx_du0 + i) += two_w;
					_H(idx_du1 + i, idx_du1 + i) += 4.f * two_w;
					_H(idx_du2 + i, idx_du2 + i) += two_w;
					_H(idx_du0 + i, idx_du1 + i) -= 2.f * two_w;
					_H(idx_du1 + i, idx_du0 + i) -= 2.f * two_w;
					_H(idx_du1 + i, idx_du2 + i) -= 2.f * two_w;
					_H(idx_du2 + i, idx_du1 + i) -= 2.f * two_w;
					_H(idx_du0 + i, idx_du2 + i) += two_w;
					_H(idx_du2 + i, idx_du0 + i) += two_w;

					_f(idx_du0 + i) += two_w * ddu_bar;
					_f(idx_du1 + i) += -2.f * two_w * ddu_bar;
					_f(idx_du2 + i) += two_w * ddu_bar;
				}
			}
		}

	// Soft obstacle constraints: per-constraint nonnegative slack with strong linear/quadratic penalty.
	if (Nz_slack > 0) {
		const int idx_slack0 = Nz_dx + Nz_du;
		const float w_lin = math::max(_weights.obstacle_slack_linear, 0.f);
		const float w_quad = math::max(_weights.obstacle_slack_quadratic, 0.f);

		for (int i = 0; i < Nz_slack; i++) {
			const int idx = idx_slack0 + i;

			if (w_quad > 0.f) {
				_H(idx, idx) += 2.f * w_quad;
			}

			if (w_lin > 0.f) {
				_f(idx) += w_lin;
			}
		}
	}

	addObstacleConstraints(xbar, active_obstacles, N, row_offset, Nz_dx, Nz_du, Nz_slack);

	if (_limits.use_stage_smoothness && _limits.use_rate_limits) {
		addRateConstraints(ubar, N, row_offset, Nz_dx, Nz_du);
	}

	addBounds(ubar, N, row_offset, Nz_dx, Nz_du, Nz_slack);

	if (row_offset > kMaxConstraints) {
		PX4_ERR("QP constraint buffer overflow (%d > %d)", row_offset, kMaxConstraints);
		return false;
	}

	n_constraints = row_offset;
	return true;
}

bool FwMpcController::solveQP(matrix::Vector<float, kMaxVars> &z, int n_vars, int n_constraints)
{
	_last_qp_debug = {};
	_last_qp_debug.objective_value = NAN;
	_last_qp_debug.primal_residual = NAN;
	_last_qp_debug.dual_residual = NAN;
	_last_qp_debug.active_slack_max = NAN;
	_last_qp_debug.active_slack_sum = NAN;
	_last_qp_debug.solve_time_us = NAN;
	_last_qp_debug.status_polish = 0;

	// Symmetrize H
	for (int i = 0; i < n_vars; i++) {
		for (int j = i + 1; j < n_vars; j++) {
			const float v = 0.5f * (_H(i, j) + _H(j, i));
			_H(i, j) = v;
			_H(j, i) = v;
		}
	}

	std::vector<OSQPFloat> P_data;
	std::vector<OSQPInt> P_i;
	std::vector<OSQPInt> P_p;
	denseToCSC(_H, n_vars, n_vars, true, P_data, P_i, P_p);

	std::vector<OSQPFloat> A_data;
	std::vector<OSQPInt> A_i;
	std::vector<OSQPInt> A_p;
	denseToCSC(_A, n_constraints, n_vars, false, A_data, A_i, A_p);

	std::vector<OSQPFloat> q_vec(n_vars, 0.f);

	for (int i = 0; i < n_vars; i++) {
		q_vec[i] = _f(i);
	}

	std::vector<OSQPFloat> l_vec(n_constraints, -OSQP_INFTY);
	std::vector<OSQPFloat> u_vec(n_constraints, OSQP_INFTY);

	for (int i = 0; i < n_constraints; i++) {
		l_vec[i] = _l(i);
		u_vec[i] = _u(i);
	}

	OSQPCscMatrix *P = OSQPCscMatrix_new(n_vars, n_vars, (OSQPInt)P_data.size(), P_data.data(), P_i.data(),
					     P_p.data());
	OSQPCscMatrix *A = OSQPCscMatrix_new(n_constraints, n_vars, (OSQPInt)A_data.size(), A_data.data(), A_i.data(),
					     A_p.data());
	OSQPSettings *settings = OSQPSettings_new();

	if (!P || !A || !settings) {
		OSQPCscMatrix_free(A);
		OSQPCscMatrix_free(P);
		OSQPSettings_free(settings);
		_last_qp_status = -1;
		_last_qp_debug.solve_success = false;
		z.setZero();
		return false;
	}

	osqp_set_default_settings(settings);
	settings->verbose = 0;
	settings->alpha = 1.2f;
	settings->eps_abs = 1e-3f;
	settings->eps_rel = 1e-3f;
	settings->warm_starting = 1;
	settings->max_iter = 400;

	OSQPSolver *solver = nullptr;
	OSQPInt exitflag = osqp_setup(&solver, P, q_vec.data(), A, l_vec.data(), u_vec.data(), n_constraints, n_vars,
				      settings);

	if (exitflag == 0) {
		if (_have_warm_start && _warm_start_n_vars == n_vars) {
			std::vector<OSQPFloat> x_warm(n_vars, 0.f);

			for (int i = 0; i < n_vars; i++) {
				x_warm[i] = static_cast<OSQPFloat>(_warm_start_z(i));
			}

			(void)osqp_warm_start(solver, x_warm.data(), nullptr);
		}

		exitflag = osqp_solve(solver);
	}

	bool ok = false;

	if (solver && solver->info) {
		_last_qp_status = solver->info->status_val;
		_last_qp_debug.objective_value = static_cast<float>(solver->info->obj_val);
		_last_qp_debug.primal_residual = static_cast<float>(solver->info->prim_res);
		_last_qp_debug.dual_residual = static_cast<float>(solver->info->dual_res);
		_last_qp_debug.iterations = solver->info->iter;
		_last_qp_debug.status_polish = solver->info->status_polish;
		_last_qp_debug.solve_time_us = static_cast<float>(solver->info->solve_time * 1e6);

		ok = (solver->info->status_val == OSQP_SOLVED)
		     || (solver->info->status_val == OSQP_SOLVED_INACCURATE);

	} else {
		_last_qp_status = exitflag;
	}

	_last_qp_debug.solve_success = ok;

	const int idx_slack0 = _N * (kStateSize + kControlSize);
	const int n_slack = math::max(n_vars - idx_slack0, 0);

	if (ok && n_slack > 0 && solver && solver->solution && solver->solution->x) {
		float slack_max = 0.f;
		float slack_sum = 0.f;

		for (int i = 0; i < n_slack; i++) {
			const float slack = math::max(static_cast<float>(solver->solution->x[idx_slack0 + i]), 0.f);
			slack_max = math::max(slack_max, slack);
			slack_sum += slack;
		}

		_last_qp_debug.active_slack_max = slack_max;
		_last_qp_debug.active_slack_sum = slack_sum;
	}

	if (ok && solver && solver->solution && solver->solution->x) {
		for (int i = 0; i < n_vars; i++) {
			z(i) = solver->solution->x[i];
		}

		_warm_start_z.setZero();

		for (int i = 0; i < n_vars; i++) {
			_warm_start_z(i) = z(i);
		}

		_warm_start_n_vars = n_vars;
		_have_warm_start = true;

	} else {
		z.setZero();
	}

	osqp_cleanup(solver);
	OSQPCscMatrix_free(A);
	OSQPCscMatrix_free(P);
	OSQPSettings_free(settings);
	return ok;
}

void FwMpcController::addObstacleConstraints(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
		const std::array<bool, kMaxObstacles> &active_obstacles, int N, int &row_offset, int Nz_dx, int Nz_du, int Nz_slack)
{
	if (_n_obstacles <= 0) {
		return;
	}

	const int idx_slack0 = Nz_dx + Nz_du;

	for (int k = 0; k < N; k++) {
		const Vector3f pbar{xbar(9, k + 1), xbar(10, k + 1), xbar(11, k + 1)};
		const int idx_dxk = k * kStateSize;

		for (int j = 0; j < _n_obstacles; j++) {
			if (!active_obstacles[j]) {
				continue;
			}

			if (PX4_ISFINITE(_obstacles[j].height) && _obstacles[j].height > 0.f) {
				const float half_height_buffered = 0.5f * _obstacles[j].height + _obstacles[j].margin;
				const float vertical_distance_to_surface = fabsf(pbar(2) - _obstacles[j].c(2)) - half_height_buffered;

				// Finite-height obstacle does not constrain the solution outside its vertical span.
				if (vertical_distance_to_surface > 0.f) {
					continue;
				}
			}

			const float Rbuf = _obstacles[j].R + _obstacles[j].margin + _obstacles[j].planning_margin;
			Vector2f dvec_xy{pbar(0) - _obstacles[j].c(0), pbar(1) - _obstacles[j].c(1)};
			float d_xy = dvec_xy.norm();

			if (d_xy < 1e-6f) {
				d_xy = 1e-6f;
				dvec_xy = Vector2f{1.f, 0.f};
			}

			// Hard inner obstacle: exact obstacle radius + physical margin, no slack allowed.
			const float Rhard = _obstacles[j].R + _obstacles[j].margin;
			const float gbar_hard = Rhard - d_xy;
			const Vector2f gradg_xy = -(dvec_xy / d_xy);

			if (row_offset >= kMaxConstraints) {
				return;
			}

			_A(row_offset, idx_dxk + 9) = gradg_xy(0);
			_A(row_offset, idx_dxk + 10) = gradg_xy(1);
			_u(row_offset) = -gbar_hard;
			_l(row_offset) = -OSQP_INFTY;
			row_offset++;

			if (Nz_slack <= 0) {
				continue;
			}

			const int idx_slack = idx_slack0 + k * _n_obstacles + j;

			if (idx_slack >= (idx_slack0 + Nz_slack)) {
				continue;
			}

			// Soft outer planning buffer: same geometry but enlarged by planning margin.
			const float gbar_soft = Rbuf - d_xy;

			if (row_offset >= kMaxConstraints) {
				return;
			}

			_A(row_offset, idx_dxk + 9) = gradg_xy(0);
			_A(row_offset, idx_dxk + 10) = gradg_xy(1);
			_A(row_offset, idx_slack) = -1.f; // grad*dx <= -gbar + slack
			_u(row_offset) = -gbar_soft;
			_l(row_offset) = -OSQP_INFTY;
			row_offset++;
		}
	}
}

void FwMpcController::addRateConstraints(const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar, int N,
		int &row_offset, int Nz_dx, int Nz_du)
{
	(void)Nz_du;

	for (int k = 0; k < N - 1; k++) {
		const int idx_du = Nz_dx + k * kControlSize;
		const int idx_du_next = Nz_dx + (k + 1) * kControlSize;

		for (int i = 0; i < kControlSize; i++) {
			const float dubar = ubar(i, k + 1) - ubar(i, k);
			const float limit = _limits.du_rate(i);

			if (row_offset < kMaxConstraints) {
				_A(row_offset, idx_du_next + i) = 1.f;
				_A(row_offset, idx_du + i) = -1.f;
				_u(row_offset) = limit - dubar;
				_l(row_offset) = -OSQP_INFTY;
				row_offset++;
			}

			if (row_offset < kMaxConstraints) {
				_A(row_offset, idx_du_next + i) = -1.f;
				_A(row_offset, idx_du + i) = 1.f;
				_u(row_offset) = limit + dubar;
				_l(row_offset) = -OSQP_INFTY;
				row_offset++;
			}
		}
	}
}

void FwMpcController::addBounds(const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar, int N, int &row_offset,
				int Nz_dx, int Nz_du, int Nz_slack)
{
	for (int k = 0; k < N; k++) {
		const int idx_duk = Nz_dx + k * kControlSize;

		for (int i = 0; i < kControlSize; i++) {
			if (row_offset >= kMaxConstraints) {
				return;
			}

			_A(row_offset, idx_duk + i) = 1.f;
			_l(row_offset) = _limits.u_min(i) - ubar(i, k);
			_u(row_offset) = _limits.u_max(i) - ubar(i, k);
			row_offset++;
		}
	}

	const int idx_slack0 = Nz_dx + Nz_du;

	for (int i = 0; i < Nz_slack; i++) {
		if (row_offset >= kMaxConstraints) {
			return;
		}

		_A(row_offset, idx_slack0 + i) = 1.f;
		_l(row_offset) = 0.f;
		_u(row_offset) = OSQP_INFTY;
		row_offset++;
	}
}
