#pragma once

#include "FixedWingMpcModel.hpp"

#include <matrix/matrix/math.hpp>
#include <osqp.h>

#include <array>
#include <vector>

/**
 * Fixed-wing MPC controller.
 * Builds a horizon-based QP (linearized dynamics, smoothness, obstacle constraints)
 * and solves it with OSQP to generate control increments.
 */
class FwMpcController
{
public:
	static constexpr int kStateSize = FixedWingMpcModel::kStateSize;   // 12
	static constexpr int kControlSize = FixedWingMpcModel::kControlSize; // 4
	static constexpr int kMinHorizon = 2;
	static constexpr int kMaxHorizon = 64;
	static constexpr int kMaxObstacles = 4;
	// dx (N*n) + du (N*m) + obstacle slacks (N*max_obstacles)
	static constexpr int kMaxVars = kMaxHorizon * (kStateSize + kControlSize + kMaxObstacles);
	static constexpr int kMaxConstraints = 2400; // N*n eq + hard/soft obstacle + rate + bounds

	using StateVec = FixedWingMpcModel::State;
	using ControlVec = FixedWingMpcModel::Control;

	enum class Mode {
		LMPC,   // 1 RTI iteration
		SLMPC   // 2 RTI iterations
	};

	struct Weights {
		matrix::Matrix3f Qp{matrix::diag(matrix::Vector3f{4.f, 4.f, 6.f})};
		float Qterm{20.f};
		matrix::Matrix3f Qang{matrix::diag(matrix::Vector3f{0.25f, 0.60f, 0.80f})};
		matrix::SquareMatrix<float, kControlSize> Rdu{matrix::diag(matrix::Vector4f{0.35f, 0.45f, 0.35f, 0.12f})};
		matrix::SquareMatrix<float, kControlSize> Ru_abs{matrix::diag(matrix::Vector4f{0.03f, 0.03f, 0.03f, 0.03f})};
		matrix::Vector4f Rrate_diag{0.08f, 0.12f, 0.08f, 0.05f};
		float obstacle_proximity_weight{3.0f};
		float obstacle_proximity_distance{12.0f}; // [m] from obstacle surface
		float avoidance_tracking_scale_min{0.25f};
		float avoidance_terminal_scale_min{0.10f};
		float avoidance_control_scale_min{0.35f};
		float obstacle_slack_linear{5000.f};
		float obstacle_slack_quadratic{20000.f};
	};

	struct Limits {
		matrix::Vector4f u_min{-0.1f, -0.2f, -0.1f, 0.f};
		matrix::Vector4f u_max{0.1f, 0.3f, 0.1f, 24.f};
		bool use_stage_smoothness{true};
		bool use_rate_limits{false};
		// Absolute per-step input delta limit applied between horizon stages.
		matrix::Vector4f du_rate{0.15f, 0.15f, 0.15f, 0.30f};
	};

	struct Options {
		Mode mode{Mode::SLMPC};
		bool recompute_ff{true};
	};

	struct Obstacle {
		matrix::Vector3f c{0.f, 0.f, 0.f};
		float R{0.f};
		float height{0.f}; // <= 0 means unbounded in Z
		float margin{0.f};
		float planning_margin{0.f};
	};

	struct QpDebug {
		bool solve_success{false};
		int solve_tier_used{0};
		float objective_value{0.f};
		float primal_residual{0.f};
		float dual_residual{0.f};
		float active_slack_max{0.f};
		float active_slack_sum{0.f};
		int iterations{0};
		int status_polish{0};
		float solve_time_us{0.f};
		float nonlinear_min_clearance{NAN};
		float accepted_step_scale{NAN};
		bool full_step_rejected{false};
	};

	FwMpcController() = default;

	bool configure(float Ts, int horizon);

	void set_obstacles(const std::vector<Obstacle> &obs);
	void clear_obstacles();

	StateVec initTrim(float V_trim, float z0_up, const matrix::Vector3f &goal_up);

	/**
	 * One MPC receding-horizon step.
	 * @param x_now current state (z up, same convention as FixedWingMpcModel)
	 * @param goal_up desired waypoint in inertial frame (z up)
	 * @param V_cruise reference cruise speed
	 * @param is_last whether this is the final waypoint (not used yet)
	 * @param u_apply control to apply (du integrated on top of nominal)
	 * @param x_next nominal state after applying u_apply for Ts
	 * @return true if a command is available (fresh QP solve or fallback from the last successful trajectory)
	 */
	bool step(const StateVec &x_now, const matrix::Vector3f &goal_up, float V_cruise, bool is_last,
		  float obstacle_attention_distance, float dt_real_s, ControlVec &u_apply, StateVec &x_next);

	int last_qp_status() const { return _last_qp_status; }
	const QpDebug &last_qp_debug() const { return _last_qp_debug; }
	int horizon() const { return _N; }
	const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &last_solved_state_horizon() const { return _last_solved_xbar; }

	Weights &weights() { return _weights; }
	Limits &limits() { return _limits; }
	const Limits &limits() const { return _limits; }
	Options &options() { return _options; }

	const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar() const { return _ubar; }

	// Tune legacy MPC vehicle dynamics.
	void set_vehicle_params(float mass, const matrix::Vector3f &inertia_diag, float kdv, float kdw);
	void set_altitude_origin_amsl(float altitude_origin_amsl) { _model.set_altitude_origin_amsl(altitude_origin_amsl); }
	void set_wind_ned(const matrix::Vector3f &wind_ned) { _model.set_wind_ned(wind_ned); }
	void set_guidance_quality_factor(float guidance_quality_factor)
	{
		_guidance_quality_factor = math::constrain(guidance_quality_factor, 0.05f, 1.f);
	}
	void set_robustness_margin(float robustness_margin)
	{
		_robustness_margin = math::max(robustness_margin, 0.f);
	}

private:
	using StateMat = matrix::SquareMatrix<float, kStateSize>;

	struct StepReferenceData {
		matrix::Vector<float, kMaxHorizon> theta_ref_seq{};
		matrix::Vector<float, kMaxHorizon> T_ref_seq{};
		matrix::Vector<float, kMaxHorizon> psi_ref_seq{};
	};

	struct StepSolveContext {
		Weights base_weights{};
		matrix::Matrix<float, kControlSize, kMaxHorizon> base_ubar{};
		float effective_obstacle_attention_distance{0.f};
		float current_soft_distance{0.f};
		float current_hard_distance{0.f};
		int rti_iterations{1};
		bool real_soft_threat{false};
		bool real_hard_threat{false};
		bool very_close_hard_threat{false};
	};

	struct StepSolveResult {
		bool solved{false};
		int solved_tier{-1};
	};

	struct StepSelection {
		bool solved{false};
		bool use_fallback_trajectory{false};
		matrix::Matrix<float, kStateSize, kMaxHorizon + 1> selected_xbar{};
		matrix::Matrix<float, kControlSize, kMaxHorizon> solved_ubar{};
		ControlVec u_selected{};
		StateVec x_next{};
	};

	struct QpLayout {
		int N{0};		// horizon length
		int Nz_dx{0};		// number of state variables in the QP (N * n)
		int Nz_du{0};		// number of control variables in the QP (N * m)
		int Nz_slack{0};	// number of slack variables in the QP (N * max_obstacles)
		int n_vars{0};		// total number of variables in the QP (Nz_dx + Nz_du + Nz_slack)
	};

	struct QpCostContext {
		matrix::Matrix<float, 3, kStateSize> Spos{};
		matrix::Matrix<float, 3, kStateSize> Sang{};
		matrix::Matrix<float, kStateSize, 3> Spos_T{};
		matrix::Matrix<float, kStateSize, 3> Sang_T{};
		StateMat epsI{};
		StateMat Qpos{};
		StateMat Qang{};
		float obs_distance{0.f};
		float tracking_scale_min{1.f};
		float terminal_scale_min{1.f};
		float control_scale_min{1.f};
		std::array<float, kMaxHorizon> stage_avoidance_gain{};
		float max_horizon_avoidance_gain{0.f};
	};

	struct StageCostData {
		int stage_idx{0};
		int idx_dxk{0};
		int idx_duk{0};
		StateVec xk{};
		matrix::Vector3f ref_k{};
		matrix::Vector3f epos{};
		matrix::Vector3f eang{};
		matrix::Vector3f pbar{};
		float obstacle_urgency{0.f};
		float avoidance_gain{0.f};
		float tracking_scale{1.f};
		float control_scale{1.f};
	};

	StateVec fd_step(const StateVec &x0, const ControlVec &u) const;
	void lin_fd(const StateVec &x, const ControlVec &u, StateMat &A, matrix::Matrix<float, kStateSize, kControlSize> &B) const;
	bool solve_model_steady_reference(float V_target, float z_up, float phi_ref, float psi_ref, float gamma_ref,
					  float alpha_seed, float thrust_seed,
					  float &alpha_ref, float &theta_ref, float &thrust_ref) const;
	void update_step_timing(float dt_real_s);
	void fill_position_reference_sequence(const matrix::Vector3f &goal_up,
					 matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq) const;
	float nearest_forward_obstacle_distance(const StateVec &x_now, bool include_planning_margin) const;
	int compute_rti_iterations(bool real_soft_threat, bool real_hard_threat, bool very_close_hard_threat) const;
	StepSolveContext build_step_solve_context(const StateVec &x_now, float obstacle_attention_distance) const;
	bool should_attempt_solve_tier(const StepSolveContext &context, int tier, float last_attempt_slack_max) const;
	void compute_step_reference_data(const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq, float V_cruise,
					 StepReferenceData &references) const;
	void linearize_step_horizon(std::array<StateMat, kMaxHorizon> &Ak,
				    std::array<matrix::Matrix<float, kStateSize, kControlSize>, kMaxHorizon> &Bk) const;
	void apply_qp_solution(const matrix::Vector<float, kMaxVars> &z);
	bool solve_step_tier(const StateVec &x_now, const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
			     float V_cruise, const StepSolveContext &context, int tier);
	StepSolveResult solve_step_problem(const StateVec &x_now, const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
					   float V_cruise, const StepSolveContext &context);
	bool accept_solved_trajectory(const StateVec &x_now,
				      const matrix::Matrix<float, kControlSize, kMaxHorizon> &base_ubar,
				      StepSelection &selection);
	void select_fallback_trajectory(const StateVec &x_now, StepSelection &selection);
	void select_nominal_trajectory(const StateVec &x_now, StepSelection &selection);
	StepSelection select_step_trajectory(const StateVec &x_now, const StepSolveContext &context,
					     const StepSolveResult &solve_result);
	void constrain_selected_control(ControlVec &u_apply) const;
	void shift_control_horizon();
	void cache_solved_trajectory(const StateVec &x_now,
				     const matrix::Matrix<float, kControlSize, kMaxHorizon> &solved_ubar);
	void finalize_step_state(const StateVec &x_now, const StepSelection &selection);
	void ff_refs_from_nominal(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
				  const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq, float Vc,
				  matrix::Vector<float, kMaxHorizon> &theta_ref_seq,
				  matrix::Vector<float, kMaxHorizon> &T_ref_seq) const;
	void heading_refs(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
			  const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
			  matrix::Vector<float, kMaxHorizon> &psi_ref) const;
	float angDiff(float a, float b) const;

	bool buildQP(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
			     const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar,
			     const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
			     const matrix::Vector<float, kMaxHorizon> &theta_ref_seq,
			     const matrix::Vector<float, kMaxHorizon> &T_ref_seq,
			     const matrix::Vector<float, kMaxHorizon> &psi_ref_seq,
			     const std::array<StateMat, kMaxHorizon> &Ak,
			     const std::array<matrix::Matrix<float, kStateSize, kControlSize>, kMaxHorizon> &Bk,
			     int N, float obstacle_attention_distance, int &n_vars, int &n_constraints);
	bool initializeQpLayout(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
			       int N, float obstacle_attention_distance, QpLayout &layout,
			       std::array<bool, kMaxObstacles> &active_obstacles) const;
	void resetQpWorkspace();
	int addDynamicsConstraints(const std::array<StateMat, kMaxHorizon> &Ak,
				   const std::array<matrix::Matrix<float, kStateSize, kControlSize>, kMaxHorizon> &Bk,
				   const QpLayout &layout);
	QpCostContext buildQpCostContext() const;
	float stageObstacleUrgency(const matrix::Vector3f &pbar,
				   const std::array<bool, kMaxObstacles> &active_obstacles,
				   float obs_distance) const;
	StageCostData buildStageCostData(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
					 const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
					 const matrix::Vector<float, kMaxHorizon> &theta_ref_seq,
					 const matrix::Vector<float, kMaxHorizon> &psi_ref_seq,
					 int stage_idx, const QpLayout &layout,
					 const std::array<bool, kMaxObstacles> &active_obstacles,
					 QpCostContext &cost_context) const;
	void addStageTrackingCost(const StageCostData &stage_data, const QpCostContext &cost_context);
	void addStageControlCost(const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar,
				 const matrix::Vector<float, kMaxHorizon> &T_ref_seq,
				 const StageCostData &stage_data);
	void addStageObstacleProximityCost(const std::array<bool, kMaxObstacles> &active_obstacles,
					   const StageCostData &stage_data, const QpCostContext &cost_context);
	void addStageCosts(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
			   const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar,
			   const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
			   const matrix::Vector<float, kMaxHorizon> &theta_ref_seq,
			   const matrix::Vector<float, kMaxHorizon> &T_ref_seq,
			   const matrix::Vector<float, kMaxHorizon> &psi_ref_seq,
			   const QpLayout &layout,
			   const std::array<bool, kMaxObstacles> &active_obstacles,
			   QpCostContext &cost_context);
	void addTerminalCost(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
			     const matrix::Matrix<float, 3, kMaxHorizon> &x_ref_seq,
			     const QpLayout &layout, const QpCostContext &cost_context);
	void addSmoothnessCost(const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar,
			       const QpLayout &layout, const QpCostContext &cost_context);
	void addSlackPenalties(const QpLayout &layout);
	bool solveQP(matrix::Vector<float, kMaxVars> &z, int n_vars, int n_constraints);
	Weights weights_for_solve_tier(const Weights &base_weights, int tier) const;
	void computeActiveObstacles(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar, int N,
				    float obstacle_attention_distance, std::array<bool, kMaxObstacles> &active_obstacles) const;

	void addObstacleConstraints(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
					    const std::array<bool, kMaxObstacles> &active_obstacles,
					    int N, int &row_offset, int Nz_dx, int Nz_du, int Nz_slack);
	void addRateConstraints(const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar, int N,
				int &row_offset, int Nz_dx, int Nz_du);
	void addBounds(const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar, int N,
		       int &row_offset, int Nz_dx, int Nz_du, int Nz_slack);
	void rolloutStateHorizon(const StateVec &x0, const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar,
				 matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar) const;
	void rolloutAppliedStateSequence(const StateVec &x0, const matrix::Matrix<float, kControlSize, kMaxHorizon> &ubar,
					 matrix::Matrix<float, kStateSize, kMaxHorizon> &xapply) const;
	bool sampleStateHorizon(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar,
			       float age_s, StateVec &x_sampled) const;
	float obstacle_signed_clearance(const matrix::Vector3f &p, const Obstacle &obs, bool include_planning_margin) const;
	float nonlinear_min_hard_clearance(const matrix::Matrix<float, kStateSize, kMaxHorizon + 1> &xbar, int N) const;

	FixedWingMpcModel _model{};

	float _Ts{0.1f};
	int _N{10};

	Weights _weights{};
	Limits _limits{};
	Options _options{};

	float _V_trim{13.f};
	float _alpha_trim{0.f};
	float _theta_trim{0.f};
	float _T_trim{0.f};

	matrix::Matrix<float, kStateSize, kMaxHorizon + 1> _xbar{};
	matrix::Matrix<float, kControlSize, kMaxHorizon> _ubar{};
	matrix::Matrix<float, kControlSize, kMaxHorizon> _fallback_ubar{};
	matrix::Matrix<float, kStateSize, kMaxHorizon> _fallback_xapply{};
	matrix::Matrix<float, kStateSize, kMaxHorizon + 1> _last_solved_xbar{};
	bool _have_fallback_trajectory{false};
	float _time_since_last_solve_s{0.f};
	float _guidance_quality_factor{1.f};
	float _robustness_margin{0.f};

	std::array<Obstacle, kMaxObstacles> _obstacles{};
	int _n_obstacles{0};

	// QP storage
	matrix::SquareMatrix<float, kMaxVars> _H{};
	matrix::Vector<float, kMaxVars> _f{};
	matrix::Matrix<float, kMaxConstraints, kMaxVars> _A{};
	matrix::Vector<float, kMaxConstraints> _l{};
	matrix::Vector<float, kMaxConstraints> _u{};
	int _last_qp_status{0};
	QpDebug _last_qp_debug{};
	matrix::Vector<float, kMaxVars> _warm_start_z{};
	int _warm_start_n_vars{0};
	bool _have_warm_start{false};
};
