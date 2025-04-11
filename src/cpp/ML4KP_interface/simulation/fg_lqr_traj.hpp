#pragma once
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <prx/factor_graphs/factors/prx_propagation_factor.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include <prx/factor_graphs/utilities/dbg_utills.hpp>

namespace double_pendulum
{
using GraphValues = std::pair<gtsam::NonlinearFactorGraph, gtsam::Values>;
using GainMap = std::map<gtsam::Key, Eigen::Matrix<double, 1, 4>>;
using CostToGoMap = std::map<gtsam::Key, Eigen::Matrix<double, 4, 4>>;
using SF = prx::fg::symbol_factory_t;

template <typename State, typename Goal>
Eigen::Vector4d difference(const State x, const Goal goal)
{
  Eigen::Vector4d diff{ x - goal };
  diff[0] = std::atan2(std::sin(diff[0]), std::cos(diff[0]));
  diff[1] = std::atan2(std::sin(diff[1]), std::cos(diff[1]));
  return diff;
}

class double_pendulum_propagation_t
  : public prx::fg::plant_propagation_StateStateCtrl_factor_t<Eigen::Vector4d, Eigen::Vector<double, 1>>
{
  using State = Eigen::Vector4d;
  using Control = Eigen::Vector<double, 1>;
  using Base = prx::fg::plant_propagation_StateStateCtrl_factor_t<State, Control>;

public:
  template <typename... Args>
  double_pendulum_propagation_t(Args... args) : Base(args...){};
  virtual State error_(const State& x1, const State& prediction) const override
  {
    const State error{ difference(prediction, x1) };
    // PRX_DBG_VARS(x1.transpose(), prediction.transpose(), error.transpose());
    return error;
  }
  // virtual State predict_(const State& x0, const Control& u, const double& dt) const override
  // {
  //   const State prediction{ Base::predict_(x0, u, dt) };
  //   PRX_DBG_VARS(x0.transpose(), u.transpose(), dt, prediction.transpose());
  //   return prediction;
  // }
};

// Take a start state, controller and a plant and get the equivalent plan
template <typename StartState, typename LqrPtr, typename SystemGroupPtr>
void lqr_to_plan(const StartState x0, LqrPtr lqr, prx::condition_check_t& cond, prx::trajectory_t& traj,
                 prx::plan_t& plan, SystemGroupPtr sg)
{
  cond.reset();
  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  prx::space_point_t ui{ cs->make_point() };

  ss->copy_from(x0);
  const Eigen::Vector4d goal{ Eigen::Vector4d(prx::constants::pi, 0.0, 0.0, 0.0) };
  do
  {
    lqr->compute_controls();
    sg->propagate_once(nullptr);
    cs->copy_to(ui);
    traj.copy_onto_back(ss);
    plan.copy_onto_back(ui, prx::simulation_step);
  } while (difference(Vec(traj.back()), goal).norm() > 1.0);
  // } while (not cond.check());
}

template <typename Graph>
Eigen::Matrix<double, 4, 4> compute_S(Graph& graph, const gtsam::Key key)
{
  // '''Returns the value function matrix at variable `key` given a graph which
  //     goes up and including `key`, but no further (i.e. all time steps after
  //     `key` have already been eliminated).  Does so by aggregating all unary
  //     factors on `key`.  If value function is x^TPx, then this returns P.
  //     "Return Cost" aka "Cost-to-go" aka "Value Function".
  // Arguments:
  //     graph: factor graph in LTI form
  //     key: key in the factor graph for which we want to obtain the return cost
  // Returns:
  //     return_cost: return cost, an nxn array where `n` is dimension of `key`
  // '''
  gtsam::GaussianFactorGraph new_fg{};
  // graph->print("Original FG (S)", SF::formatter);
  for (std::size_t i = 0; i < graph->size(); ++i)
  {
    auto f = graph->at(i);
    if (f->keys().size() == 1 and f->keys()[0] == key)  // # collect unary factors on `key`
    {
      new_fg.push_back(f);
    }
  }
  // new_fg.print("New FG (S)", SF::formatter);
  auto sol_end = new_fg.eliminateSequential();
  auto S = sol_end->back()->information();

  // PRX_DBG_VARS("Ricatti:", S);
  return S;
}

template <typename Graph, typename Xkeys, typename Ukeys, typename Sout, typename Kout>
void compute_K_S(Graph& graph, const Xkeys& X, const Ukeys& U, Sout& Ss, Kout& Ks)
{
  // def get_k_and_p(graph, X, U):
  // '''Finds optimal control law given by $u=Kx$ and value function $Vx^2$ aka
  //     cost-to-go which corresponds to solutions to the algebraic, finite
  //     horizon Ricatti Equation.  K is Extracted from the bayes net and V is
  //     extracted by incrementally eliminating the factor graph.  If you only
  //     need K and not V, then use the `get_k` function below.
  // Arguments:
  //     graph: factor graph containing factor graph in LQR form
  //     X: list of state Keys
  //     U: list of control Keys
  // Returns:
  //     K: optimal control matrix, shape (T-1, 1)
  //     V: value function, shape (T, 1)
  //         TODO(gerry): support n-dimensional state space
  // '''
  // # Find K and V by using bayes net solution
  auto marginalized_fg = graph;

  Ss[X.back()] = compute_S(marginalized_fg, X.back());
  // for i in range(len(U)-2, -1, -1): # traverse backwards in time
  for (int i = U.size() - 1; i > -1; --i)  // # traverse backwards in time
  {
    // PRX_DBG_VARS(i);
    const gtsam::Key keyX1{ X[i + 1] };
    const gtsam::Key keyX0{ X[i] };
    const gtsam::Key keyU01{ U[i] };

    const std::string strKeyX1{ SF::formatter(keyX1) };
    const std::string strKeyX0{ SF::formatter(keyX0) };
    const std::string strKeyU01{ SF::formatter(keyU01) };

    // PRX_DBG_VARS(strKeyX1, strKeyX0, strKeyU01);
    gtsam::Ordering ordering{};
    ordering.push_back(keyX1);
    ordering.push_back(keyU01);

    try
    {
      // std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> >
      // bayes_net, marginalized_fg = marginalized_fg.eliminatePartialSequential(ordering);
      auto pair_eliminated = marginalized_fg->eliminatePartialSequential(ordering);
      marginalized_fg = pair_eliminated.second;
      // pair_eliminated.first->print("pair_eliminated", SF::formatter);
      // marginalized_fg->print("marginalized_fg", SF::formatter);
      Ss[keyX0] = compute_S(marginalized_fg, keyX0);
      const Eigen::MatrixXd R{ pair_eliminated.first->back()->R().matrix() };
      const Eigen::MatrixXd S{ pair_eliminated.first->back()->S().matrix() };
      const Eigen::MatrixXd K{ R.triangularView<Eigen::Upper>().solve(S) };
      // PRX_DBG_VARS(strKeyX0);
      // PRX_DBG_VARS(Ss[keyX0]);
      // PRX_DBG_VARS(R);
      // PRX_DBG_VARS(S);
      // PRX_DBG_VARS(K);
      Ks[keyX0] = K;
    }
    catch (gtsam::IndeterminantLinearSystemException e)
    {
      const std::string msg{ "[EXCEPTION] Var:" + SF::formatter(e.nearbyVariable()) + "\n" };
      prx::fg::indeterminant_linear_system_helper(marginalized_fg);
      std::cout << msg << std::string(e.what()) << std::endl;
    }
  }
}

bool check_traj_plan(prx::trajectory_t& traj, prx::plan_t& plan, std::shared_ptr<prx::system_group_t> sg)
{
  prx::trajectory_t traj_test(sg->get_state_space());
  sg->propagate(traj.front(), plan, traj_test);

  // PRX_DBG_VARS(traj.back(), traj_test.back());

  return difference(Vec(traj.back()), Vec(traj_test.back())).norm() < 0.5;
}

void create_fg(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values, prx::trajectory_t& traj, prx::plan_t& plan,
               std::vector<gtsam::Key>& Xkeys, std::vector<gtsam::Key>& Ukeys, const std::string plant_name,
               const int step_size)
{
  using Propagation = double_pendulum_propagation_t;

  std::size_t i{ 0 };
  // double ti{ 0.0 };
  std::size_t iF{ plan.size() };
  const double tot_duration{ plan.duration() };

  const Eigen::Vector4d sigmas(1, 1, 1, 1);
  gtsam::noiseModel::Base::shared_ptr Q_noise_model{ gtsam::noiseModel::Diagonal::Sigmas(sigmas) };
  gtsam::noiseModel::Base::shared_ptr R_noise_model{ gtsam::noiseModel::Isotropic::Sigma(1, 1) };
  gtsam::noiseModel::Base::shared_ptr noise_integrator_model{ gtsam::noiseModel::Isotropic::Sigma(4, 1) };
  // const int step_size{ 3 };
  const double step_duration{ step_size * prx::simulation_step };
  for (double ti = 0; ti < tot_duration; ti += step_duration)
  {
    const std::size_t parent_id{ i };
    const std::size_t target_id{ i + step_size };
    // PRX_DBG_VARS(ti, i);
    if (ti + step_duration < tot_duration)
    {
      auto step_control = plan.at(ti);
      const gtsam::Key key_xt0{ SF::create_hashed_symbol("X^{", parent_id, "}") };
      const gtsam::Key key_xt1{ SF::create_hashed_symbol("X^{", target_id, "}") };
      const gtsam::Key key_u{ SF::create_hashed_symbol("U^{", parent_id, "}_{", target_id, "}") };

      Xkeys.push_back(key_xt0);
      Ukeys.push_back(key_u);

      const double duration{ step_duration };
      const Eigen::Vector<double, 1> u{ Vec(step_control) };
      const double norm_dur0{ ti / tot_duration };
      const double norm_dur1{ (ti + duration) / tot_duration };
      // PRX_DBG_VARS(ti, duration, ti + duration, plan.duration(), norm_dur, norm_dur <= 1.0)
      const Eigen::Vector4d x0{ Vec(traj.at(norm_dur0, true)) };
      const Eigen::Vector4d x1{ Vec(traj.at(norm_dur1, true)) };

      values.insert(key_xt0, x0);
      // values.insert_or_assign(key_xt1, x1);
      values.insert(key_u, u);

      graph.emplace_shared<Propagation>(key_xt1, key_xt0, key_u, noise_integrator_model, duration, plant_name);

      graph.addPrior(key_xt0, x0, Q_noise_model);
      graph.addPrior(key_u, u, R_noise_model);
      // ti = ti + duration;
    }
    if (ti + step_duration >= tot_duration)
    {
      // PRX_DBG_VARS(ti, target_id);
      auto step_control = plan.at(ti);
      const double dt{ tot_duration - ti };
      const Eigen::Vector<double, 1> u{ Vec(step_control) };
      const gtsam::Key key_xt0{ SF::create_hashed_symbol("X^{", parent_id, "}") };
      const gtsam::Key key_xF{ SF::create_hashed_symbol("X^{", target_id, "}") };
      const gtsam::Key key_u{ SF::create_hashed_symbol("U^{", parent_id, "}_{", target_id, "}") };
      const double norm_dur0{ ti / tot_duration };
      const Eigen::Vector4d x0{ Vec(traj.at(norm_dur0, true)) };
      const Eigen::Vector4d xF{ Vec(traj.back()) };
      values.insert(key_u, u);
      values.insert(key_xt0, x0);
      values.insert(key_xF, xF);
      Ukeys.push_back(key_u);
      Xkeys.push_back(key_xt0);
      Xkeys.push_back(key_xF);
      graph.emplace_shared<Propagation>(key_xF, key_xt0, key_u, noise_integrator_model, dt, plant_name);
      graph.addPrior(key_xt0, x0, Q_noise_model);
      graph.addPrior(key_xF, xF, Q_noise_model);
    }
    // PRX_DBG_VARS(ti, parent_id, target_id);
    i += step_size;
  }
}

void create_lqr_fg(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values, std::vector<gtsam::Key>& Xkeys,
                   std::vector<gtsam::Key>& Ukeys, GainMap& Ks, CostToGoMap& Ss)
{
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  // result.print("values", SF::formatter);
  // graph.printErrors(result, "graph", SF::formatter);
  // GainMap Ks;  // u = -k X ==>  (1x1)= (1x4) (4x1)
  // std::map<gtsam::Key, Eigen::MatrixXd> Ss;

  auto linearized_graph = graph.linearize(result);
  compute_K_S(linearized_graph, Xkeys, Ukeys, Ss, Ks);

  // return Ks;
}

template <typename Grid, typename Ks, typename SystemGroupPtr>
bool check_lqr_traj(gtsam::Values& values, Grid& grid, const prx::plan_t& plan, Ks& ks, SystemGroupPtr sg)
{
  // values.print("Vals", SF::formatter);
  // for (auto K_pair : ks)
  // {
  //   PRX_DBG_VARS(SF::formatter(K_pair.first), K_pair.second);
  // }
  using State = Eigen::Vector4d;
  using Control = Eigen::Vector<double, 1>;
  using Gain = Eigen::RowVector<double, 4>;
  std::size_t i{ 0 };
  Control u;
  State tau;
  bool goal_reached, timeout;
  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  for (std::size_t i = 0; i < plan.size() - 1; ++i)
  {
    const std::size_t parent_id{ i };
    const std::size_t target_id{ i + 1 };
    const gtsam::Key key_xt0{ SF::create_hashed_symbol("X^{", parent_id, "}") };
    const gtsam::Key key_xt1{ SF::create_hashed_symbol("X^{", target_id, "}") };
    State xt{ values.at<State>(key_xt0) };
    ss->copy_from(xt);
    // PRX_DBG_VARS(key_xt0, key_xt1);

    const State xgoal{ values.at<State>(key_xt1) };
    tau = difference(xt, xgoal);
    goal_reached = timeout = false;
    const Gain K{ ks[key_xt1] };
    double ti{ 0.0 };
    if (not grid(xt)->element().isZero())
    {
      return true;
    }

    do
    {
      u = -K * tau;
      cs->copy_from(u);
      sg->propagate_once(nullptr);
      ss->enforce_bounds();
      ss->copy_to(xt);
      ti += prx::simulation_step;

      tau = difference(xt, xgoal);
      goal_reached = tau.norm() < 1e-3;
      timeout = ti > 2.50;
      // PRX_DBG_VARS(ti, goal_reached, u, K, tau.transpose());
      // PRX_DBG_VARS(xt.transpose(), xgoal.transpose());
    } while (not goal_reached and not timeout);

    if (not goal_reached)
      return false;
  }
  return true;
}
}  // namespace double_pendulum