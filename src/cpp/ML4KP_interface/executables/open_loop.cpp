#include <fstream>
#include <queue>
#include <set>

#include <prx/utilities/defs.hpp>
#include <prx/utilities/general/param_loader.hpp>

#include <prx/simulation/plants/plants.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>

#include <prx/planning/world_model.hpp>
#include <prx/planning/planners/aorrt.hpp>
#include <prx/planning/planners/planner.hpp>

#include <prx/visualization/three_js_group.hpp>
#include <prx/utilities/data_structures/regular_grid.hpp>

#include "ML4KP_interface/simulation/acrobot.hpp"
#include "ML4KP_interface/simulation/acrobot_pid.hpp"
#include "ML4KP_interface/simulation/pendubot.hpp"
#include "ML4KP_interface/simulation/utils.hpp"
#include "ML4KP_interface/simulation/fg_lqr_traj.hpp"

// using namespace prx;
using prx::utilities::convert_to;
using State = Eigen::Vector4d;
using Control = Eigen::Vector<double, 1>;
using Gain = Eigen::RowVector4d;
using Element = Eigen::RowVector<double, 8>;
using Cell = prx::utilities::cube_cell_t<Element, 4>;
using CellPtr = std::shared_ptr<Cell>;
using Grid = prx::utilities::regular_grid_t<Cell, 4>;
using ReachedFunc = std::function<bool(State&)>;

double dist_to_goal(prx::space_point_t xF)
{
  static const Eigen::Vector4d ref(prx::constants::pi, 0.0, 0.0, 0.0);
  Eigen::Vector4d diff{ Vec(xF) - ref };
  diff[0] = std::atan2(std::sin(diff[0]), std::cos(diff[0]));
  diff[1] = std::atan2(std::sin(diff[1]), std::cos(diff[1]));
  // const double dv0{ x[2] - ref[2] };
  // const double dv1{ x[3] - ref[3] };
  return diff.norm();
}

static std::string timestamp()
{
  auto t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  std::stringstream strstr{};
  strstr << std::put_time(&tm, "%y%m%d_%H%M%S");
  return strstr.str();
}

template <typename Pt0, typename Pt1>
bool goal_reached(const Pt0 pt0, const Pt1 pt1)
{
  // static const Eigen::Vector4d ref(prx::constants::pi, 0.0, 0.0, 0.0);
  Eigen::Vector4d diff{ pt1 - pt0 };
  diff[0] = std::atan2(std::sin(diff[0]), std::cos(diff[0]));
  diff[1] = std::atan2(std::sin(diff[1]), std::cos(diff[1]));
  // const double dv0{ x[2] - ref[2] };
  // const double dv1{ x[3] - ref[3] };
  return diff.norm() < 1e-4;
}

bool check_lqr(State x0, prx::space_t* ss, std::shared_ptr<prx::system_group_t> sg, double_pendulum::LQRptr lqr,
               const State x_desired)
{
  Control u_lqr;
  double t{ 0.0 };
  ss->copy_from(x0);
  // std::size_t idx_curr{ grid.index(x0) };
  // const std::size_t idx_desired{ grid.index(x_desired) };
  // PRX_DBG_VARS(x0.transpose());
  bool lqr_succeded{ false };
  do
  {
    lqr->compute_controls();
    sg->propagate_once(nullptr);
    ss->enforce_bounds();
    ss->copy_to(x0);
    // sg->get_control_space()->copy_to(u_lqr);
    t += prx::simulation_step;

    // PRX_DBG_VARS(t, x0.transpose(), u_lqr);
    // idx_curr = grid.index(x0);
    lqr_succeded = goal_reached(x0, x_desired);
  } while (not lqr_succeded and t < 2.50);
  // } while (t < 5.0 and visited.count(idx_curr) == 0);
  // idx_curr = grid.index(x0);
  // PRX_DBG_VARS(t, idx_curr, idx_desired, x0.transpose());
  // PRX_DBG_VARS(idx_curr, K, current.transpose());
  // for (auto vi : grid(x0)->vertices())
  // {
  // PRX_DBG_VARS(lqr_succeded, x_desired.transpose(), x0.transpose());
  return lqr_succeded;
  // }
  // return idx_curr == idx_desired;
}

int main(int argc, char* argv[])
{
  // prx::param_loader params{};
  prx::param_loader params{ argc, argv };

  std::string plant_in{};
  std::string plant_name{ params["plant"].as<>() };

  // LQRptr lqr;
  if (not params.exists("plant"))
  {
    prx_throw("Need plant param: {acrobot, pendubot} ");
  }
  // if (params["plant"].as<>() == "acrobot")
  // {
  //   plant_name = "acrobot_dp";
  // }
  // else if (params["plant"].as<>() == "pendubot")
  // {
  //   plant_name = "pendubot_dp";
  // }
  // else
  // {
  //   prx_throw("Plant " << params["plant"].as<>() << " not supported.");
  // }
  params.add_opts(argc, argv);

  // params.add_filem(plant_in);

  params.print();
  prx::simulation_step = 0.002;

  // const std::string plant_name{ params["/plant/name"].as<>() };

  prx::system_ptr_t system_aux{ prx::system_factory_t::create_system(plant_name, plant_name) };
  std::shared_ptr<prx::plant_t> plant = std::dynamic_pointer_cast<prx::plant_t>(system_aux);
  prx_assert(plant != nullptr, "Plant is nullptr!");
  // plant->init(params["/plant"]);

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };

  prx::trajectory_t traj(ss);
  prx::plan_t plan(cs);

  plan.from_file(params["plan"].as<>());

  // const bool traj_match{ double_pendulum::check_traj_plan(traj, plan, sg) };
  // prx_assert(traj_match, "Traj / plan do not match");

  prx::space_point_t x0(ss->make_point());
  ss->copy(x0, params["x0"].as<std::vector<double>>());
  sg->propagate(x0, plan, traj);

  PRX_DBG_VARS(traj.back());
  plan.to_file(params["plan"].as<>());
  traj.to_file(params["traj"].as<>());

  if (params.exists("lqr") and params["lqr"].as<bool>())
  {
    double_pendulum::lqr_query_t lqr_query;
    lqr_query.Q = Eigen::DiagonalMatrix<double, 4>(10.0, 10.0, 1.0, 1.0);
    prx::trajectory_t lqr_final_traj(ss);
    double_pendulum::LQRptr lqr{ double_pendulum::create_lqr(plant, lqr_query) };
    prx::condition_check_t lqr_cond_check("sim_time", 5.0);  // 5 secs
    sg->propagate(traj.back(), lqr, lqr_cond_check, lqr_final_traj);
    traj += lqr_final_traj;
  }

  // const std::string body_name{ plant_name + "/" + params["/plant/vis_body"].as<>() };
  const std::string body_name{ plant_name + "/ball" };

  prx::three_js_group_t* vis_group{ new prx::three_js_group_t({ plant }, {}) };
  vis_group->add_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  // vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, traj.front());
  vis_group->output_html(plant_name + "_open_loop.html");

  return 0;
}
