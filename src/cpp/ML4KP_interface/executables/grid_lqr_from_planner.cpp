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

  if (params.exists("grid_file"))
  {
    const std::string grid_in{ params["grid_file"].as<>() };
    params["grid"].add_file(grid_in);
  }
  else
  {
    prx_throw("No grid file set");
  }

  if (params.exists("plant_file"))
  {
    plant_in = params["plant_file"].as<>();
    params["plant"].add_file(plant_in);
  }
  else
  {
    prx_throw("No plant file set");
  }
  params.add_opts(argc, argv);

  // params.add_filem(plant_in);

  params.print();
  prx::simulation_step = 0.002;

  const std::string plant_name{ params["/plant/name"].as<>() };

  prx::system_ptr_t system_aux{ prx::system_factory_t::create_system(plant_name, plant_name) };
  std::shared_ptr<prx::plant_t> plant = std::dynamic_pointer_cast<prx::plant_t>(system_aux);
  prx_assert(plant != nullptr, "Plant is nullptr!");
  plant->init(params["/plant"]);

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };

  // Goal: pi,0,0,0
  Cell::Coordinate min(0.0, 0.0, -30.0, -30.0);
  Cell::Coordinate max(2 * prx::constants::pi, 2 * prx::constants::pi, +30.0, +30.0);
  Cell::Coordinate cell_length(0.2, 0.2, 0.1, 0.1);
  ss->copy(min, params["grid/bound/min"].as<std::vector<double>>());
  ss->copy(max, params["grid/bound/max"].as<std::vector<double>>());
  ss->copy(cell_length, params["grid/cell_length"].as<std::vector<double>>());
  // const Cell::Coordinate cell_length(0.1, 0.1, 0.2, 0.2);
  PRX_DBG_VARS(min.transpose(), max.transpose());
  const std::vector<Cell::Coordinate> movements{
    { +cell_length[0], 0.0, 0.0, 0.0 }, { -cell_length[0], 0.0, 0.0, 0.0 },  // no-lint
    { 0.0, +cell_length[1], 0.0, 0.0 }, { 0.0, -cell_length[1], 0.0, 0.0 },  // no-lint
    { 0.0, 0.0, +cell_length[2], 0.0 }, { 0.0, 0.0, -cell_length[2], 0.0 },  // no-lint
    { 0.0, 0.0, 0.0, +cell_length[3] }, { 0.0, 0.0, 0.0, -cell_length[3] }
  };

  PRX_DEBUG_PRINT
  auto line_to_element = [](const std::vector<std::string> line) {
    Element element;
    for (int i = 0; i < 8; ++i)
    {
      element[i] = convert_to<double>(line[i]);
    }
    return element;
  };

  // Grid lqr_grid(params["grid/filename"].as<>(), line_to_element);
  PRX_DEBUG_PRINT

  std::set<std::size_t> visited;

  // Current, local goal

  State goal(prx::constants::pi, 0.0, 0.0, 0.0);
  prx::space_point_t x0{ ss->make_point(min) };

  prx::trajectory_t traj(ss);
  prx::plan_t plan(cs);
  prx::plan_t lqr_plan(cs);

  // const std::string file_id{ "250311_232304_146225553" };
  // const std::string file_id{ "250312_012633_25525950" };
  // const std::string file_id{ "250311_221753_593446886" }; <--
  const std::string file_id{ params["file_id"].as<>() };
  const std::string dir{ "/Users/Gary/pracsys/double_pendulum/src/cpp/out/" };
  traj.from_file(dir + "/traj_" + file_id + ".txt");
  plan.from_file(dir + "/plan_" + file_id + ".txt");
  PRX_DBG_VARS(plan);
  PRX_DBG_VARS(traj.front());

  const bool traj_match{ double_pendulum::check_traj_plan(traj, plan, sg) };
  prx_assert(traj_match, "Traj / plan do not match");

  double_pendulum::lqr_query_t lqr_query;
  lqr_query.Q = Eigen::DiagonalMatrix<double, 4>(10.0, 10.0, 1.0, 1.0);
  prx::trajectory_t lqr_final_traj(ss);
  double_pendulum::LQRptr lqr{ double_pendulum::create_lqr(plant, lqr_query) };
  prx::condition_check_t lqr_cond_check("sim_time", 5.0);  // 5 secs
  lqr_cond_check.reset();
  // sg->propagate(traj.back(), lqr, lqr_cond_check, lqr_final_traj);
  // double_pendulum::lqr_to_plan(traj.back(), lqr, lqr_cond_check, lqr_final_traj, lqr_plan, sg);
  // traj += lqr_final_traj;
  // plan += lqr_plan;
  // PRX_DBG_VARS(plan);

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;
  std::vector<gtsam::Key> Xkeys;
  std::vector<gtsam::Key> Ukeys;
  const int step_size{ 1 };

  double_pendulum::create_fg(graph, values, traj, plan, Xkeys, Ukeys, plant_name, step_size);

  // graph.print("graph", double_pendulum::SF::formatter);
  // values.print("values", double_pendulum::SF::formatter);
  double_pendulum::SF::symbols_to_file(dir + "/factor_graph_symbols.txt");
  double_pendulum::GainMap Ks;
  double_pendulum::CostToGoMap Ss;
  double_pendulum::create_lqr_fg(graph, values, Xkeys, Ukeys, Ks, Ss);

  PRX_DBG_VARS(Ks.size(), Ss.size(), plan.duration());
  std::size_t idx{ 0 };
  State xi(Vec(traj.front()));
  double ti{ 0.0 };

  // xi = xi + Eigen::Vector4d::Random() * 0.01;
  PRX_DBG_VARS(traj.front(), xi.transpose());
  ss->copy_from(xi);
  prx::trajectory_t fg_traj(ss);
  // fg_traj.push_back(xi);
  fg_traj.copy_onto_back(ss);
  std::ofstream ofs(dir + "/" + plant_name + "_" + file_id + "_lqr_traj.txt");
  for (int i = 0; i < traj.size() - 1; ++i)
  {
    if (i % step_size == 0)
    {
      idx = std::min(idx + step_size, traj.size() - 1);
    }
    // PRX_DBG_VARS(idx, ti);

    const gtsam::Key key_xt0{ double_pendulum::SF::create_hashed_symbol("X^{", idx, "}") };
    const State xd{ double_pendulum::difference(Vec(fg_traj.back()), Vec(traj[idx])) };
    const Control du{ -Ks[key_xt0] * xd };
    const Control u{ du + Vec(plan.at(ti)) };

    ofs << traj[idx] << " ";
    ofs << plan.at(ti) << " ";
    ofs << Ks[key_xt0] << "\n";

    cs->copy_from(u);
    cs->enforce_bounds();
    sg->propagate_once();
    // ss->copy_to(xi);
    fg_traj.copy_onto_back(ss);
    // PRX_DBG_VARS(ti, du, u, fg_traj.back());
    ti += prx::simulation_step;
  }
  ofs.close();

  PRX_DBG_VARS(traj.back(), fg_traj.back());

  lqr_final_traj.clear();
  lqr_cond_check.reset();
  sg->propagate(traj.back(), lqr, lqr_cond_check, lqr_final_traj);
  traj += lqr_final_traj;

  lqr_final_traj.clear();
  lqr_cond_check.reset();
  sg->propagate(fg_traj.back(), lqr, lqr_cond_check, lqr_final_traj);
  fg_traj += lqr_final_traj;

  const std::string body_name{ plant_name + "/" + params["/plant/vis_body"].as<>() };

  traj.to_file(dir + "/traj.txt");
  fg_traj.to_file(dir + "/fg_traj.txt");

  prx::three_js_group_t* vis_group{ new prx::three_js_group_t({ plant }, {}) };
  vis_group->add_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  // vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, traj.front());
  vis_group->output_html(plant_name + "_fglqr_traj_expected.html");

  prx::three_js_group_t* vis_group_fg{ new prx::three_js_group_t({ plant }, {}) };
  vis_group_fg->add_vis_infos(prx::info_geometry_t::FULL_LINE, fg_traj, body_name, ss);
  // vis_group_fg->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, fg_traj, body_name, ss);
  vis_group_fg->add_animation(fg_traj, ss, fg_traj.front());
  vis_group_fg->output_html(plant_name + "_fglqr_traj_res.html");
  // auto valid_cell_func = [](CellPtr cell) { return not cell->element().isZero(); };
  // lqr_grid.to_file("/Users/Gary/pracsys/double_pendulum/src/cpp/out/updated_lqr_grid.txt", valid_cell_func);

  return 0;
}
