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

  Grid lqr_grid(min, max, cell_length);

  double_pendulum::lqr_query_t lqr_query;

  std::set<std::size_t> visited;

  // Current, local goal
  using LqrTuple = std::tuple<State, State, Control>;
  std::queue<LqrTuple> queue;

  State goal(prx::constants::pi, 0.0, 0.0, 0.0);
  prx::space_point_t x0{ ss->make_point(min) };
  double_pendulum::LQRptr lqr_goal{ double_pendulum::create_lqr(plant, lqr_query) };

  const Gain K_goal{ lqr_goal->lqr().K() };
  PRX_DBG_VARS(K_goal);
  int tot_success{ 0 };
  Eigen::Vector2d rod1, ball;
  do
  {
    // traj.clear();
    // cond_check.reset();

    ss->copy_from(x0);
    plant->update_configuration();
    rod1 = plant->configuration("rod1").translation().head(2);
    ball = plant->configuration("ball").translation().head(2);
    //                                     // no-lint
    // if (ball[1] > 0.45 and lqr_grid.in_bounds(Vec(x0)))  // no-lint
    if (lqr_grid.in_bounds(Vec(x0)))  // no-lint
    {
      // ofs_states << print_pt(x0) << " ";
      // ofs_states << rod1.transpose() << " " << ball.transpose() << " ";
      const bool success{ check_lqr(Vec(x0), ss, sg, lqr_goal, goal) };
      if (success)
      {
        tot_success++;
        // PRX_DBG_VARS(tot_success, x0);
        if (tot_success % 1000 == 0)
          PRX_DBG_VARS(tot_success, x0);

        const Element K{ (Element() << K_goal, goal.transpose()).finished() };
        lqr_grid(Vec(x0))->element() = K;
      }
      // sg->propagate(pt, lqr, cond_check, traj);
    }
    // ofs_trajs << traj;
  } while (x0->step(cell_length, min, max));

  auto valid_cell_func = [](CellPtr cell) { return not cell->element().isZero(); };
  lqr_grid.to_file("/Users/Gary/pracsys/double_pendulum/src/cpp/out/lqr_grid.txt", valid_cell_func);
}
