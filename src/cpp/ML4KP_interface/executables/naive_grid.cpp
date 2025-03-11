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
using Element = Eigen::RowVector<double, 2>;  // Control, counter
using Cell = prx::utilities::cube_cell_t<Element, 4>;
using CellPtr = std::shared_ptr<Cell>;
using Grid = prx::utilities::regular_grid_t<Cell, 4>;
using CellString = prx::utilities::cube_cell_t<std::string, 4>;
using GridString = prx::utilities::regular_grid_t<CellString, 4>;
using LqrGrid = prx::utilities::regular_grid_t<prx::utilities::cube_cell_t<Eigen::RowVector<double, 8>, 4>, 4>;
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

template <typename Grid>
void states_to_grid(const std::string filename, Grid& grid)
{
  using prx::utilities::convert_to;
  using prx::utilities::csv_reader_t;
  using Line = std::vector<std::string>;
  csv_reader_t reader_tree(filename);
  while (reader_tree.has_next_line())
  {
    Line line{ reader_tree.next_line() };

    if (line.size() == 0)
      continue;

    const std::string traj_file{ line[0] };
    const double th0{ convert_to<double>(line[1]) };
    const double th1{ convert_to<double>(line[2]) };
    const double dth0{ convert_to<double>(line[3]) };
    const double dth1{ convert_to<double>(line[4]) };
    grid(th0, th1, dth0, dth1)->element() = traj_file;
  }
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

  auto line_to_element = [](const std::vector<std::string> line) {
    Eigen::Vector<double, 8> element;
    for (int i = 0; i < 8; ++i)
    {
      element[i] = convert_to<double>(line[i]);
    }
    return element;
  };
  LqrGrid lqr_grid(params["grid/filename"].as<>(), line_to_element);
  Grid naive_grid(min, max, cell_length);

  const std::string init_str("");
  GridString files_grid(min, max, cell_length, init_str);
  const std::string dir{ params["dir"].as<>() };
  states_to_grid(params["states_filename"].as<>(), files_grid);

  prx::plan_t plan(cs);
  prx::trajectory_t traj(ss);

  for (auto cell_ptr : lqr_grid)
  {
    const State xi{ cell_ptr->vertex(0) + cell_length / 2.0 };
    if (not lqr_grid(xi)->element().isZero())
    {
      naive_grid(xi)->element() += (Element() << -1, -1).finished();
    }
  }
  for (auto cell_ptr : files_grid)
  {
    const std::string trajname{ cell_ptr->element() };
    if (trajname != "")
    {
      traj.clear();
      plan.clear();
      const std::string traj_filename(dir + "/" + trajname);
      const std::string plan_filename(dir + "/" + trajname.substr(4));
      traj.from_file(traj_filename);
      plan.from_file(plan_filename);
      const double duration{ plan.duration() };
      const double dur_so_far{ 0.0 };
      for (auto step : plan)
      {
        const double dur_i{ step.duration };
        const Control ui{ Vec(step.control) };
        const double prop{ dur_i / duration };
        for (double ti = dur_so_far; ti < prop; ti += 0.01)
        {
          const State xi{ Vec(traj.at(ti, true)) };
          if (lqr_grid(xi)->element().isZero())
          {
            naive_grid(xi)->element() += (Element() << ui, 1).finished();
          }
        }
      }
    }
  }

  auto valid_cell_func = [](CellPtr cell) { return not cell->element().isZero(); };
  naive_grid.to_file("/Users/Gary/pracsys/double_pendulum/src/cpp/out/naive_grid.txt", valid_cell_func);
  auto invalid_cell_func = [](CellPtr cell) { return cell->element().isZero(); };
  naive_grid.to_file("/Users/Gary/pracsys/double_pendulum/src/cpp/out/naive_grid_empty.txt", invalid_cell_func);

  return 0;
}
