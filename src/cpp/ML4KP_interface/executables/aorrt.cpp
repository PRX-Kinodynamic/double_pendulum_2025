#include <fstream>
#include <prx/utilities/defs.hpp>
#include <prx/utilities/general/param_loader.hpp>

#include <prx/simulation/plants/plants.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>

#include <prx/planning/world_model.hpp>
#include <prx/planning/planners/aorrt.hpp>
#include <prx/planning/planners/planner.hpp>

#include <prx/visualization/three_js_group.hpp>

#include "ML4KP_interface/simulation/acrobot.hpp"
#include "ML4KP_interface/simulation/utils.hpp"
// using namespace prx;
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

int main(int argc, char* argv[])
{
  // prx::param_loader params{};
  prx::param_loader params{ argc, argv };

  std::string plant_in{};
  std::string planner_in{};
  if (params.exists("planner_file"))
  {
    planner_in = params["planner_file"].as<>();
    params["planner"].add_file(planner_in);
    // params.add_file(planner_in);
  }
  else  ///
  {
    prx_throw("No planner file set");
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
  prx::init_random(params["/planner/random_seed"].as<int>());

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

  prx::aorrt_t aorrt("AORRT");
  prx::aorrt_specification_t aorrt_spec(sg, cg);
  aorrt_spec.init(params["planner"]);
  aorrt_spec.valid_state = [](prx::space_point_t& s) { return true; };

  aorrt_spec.valid_check = [](prx::trajectory_t& traj) { return true; };
  // aorrt_spec.sample_state = [this](prx::space_point_t& s) {
  //   s->()
  // };

  // rrt_spec.sample_plan = [&](plan_t& plan, space_point_t pose)
  // {
  // Add custom sample plan here
  // };
  aorrt_spec.distance_function = [](const prx::space_point_t& s1, const prx::space_point_t& s2) {
    // static const Eigen::Vector4d ref(prx::constants::pi, 0.0, 0.0, 0.0);
    Eigen::Vector4d diff{ Vec(s1) - Vec(s2) };
    diff[0] = std::atan2(std::sin(diff[0]), std::cos(diff[0]));
    diff[1] = std::atan2(std::sin(diff[1]), std::cos(diff[1]));
    return diff.norm();
  };
  aorrt_spec.cost_function = [](const prx::trajectory_t& t, const prx::plan_t& plan) {
    const double s_time{ plan.duration() };
    return s_time;
    // const double energy{ plan.duration() };
  };

  prx::aorrt_query_t aorrt_query(ss, cs);
  aorrt_query.start_state = ss->make_point();
  aorrt_query.goal_state = ss->make_point();
  ss->copy(aorrt_query.start_state, params["start_state"].as<std::vector<double>>());
  ss->copy(aorrt_query.goal_state, { prx::constants::pi, 0.0, 0.0, 0.0 });
  // TODO: Include more complex goal region
  // aorrt_query.goal_region_radius = params["/plant/goal/radius"].as<double>();

  // Alternatively, change the goal_check function
  aorrt_query.goal_check = [&](prx::space_point_t pt) {
    ss->copy_from(pt);
    plant->update_configuration();
    const double y{ plant->configuration("ball").translation()[1] };
    // PRX_DBG_VARS(y, pt);
    return y > 0.45 and Vec(pt).tail(2).norm() < 5.0;
  };

  aorrt.link_and_setup_spec(&aorrt_spec);
  aorrt.preprocess();
  aorrt.link_and_setup_query(&aorrt_query);

  prx::condition_check_t checker(params["/planner/checker"]);

  aorrt.resolve_query(&checker);
  aorrt.fulfill_query();

  PRX_DBG_VARS(aorrt.get_statistics());
  // params.print();

  prx::trajectory_t final_traj(ss);
  double_pendulum::LQRptr lqr{ double_pendulum::create_lqr(plant) };

  prx::condition_check_t cond_check("sim_time", 5.0);  // 5 secs

  sg->propagate(aorrt_query.solution_traj.back(), lqr, cond_check, final_traj);

  if (dist_to_goal(final_traj.back()) < 0.01)
  {
    const std::string MSG{ "Goal reached LQR ROA" };
    PRX_DBG_VARS(MSG);

    const std::string out_dir{ params["out_dir"].as<>() };
    const std::string ts{ timestamp() };
    aorrt_query.solution_traj.to_file(out_dir + "/traj_" + ts + ".txt");
    aorrt_query.solution_plan.to_file(out_dir + "/plan_" + ts + ".txt");
  }
  else
  {
    const std::string MSG{ "Goal NOT reached LQR ROA" };
    PRX_DBG_VARS(MSG);
  }
  prx::three_js_group_t* vis_group{ new prx::three_js_group_t({ plant }, {}) };

  if (params["/planner/visualize"].as<bool>())
  {
    prx::trajectory_t viz_traj(ss);
    viz_traj += aorrt_query.solution_traj;
    viz_traj += final_traj;
    const std::string body_name{ plant_name + "/" + params["/plant/vis_body"].as<>() };

    vis_group->add_vis_infos(prx::info_geometry_t::LINE, aorrt_query.tree_visualization, body_name, ss);
    vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, viz_traj, body_name, ss);
    vis_group->add_animation(viz_traj, ss, aorrt_query.start_state);
    vis_group->output_html("double_pendulum_aorrt.html");
  }
  delete vis_group;
}
