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
// using namespace prx;

int main(int argc, char* argv[])
{
  prx::param_loader params{ argc, argv };

  if (params.exists("planner_file"))
  {
    params.add_file(params["file"].as<>());
  }
  else
  {
    prx_throw("No planner file set");
  }
  if (params.exists("plant_file"))
  {
    params.add_file(params["plant_file"].as<>());
  }
  else
  {
    prx_throw("No plant file set");
  }
  params.print();
  prx::simulation_step = params["/planner/simulation_step"].as<double>();
  prx::init_random(params["/planner/random_seed"].as<int>());

  const std::string plant_name{ params["/plant/name"].as<>() };

  prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_name) };
  prx_assert(plant != nullptr, "Plant is nullptr!");
  plant->init(params["/plant"]);

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::aorrt_t aorrt("AORRT");
  prx::aorrt_specification_t aorrt_spec(sg, cg);
  aorrt_spec.init(params["planner"]);
  // rrt_spec.valid_state = [](space_point_t& s)
  // {
  // Custom valid_state can be added here.
  // };

  // rrt_spec.valid_check = [&rrt_spec](trajectory_t& traj)
  // {
  // Custom valid_check goes here...
  // Basically for x in traj, call valid_state
  // };

  // rrt_spec.sample_plan = [&](plan_t& plan, space_point_t pose)
  // {
  // Add custom sample plan here
  // };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };

  prx::aorrt_query_t aorrt_query(ss, cs);
  aorrt_query.start_state = ss->make_point();
  aorrt_query.goal_state = ss->make_point();

  // TODO: Include more complex goal region
  // aorrt_query.goal_region_radius = params["/plant/goal/radius"].as<double>();

  // Alternatively, change the goal_check function
  // rrt_query.goal_check = [&](space_point_t pt)
  // {
  //    // Default is:
  // return space_t::euclidean_2d(pt, rrt_query.goal_state) < goal_region_radius;
  // }

  aorrt.link_and_setup_spec(&aorrt_spec);
  aorrt.preprocess();
  aorrt.link_and_setup_query(&aorrt_query);

  prx::condition_check_t checker(params["/planner/checker"]);

  aorrt.resolve_query(&checker);
  aorrt.fulfill_query();

  params.print();

  prx::three_js_group_t* vis_group{ new prx::three_js_group_t({ plant }, {}) };

  const std::string body_name{ plant_name + "/" + params["/plant/vis_body"].as<>() };

  vis_group->add_vis_infos(prx::info_geometry_t::LINE, aorrt_query.tree_visualization, body_name, ss);
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, aorrt_query.solution_traj, body_name, ss);
  vis_group->add_animation(aorrt_query.solution_traj, ss, aorrt_query.start_state);
  vis_group->output_html("double_pendulum.html");

  delete vis_group;
}
