#include <fstream>
#include <prx/utilities/defs.hpp>
#include <prx/utilities/general/param_loader.hpp>

#include <prx/simulation/plants/plants.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>

#include <prx/planning/world_model.hpp>
#include <prx/planning/planners/aorrt.hpp>
#include <prx/planning/planners/planner.hpp>

#include <prx/visualization/three_js_group.hpp>
#include <prx/simulation/playback/tree_to_trajs_and_plans.hpp>

#include "ML4KP_interface/simulation/acrobot.hpp"
#include "ML4KP_interface/simulation/acrobot_pid.hpp"
#include "ML4KP_interface/simulation/pendubot.hpp"
#include "ML4KP_interface/simulation/utils.hpp"
#include "ML4KP_interface/simulation/acrobot_u2.hpp"
// using namespace prx;
using prx::utilities::convert_to;

// double dist_to_goal(prx::space_point_t xF)
// {
//   static const Eigen::Vector4d ref(prx::constants::pi, 0.0, 0.0, 0.0);
//   Eigen::Vector4d diff{ Vec(xF).head(4) - ref };
//   diff[0] = std::atan2(std::sin(diff[0]), std::cos(diff[0]));
//   diff[1] = std::atan2(std::sin(diff[1]), std::cos(diff[1]));
//   // const double dv0{ x[2] - ref[2] };
//   // const double dv1{ x[3] - ref[3] };
//   return diff.norm();
// }

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

  params["epsilon"].set(5.0);
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
  const int seed{ params["/planner/random_seed"].as<int>() };
  prx::init_random(seed);

  const std::string plant_name{ params["/plant/name"].as<>() };
  // std::string plant_name_lqr{ plant_name == "acrobot_u2" ? "acrobot_lqr_u2" : "pendubot_lqr_u2" };

  prx::system_ptr_t system_aux{ prx::system_factory_t::create_system(plant_name, plant_name) };
  // prx::system_ptr_t system_aux_lqr{ prx::system_factory_t::create_system(plant_name_lqr, plant_name_lqr) };
  std::shared_ptr<prx::plant_t> plant{ std::dynamic_pointer_cast<prx::plant_t>(system_aux) };
  // std::shared_ptr<prx::plant_t> plant_lqr{ std::dynamic_pointer_cast<prx::plant_t>(system_aux_lqr) };
  prx_assert(plant != nullptr, "Plant is nullptr!");
  plant->init(params["/plant"]);

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  prx::space_t* ps{ sg->get_parameter_space() };

  prx::aorrt_t aorrt("AORRT");
  prx::aorrt_specification_t aorrt_spec(sg, cg);
  aorrt_spec.init(params["planner"]);

  const double angle_limit{ 720.0 * prx::constants::pi / 180.0 };
  const double angle_vel_limit{ 20 };
  const Eigen::Array<double, 4, 1> limits(angle_limit, angle_limit, angle_vel_limit, angle_vel_limit);
  PRX_DBG_VARS(limits);
  aorrt_spec.valid_state = [&](prx::space_point_t& s) {
    // const bool valid_th0{ -angle_limit < s->at(0) and s->at(0) < angle_limit };
    // const bool valid_th1{ -angle_limit < s->at(1) and s->at(1) < angle_limit };
    // const bool valid_dth0{ -angle_vel_limit < s->at(2) and s->at(2) < angle_vel_limit };
    // const bool valid_dth1{ -angle_vel_limit < s->at(3) and s->at(3) < angle_vel_limit };
    const bool valid{ (Vec(s).cwiseAbs().head(4).array() < limits).all() };

    return valid;
    // return valid_th0 and valid_th1 and valid_dth0 and valid_dth1;
  };

  // aorrt_spec.valid_check = [](prx::trajectory_t& traj) { return true; };
  // aorrt_spec.sample_state = [this](prx::space_point_t& s) {
  //   s->()
  // };

  aorrt_spec.sample_plan = [&](prx::plan_t& plan, prx::space_point_t pose) {
    plan.clear();
    double multiplier = prx::simulation_step >= 1 ? prx::simulation_step : 1. / prx::simulation_step;
    const double s{ prx::uniform_random(0.0, 1.0) };
    plan.append_onto_back(prx::uniform_int_random(aorrt_spec.min_control_steps, aorrt_spec.max_control_steps) /
                          multiplier);
    cs->sample(plan.back().control);
    if (s > 0.5)
    {
      if (pose->at(3) > 15)
      {
        plan.back().control->at(0) = -6;
      }
      else if (pose->at(3) < -15)
      {
        plan.back().control->at(0) = +6;
      }
    }
  };

  aorrt_spec.distance_function = [](const prx::space_point_t& s1, const prx::space_point_t& s2) {
    // static const Eigen::Vector4d ref(prx::constants::pi, 0.0, 0.0, 0.0);
    Eigen::Vector4d diff{ Vec(s1).head(4) - Vec(s2).head(4) };
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
  aorrt_query.init(params["planner"]);

  aorrt_query.start_state = ss->make_point();
  aorrt_query.goal_state = ss->make_point();
  ss->copy(aorrt_query.start_state, params["start_state"].as<std::vector<double>>());
  ss->copy(aorrt_query.goal_state, params["goal_state"].as<std::vector<double>>());
  // TODO: Include more complex goal region
  // aorrt_query.goal_region_radius = params["/plant/goal/radius"].as<double>();

  prx::condition_check_t lqr_cond_check("sim_time", 5.0);  // 5 secs
  prx::space_point_t lqr_final_pt{ ss->make_point() };

  ps->copy_from({ -1.0 });
  double_pendulum::lqr_query_t<8, 2> lqr_query;
  // PRX_DBG_VARS(lqr_query.Q);
  lqr_query.Q(0, 0) = lqr_query.Q(1, 1) = 10;
  lqr_query.Q(2, 2) = lqr_query.Q(3, 3) = 1.0;
  lqr_query.Q(4, 4) = lqr_query.Q(5, 5) = 0.0;
  lqr_query.Q(6, 6) = lqr_query.Q(7, 7) = 0.0;
  lqr_query.R(0, 0) = lqr_query.R(1, 1) = 1;

  // LQRptr lqr{ double_pendulum::create_lqr(plant_lqr, lqr_query) };
  // PRX_DBG_VARS(lqr_query.Q);
  Eigen::Matrix<double, 2, 8> lqrK;
  // Acrobot
  lqrK << 8.84197, 2.20974, 0.140444, 0.051115, 0.0, 0.0, 0.0, 0.0,  // no-lint
      2.94643, 5.37887, 0.0728898, 0.0792394, 0.0, 0.0, 0.0, 0.0;
  // ss->copy(lqrK, params["/plant/lqr/K"].as<std::vector<double>>());
  double_pendulum::LQRptr lqr{ std::make_shared<double_pendulum::LQR>(plant, plant_name, lqrK,
                                                                      Vec(aorrt_query.goal_state), lqr_query.diff) };
  ps->copy_from({ 1.0 });
  const double epsilon{ params["epsilon"].as<double>() };

  // const Eigen::Array<double, 4, 1> goal_cond(0.25, 0.25, 5.0, 5.0);
  const Eigen::Array<double, 4, 1> goal_cond(0.5, 0.5, 10.0, 10.0);
  // Alternatively, change the goal_check function
  aorrt_query.goal_check = [&](prx::space_point_t pt) {
    // ss->copy_from(pt);
    // plant->update_configuration();
    // const double y{ plant->configuration("ball").translation()[1] };
    // const bool goal_region_reached{ y > 0.45 };
    Eigen::Vector4d diff{ Vec(pt).head(4) - Vec(aorrt_query.goal_state).head(4) };
    diff[0] = std::atan2(std::sin(diff[0]), std::cos(diff[0]));
    diff[1] = std::atan2(std::sin(diff[1]), std::cos(diff[1]));
    const bool goal_reached{ (diff.cwiseAbs().array() < goal_cond).all() };
    // PRX_DBG_VARS(pt, goal_reached);
    // return goal_reached;
    // ((boxMin.array()<p0.array()).all() && (boxMax.array()>p0.array()).all())
    // const bool close_th0{ std::abs(diff[0]) < 0.25 };
    // const bool close_th1{ std::abs(diff[1]) < 0.25 };
    // const bool close_dth0{ std::abs(diff[2]) < 5.0 };
    // const bool close_dth1{ std::abs(diff[3]) < 5.0 };
    // return close_th0 and close_th1 and close_dth0 and close_dth1;
    if (goal_reached)
    {
      ps->copy_from({ -1.0 });
      lqr_cond_check.reset();
      sg->propagate(pt, lqr, lqr_cond_check, lqr_final_pt);
      const double final_dist{ lqr_query.diff(Vec(lqr_final_pt), lqr_query.x_goal).norm() };
      // const double final_dist{ dist_to_goal(lqr_final_pt) };
      PRX_DBG_VARS(pt, lqr_final_pt, final_dist);
      ps->copy_from({ 1.0 });
      // PRX_DBG_VARS(pt, lqr_final_pt, final_dist);
      return final_dist < 0.01;
    }
    // else
    // {
    return false;
    // }
  };

  aorrt.link_and_setup_spec(&aorrt_spec);
  aorrt.preprocess();
  aorrt.link_and_setup_query(&aorrt_query);

  PRX_DBG_VARS(aorrt_spec);
  PRX_DBG_VARS(aorrt_query);

  prx::condition_check_t checker(params["/planner/checker"]);

  // aorrt.resolve_query(&checker);

  std::vector<std::vector<double>> all_stats;
  const int repeats{ params["/planner/checker/repeats"].as<int>() };

  const double accepted_cost{ params["/planner/min_cost"].as<double>() };
  int sln_repeats{ 0 };
  double curr_cost{ std::numeric_limits<double>::infinity() };
  // for (int i = 0; i < repeats; i++)
  while (sln_repeats < repeats and curr_cost > accepted_cost)
  {
    checker.reset();
    aorrt.resolve_query(&checker);
    const std::vector<double> stats{ aorrt.get_statistics() };
    all_stats.push_back(stats);
    curr_cost = stats[3];
    if (std::isfinite(curr_cost))
    {
      // PRX_DBG_VARS(curr_cost);
      sln_repeats++;
    }
    PRX_DBG_VARS(stats);
  }
  // PRX_DBG_VARS(sln_repeats, curr_cost);

  aorrt.fulfill_query();

  // PRX_DBG_VARS(aorrt.get_statistics());
  // params.print();

  prx::trajectory_t final_traj(ss);

  // prx::condition_check_t lqr_cond_check("sim_time", 5.0);  // 5 secs

  if (aorrt_query.solution_traj.size() > 0)
  {
    std::shared_ptr<prx::tree_t> tree_slns{ aorrt.tree_of_solutions() };
    std::vector<std::pair<prx::trajectory_t, prx::plan_t>> total_solutions{
      prx::simulation::tree_to_trajs_and_plans<prx::aorrt_node_t, prx::aorrt_edge_t>(tree_slns, aorrt.root_index())
    };

    // lqr_cond_check.reset();
    // sg->propagate(aorrt_query.solution_traj.back(), lqr, lqr_cond_check, final_traj);
    // const double final_dist{ dist_to_goal(final_traj.back()) };
    // PRX_DBG_VARS(aorrt_query.solution_traj.back(), final_traj.back(), final_dist);
    // if (final_dist < 0.01)
    // {
    // const std::string MSG{ "Goal reached LQR ROA" };
    const std::string MSG{ "Trajectories found" };
    PRX_DBG_VARS(MSG);
    PRX_DBG_VARS(total_solutions.size());

    const std::string out_dir{ params["out_dir"].as<>() };
    const std::string ts{ timestamp() + "_" + convert_to<std::string>(seed) };
    for (int i = 0; i < total_solutions.size(); ++i)
    {
      const std::string prefix{ ts + "_" + convert_to<std::string>(i) };
      total_solutions[i].first.to_file(out_dir + "/traj_" + prefix + ".txt");
      total_solutions[i].second.to_file(out_dir + "/plan_" + prefix + ".txt");
    }
    // aorrt_query.solution_traj.to_file(out_dir + "/traj_" + ts + ".txt");
    // aorrt_query.solution_plan.to_file(out_dir + "/plan_" + ts + ".txt");
    std::ofstream ofs(out_dir + "/stats_" + ts + ".txt");
    for (auto stats : all_stats)
    {
      for (auto s : stats)
      {
        ofs << s << " ";
      }
      ofs << "\n";
    }
    ofs.close();
    // }
    // else
    // {
    //   const std::string MSG{ "Goal NOT reached LQR ROA" };
    //   PRX_DBG_VARS(MSG);
    // }
  }
  else
  {
    const std::string MSG{ "No solution found" };
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
    vis_group->output_html(plant_name + "_aorrt.html");
  }
  delete vis_group;
}
