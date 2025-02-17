#pragma once

#include <prx/simulation/plant.hpp>
#include "model/src/dp_plant.hpp"
#include "simulator/src/simulator.hpp"

namespace double_pendulum
{

class acrobot_t : public prx::plant_t
{
  using State = Eigen::Vector4d;
  using StateDDot = Eigen::Vector2d;
  using Control = Eigen::Vector2d;  // 2d for compatibility on dp_plant but only exposing 1d

public:
  acrobot_t(const std::string& path)
    : prx::plant_t(path)
    , _x(State::Zero())
    , _u(Control::Zero())
    , _integrator("runge_kutta")
    , _dt(0.002)
    , _plant(false, true)
    , _sim()
    , _length_1(0.2)
    , _length_2(0.3)
    , _box_w(0.05)
  {
    set_params();
    _sim.set_plant(_plant);

    state_memory = { &_x[0], &_x[1],  // no-lint
                     &_x[2], &_x[3] };
    state_space = new prx::space_t("RREE", state_memory, "acrobot_state");
    // state_space->set_bounds({ -9.42, -9.42, -30, -30 },  // 540 deg~= 9.42 rad
    //                         { +9.42, +9.42, +30, +30 });
    state_space->set_bounds({ -1e50, -1e50, -1e50, -1e50 },  // 540 deg~= 9.42 rad
                            { +1e50, +1e50, +1e50, +1e50 });

    control_memory = { &_u[1] };
    input_control_space = new prx::space_t("E", control_memory, "acrobot_ctrl");
    input_control_space->set_bounds({ -6 }, { 6 });

    // Derivative memory not used.
    derivative_memory = { &_x[2], &_x[3], &_xdd[0], &_xdd[1] };
    derivative_space = new prx::space_t("EEEE", derivative_memory, "acrobot_dd");

    parameter_memory = {};
    // const std::string param_topology{ std::string(parameter_memory.size(), 'E') };
    parameter_space = new prx::space_t("", parameter_memory, "acrobot_params");

    geometries["rod1"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX);
    geometries["rod1"]->initialize_geometry({ _length_1, _box_w, _box_w });
    geometries["rod1"]->generate_collision_geometry();
    geometries["rod1"]->set_visualization_color("0x00ff00");
    configurations["rod1"] = std::make_shared<prx::transform_t>();
    configurations["rod1"]->setIdentity();

    geometries["rod2"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX);
    geometries["rod2"]->initialize_geometry({ _length_2, _box_w, _box_w });
    geometries["rod2"]->generate_collision_geometry();
    geometries["rod2"]->set_visualization_color("0xff0000");
    configurations["rod2"] = std::make_shared<prx::transform_t>();
    configurations["rod2"]->setIdentity();

    geometries["ball"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::SPHERE);
    geometries["ball"]->initialize_geometry({ _box_w });
    geometries["ball"]->generate_collision_geometry();
    geometries["ball"]->set_visualization_color("0x0000ff");
    configurations["ball"] = std::make_shared<prx::transform_t>();
    configurations["ball"]->setIdentity();
  }

  virtual ~acrobot_t()
  {
  }

  void set_params()
  {
    // Params from:
    // https://dfki-ric-underactuated-lab.github.io/real_ai_gym_leaderboard/acrobot_simulation_performance_leaderboard_v2.html
    const double gravity{ 9.81 };
    const double m1{ 0.5234602302310271 };
    const double m2{ 0.6255677234174437 };
    // const double l1{ 0.2 };
    // const double l2{ 0.3 };

    const double I1{ 0.031887199591513114 };
    const double I2{ 0.05086984812807257 };

    const double cm1{ 0.2 };
    const double cm2{ 0.25569305436052964 };

    const double d1{ 0.0 };
    const double d2{ 0.0 };

    const double cf1{ 0.0 };
    const double cf2{ 0.0 };

    const double tl1{ 0.0 };
    const double tl2{ 6.0 };

    _plant.set_parameters(m1, m2,                // no-lint
                          _length_1, _length_2,  // no-lint
                          cm1, cm2,              // no-lint
                          I1, I2,                // no-lint
                          d1, d2,                // no-lint
                          cf1, cf2,              // no-lint
                          gravity,               // no-lint
                          tl1, tl2);
  }

  virtual void propagate(const double simulation_step) override
  {
    // _x[1] -= prx::constants::pi;
    prx_assert(simulation_step > 0, "simulation_step not greater than zero!");
    _sim.set_state(0.0, _x);
    _xdd = _plant.forward_dynamics(_x, _u);
    _sim.step(_u, simulation_step, _integrator);
    _x = _sim.get_state();
    // _x[1] += prx::constants::pi;
    // PRX_DBG_VARS(_x.transpose());
    // PRX_DBG_VARS(_u.transpose());

    // PRX_DBG_VARS(_xdd.transpose());
  }

  virtual void update_configuration() override
  {
    using Vector3 = Eigen::Vector3d;
    using Quaternion = Eigen::Quaterniond;
    const double& pi{ prx::constants::pi };
    const double pi2{ prx::constants::pi / 2.0 };
    const double& theta1{ _x[0] };
    const double& theta2{ _x[1] };

    auto rod1 = configurations["rod1"];
    rod1->setIdentity();
    rod1->linear() = Eigen::Matrix3d(Eigen::AngleAxisd(theta1 - pi2, Vector3::UnitZ()));
    rod1->translation() = rod1->linear() * Vector3(_length_1 / 2, 0.0, 1.5);

    // Endpoint of rod1
    const Vector3 rod1_ep{ rod1->linear() * Vector3(_length_1, 0.0, 1.5) };

    auto rod2 = configurations["rod2"];
    rod2->setIdentity();
    rod2->linear() = rod1->linear() * Eigen::Matrix3d(Eigen::AngleAxisd(theta2, Vector3::UnitZ()));
    rod2->translation() = rod1_ep + rod2->linear() * Vector3(_length_2 / 2, 0.0, 0.0);

    auto ball = configurations["ball"];
    ball->setIdentity();  // No rotation necessary
    ball->translation() = rod1_ep + rod2->linear() * Vector3(_length_2, 0.0, 0.0);
  }

protected:
  virtual void compute_derivative() override final
  {
  }

  State _x;
  Control _u;
  StateDDot _xdd;  // derivative state

  Simulator _sim;
  DPPlant _plant;

  const double _length_1;
  const double _length_2;
  const double _box_w;

  // const Eigen::Vector3d _length1;  // viz only

  const double _dt;
  const std::string _integrator;
};
}  // namespace double_pendulum

PRX_REGISTER_SYSTEM(double_pendulum::acrobot_t, acrobot_dp)
