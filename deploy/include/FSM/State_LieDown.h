// deploy/include/FSM/State_LieDown.h
#pragma once

#include "FSMState.h"
#include "LinearInterpolator.h"

class State_LieDown : public FSMState
{
public:
    State_LieDown(int state, std::string state_string = "LieDown")
    : FSMState(state, state_string)
    {
        ts_ = param::config["FSM"]["LieDown"]["ts"].as<std::vector<float>>();
        qs_ = param::config["FSM"]["LieDown"]["qs"].as<std::vector<std::vector<float>>>();
        assert(ts_.size() == qs_.size());
    }

    void enter()
    {
        static auto kp = param::config["FSM"]["LieDown"]["kp"].as<std::vector<float>>();
        static auto kd = param::config["FSM"]["LieDown"]["kd"].as<std::vector<float>>();
        for(int i(0); i < kp.size(); ++i)
        {
            auto & motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = kp[i];
            motor.kd() = kd[i];
            motor.dq() = motor.tau() = 0;
        }

        std::vector<float> q0;
        for(int i(0); i < kp.size(); ++i) {
            q0.push_back(lowcmd->msg_.motor_cmd()[i].q());
        }
        qs_[0] = q0;
        t0_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
    }

    void run()
    {
        float t = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3 - t0_;
        auto q = linear_interpolate(t, ts_, qs_);
        for(int i(0); i < q.size(); ++i) {
            lowcmd->msg_.motor_cmd()[i].q() = q[i];
        }
    }

private:
    double t0_{0.0};
    std::vector<float> ts_;
    std::vector<std::vector<float>> qs_;
};

REGISTER_FSM(State_LieDown)
