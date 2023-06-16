#pragma once

#include <functional>
#include <unordered_map>

template <typename State>
class StateMachine {
public:
    using LoopFunction = std::function<void()>;
    using InitFunction = std::function<void()>;
    using ExitFunction = std::function<void()>;

    void AddState(State state, LoopFunction loop, InitFunction init = nullptr, ExitFunction exit = nullptr) {
        states[state] = { loop, init, exit };
    }

    void SetInitialState(State initialState) {
        currentState = initialState;
    }

    void SetState(const State& newState) {
        if (states.count(currentState)) {
            if (states[currentState].exit) {
                states[currentState].exit();
            }
        }

        currentState = newState;

        if (states.count(currentState)) {
            if (states[currentState].init) {
                states[currentState].init();
            }
        }
    }

    void Loop() {
        if (states.count(currentState)) {
            states[currentState].loop();
        }
    }

private:
    struct StateInfo {
        LoopFunction loop;
        InitFunction init;
        ExitFunction exit;
    };

    State currentState;
    std::unordered_map<State, StateInfo> states;
};
