//
// Created by suave on 9/17/24.
//

#ifndef CONTROLLERMACROS_H
#define CONTROLLERMACROS_H

#include <chrono>

#include "../util/Util.h"

#define sleep(sec) std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(sec * 1e6)));

// REQUIRES m_end_controller IN SCOPE
#define try_mav(act, success) \
{\
suave_log << "Starting " << #act << std::endl; \
if (m_end_controller)     \
{\
suave_err << "m_end_controller = true" << std::endl; \
this->shutdown();\
return;\
}\
const auto act_result = act; \
if (act_result != success) \
{ \
suave_err << #act << " failed: " << act_result << std::endl; \
this->shutdown(); \
return; \
} \
suave_log << #act << ": " << act_result << std::endl;\
}

#define try_action(act) try_mav(act, Action::Result::Success)
#define try_offboard(act) try_mav(act, Offboard::Result::Success)

#endif //CONTROLLERMACROS_H
