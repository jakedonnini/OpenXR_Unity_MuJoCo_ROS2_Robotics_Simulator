#include "panda_kinematics/commands/callback_command.hpp"

namespace panda_kinematics {

CallbackCommand::CallbackCommand(
    const std::string& name,
    CallbackFn callback,
    double timeout
) : name_(name), callback_(callback), timeout_(timeout)
{
}

} // namespace panda_kinematics
