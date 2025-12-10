#include "data_server.h"
#include <mujoco/mjplugin.h>

namespace mujoco::plugin::dataserver {

mjPLUGIN_LIB_INIT { DataServer::RegisterPlugin(); }

} // namespace mujoco::plugin::dataserver