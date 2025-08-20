#include "common.hpp"
#include "lorawan.h"

#include "cmd_break.hpp"
#include "cmd_cfggedad.hpp"
#include "cmd_cfglorawan.hpp"
#include "cmd_rst.hpp"
#include "cmd_start.hpp"
#include "cmd_traingedad.hpp"
#include "cmd_ver.hpp"

#include "api/context.hpp"

#include "core/logger.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

namespace v0 {

static bool initContext()
{
    static auto serializer = core::Serializer::create('\r');
    if (!serializer) [[unlikely]]
    {
        LOG(ERROR, "Failed to create default serializer");
        return false;
    }
    v0::defaults::serializer = serializer.get();

    return true;
}

static bool preInitHooks()
{
    v0::CmdCfgGEDAD::preInitHook();
    v0::CmdTrainGEDAD::preInitHook();
    v0::CmdStart::preInitHook();
    v0::CmdCfgLoRaWAN::preInitHook();

    // 1. read config from device
    //     std::lock_guard<std::mutex> lock(v0::shared::lorawan_mutex);
    //     auto &lorawan_eui = v0::shared::lorawan_eui;
    //     auto &lorawan_join_eui = v0::shared::lorawan_join_eui;
    //     auto &lorawan_app_key = v0::shared::lorawan_app_key;
    //     auto &lorawan_freq = v0::shared::lorawan_freq;
    // 2. check config, init queue/flag
    // 3. launch lorawan threads
    {
        std::lock_guard<std::mutex> lock(v0::shared::lorawan_mutex);
        auto &lorawan_eui = v0::shared::lorawan_eui;
        auto &lorawan_join_eui = v0::shared::lorawan_join_eui;
        auto &lorawan_app_key = v0::shared::lorawan_app_key;
        auto &lorawan_freq = v0::shared::lorawan_freq;

        do {
            if (lorawan_eui.empty() || lorawan_join_eui.empty() || lorawan_app_key.empty() || lorawan_freq.empty())
            {
                LOG(ERROR, "LoRaWAN configuration is incomplete, please configure it first");
                break;
            }
            if (lorawan_eui.length() != 16 || lorawan_join_eui.length() != 16 ||
                lorawan_app_key.length() != 32 || lorawan_freq.length() < 5)
            {
                LOG(ERROR, "LoRaWAN configuration is invalid, please check the values");
                break;
            }
            lorawan_init(lorawan_eui.c_str(), lorawan_join_eui.c_str(), lorawan_app_key.c_str(), lorawan_freq.c_str());
        } while (false);
    }

    return true;
}

static bool registerCommands()
{
    [[maybe_unused]] static auto break_cmd = v0::CmdBreak();
    [[maybe_unused]] static auto cfggedad_cmd = v0::CmdCfgGEDAD();
    [[maybe_unused]] static auto rst_cmd = v0::CmdRst();
    [[maybe_unused]] static auto start_cmd = v0::CmdStart();
    [[maybe_unused]] static auto traingedad_cmd = v0::CmdTrainGEDAD();
    [[maybe_unused]] static auto ver_cmd = v0::CmdVer();
    [[maybe_unused]] static auto cfglorawan_cmd = v0::CmdCfgLoRaWAN();

    return true;
}

} // namespace v0

namespace api {

core::Status Context::report(core::Status status) noexcept
{
    auto transport = _console_ptr;
    if (!transport) [[unlikely]]
    {
        LOG(ERROR, "Console transport is not initialized");
        return status;
    }

    static size_t counter = 0;

    auto writer = v0::defaults::serializer->writer(v0::defaults::wait_callback,
        std::bind(&v0::defaults::write_callback, std::ref(*transport), std::placeholders::_1, std::placeholders::_2),
        std::bind(&v0::defaults::flush_callback, std::ref(*transport)));

    writer["type"] += v0::ResponseType::System;
    writer["name"] += "SYSTEM";
    writer["tag"] += std::to_string(counter++);
    writer["code"] += status.code();
    writer["msg"] << status.message();
    {
        auto data = writer["data"].writer<core::ObjectWriter>();
        data["ts"]
            += std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch())
                   .count();
    }

    return STATUS_OK();
}

Context *Context::create() noexcept
{
    static Context instance(CONTEXT_VERSION, CONTEXT_CONSOLE, nullptr);

    if (!v0::initContext())
    {
        return nullptr;
    }

    if (!v0::preInitHooks())
    {
        return nullptr;
    }

    if (!v0::registerCommands())
    {
        return nullptr;
    }

    return &instance;
}

} // namespace api
