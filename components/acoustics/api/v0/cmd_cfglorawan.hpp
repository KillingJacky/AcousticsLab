#pragma once
#ifndef CMD_CFGLORAWAN_HPP
#define CMD_CFGLORAWAN_HPP

#include "common.hpp"
#include "task_gedad.hpp"

#include "api/command.hpp"
#include "api/executor.hpp"

#include "core/logger.hpp"

#include "hal/device.hpp"
#include "hal/sensor.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace v0 {

// Usage:
// - Set: AT+<TAG>@CFGLORAWAN=<lorawan_eui>,<lorawan_join_eui>,<lorawan_app_key>,<lorawan_freq>
// - Query: AT+<TAG>@CFGLORAWAN
// - Apply Example:
//      1. AT+<TAG>@BREAK
//      2. AT+<TAG>@CFGLORAWAN=<lorawan_eui>,<lorawan_join_eui>,<lorawan_app_key>,<lorawan_freq>
//      3. AT+RST
class CmdCfgLoRaWAN final: public api::Command
{
    static inline constexpr const char *lorawan_eui_path = CONTEXT_PREFIX CONTEXT_VERSION "/lorawan_eui";
    static inline constexpr const char *lorawan_join_eui_path = CONTEXT_PREFIX CONTEXT_VERSION "/lorawan_join_eui";
    static inline constexpr const char *lorawan_app_key_path = CONTEXT_PREFIX CONTEXT_VERSION "/lorawan_app_key";
    static inline constexpr const char *lorawan_freq_path = CONTEXT_PREFIX CONTEXT_VERSION "/lorawan_freq";

public:
    static inline void preInitHook()
    {
        auto device = hal::DeviceRegistry::getDevice();
        if (!device || !device->initialized()) [[unlikely]]
        {
            LOG(ERROR, "Device not registered or not initialized");
            return;
        }

        std::lock_guard<std::mutex> lock(v0::shared::lorawan_mutex);

        {
            size_t size = 0;
            auto status = device->load(hal::Device::StorageType::Internal, lorawan_eui_path, nullptr, size);
            if (status && size > 0)
            {
                v0::shared::lorawan_eui.resize(size);
                status = device->load(hal::Device::StorageType::Internal, lorawan_eui_path,
                    v0::shared::lorawan_eui.data(), size);
            }
            if (!status || size != v0::shared::lorawan_eui.size()) [[unlikely]]
            {
                LOG(ERROR, "Failed to load LoRaWAN EUI or size mismatch, error: %s", status.message().c_str());
            }
        }
        {
            size_t size = 0;
            auto status = device->load(hal::Device::StorageType::Internal, lorawan_join_eui_path, nullptr, size);
            if (status && size > 0)
            {
                v0::shared::lorawan_join_eui.resize(size);
                status = device->load(hal::Device::StorageType::Internal, lorawan_join_eui_path,
                    v0::shared::lorawan_join_eui.data(), size);
            }
            if (!status || size != v0::shared::lorawan_join_eui.size()) [[unlikely]]
            {
                LOG(ERROR, "Failed to load LoRaWAN Join EUI or size mismatch, error: %s", status.message().c_str());
            }
        }
        {
            size_t size = 0;
            auto status = device->load(hal::Device::StorageType::Internal, lorawan_app_key_path, nullptr, size);
            if (status && size > 0)
            {
                v0::shared::lorawan_app_key.resize(size);
                status = device->load(hal::Device::StorageType::Internal, lorawan_app_key_path,
                    v0::shared::lorawan_app_key.data(), size);
            }
            if (!status || size != v0::shared::lorawan_app_key.size()) [[unlikely]]
            {
                LOG(ERROR, "Failed to load LoRaWAN App Key or size mismatch, error: %s", status.message().c_str());
            }
        }
        {
            size_t size = 0;
            auto status = device->load(hal::Device::StorageType::Internal, lorawan_freq_path, nullptr, size);
            if (status && size > 0)
            {
                v0::shared::lorawan_freq.resize(size);
                status = device->load(hal::Device::StorageType::Internal, lorawan_freq_path,
                    v0::shared::lorawan_freq.data(), size);
            }
            if (!status || size != v0::shared::lorawan_freq.size()) [[unlikely]]
            {
                LOG(ERROR, "Failed to load LoRaWAN Frequency or size mismatch, error: %s", status.message().c_str());
            }
        }
    }

    CmdCfgLoRaWAN()
        : Command("CFGLORAWAN", "Configure LoRaWan",
              core::ConfigObjectMap { CONFIG_OBJECT_DECL_STRING("lorawan_eui", "LoRaWAN EUI", v0::shared::lorawan_eui),
                  CONFIG_OBJECT_DECL_STRING("lorawan_join_eui", "LoRaWAN Join EUI", v0::shared::lorawan_join_eui),
                  CONFIG_OBJECT_DECL_STRING("lorawan_app_key", "LoRaWAN App Key", v0::shared::lorawan_app_key),
                  CONFIG_OBJECT_DECL_STRING("lorawan_freq", "LoRaWAN Frequency", v0::shared::lorawan_freq) }),
          _device(hal::DeviceRegistry::getDevice())
    {
    }

    ~CmdCfgLoRaWAN() noexcept override = default;

    std::shared_ptr<api::Task> operator()(api::Context &context, hal::Transport &transport, const core::ConfigMap &args,
        size_t id) override
    {
        auto status = STATUS_OK();

        std::lock_guard<std::mutex> lock(v0::shared::lorawan_mutex);

        if (auto it = args.find("@0"); it != args.end())
        {
            if (auto lorawan_eui = std::get_if<std::string>(&it->second);
                lorawan_eui != nullptr && !lorawan_eui->empty())
            {
                if (*lorawan_eui != v0::shared::lorawan_eui)
                {
                    if (_device
                        && !(status = _device->store(hal::Device::StorageType::Internal, lorawan_eui_path,
                                 lorawan_eui->c_str(), lorawan_eui->size())))
                    {
                        goto Reply;
                    }
                    v0::shared::lorawan_eui = *lorawan_eui;
                }
            }
        }

        if (auto it = args.find("@1"); it != args.end())
        {
            if (auto lorawan_join_eui = std::get_if<std::string>(&it->second);
                lorawan_join_eui != nullptr && !lorawan_join_eui->empty())
            {
                if (*lorawan_join_eui != v0::shared::lorawan_join_eui)
                {
                    if (_device
                        && !(status = _device->store(hal::Device::StorageType::Internal, lorawan_join_eui_path,
                                 lorawan_join_eui->c_str(), lorawan_join_eui->size())))
                    {
                        goto Reply;
                    }
                    v0::shared::lorawan_join_eui = *lorawan_join_eui;
                }
            }
        }

        if (auto it = args.find("@2"); it != args.end())
        {
            if (auto lorawan_app_key = std::get_if<std::string>(&it->second);
                lorawan_app_key != nullptr && !lorawan_app_key->empty())
            {
                if (*lorawan_app_key != v0::shared::lorawan_app_key)
                {
                    if (_device
                        && !(status = _device->store(hal::Device::StorageType::Internal, lorawan_app_key_path,
                                 lorawan_app_key->c_str(), lorawan_app_key->size())))
                    {
                        goto Reply;
                    }
                    v0::shared::lorawan_app_key = *lorawan_app_key;
                }
            }
        }

        if (auto it = args.find("@3"); it != args.end())
        {
            if (auto lorawan_freq = std::get_if<std::string>(&it->second);
                lorawan_freq != nullptr && !lorawan_freq->empty())
            {
                if (*lorawan_freq != v0::shared::lorawan_freq)
                {
                    if (_device
                        && !(status = _device->store(hal::Device::StorageType::Internal, lorawan_freq_path,
                                 lorawan_freq->c_str(), lorawan_freq->size())))
                    {
                        goto Reply;
                    }
                    v0::shared::lorawan_freq = *lorawan_freq;
                }
            }
        }

    Reply: {
        auto writer = v0::defaults::serializer->writer(v0::defaults::wait_callback,
            std::bind(&v0::defaults::write_callback, std::ref(transport), std::placeholders::_1, std::placeholders::_2),
            std::bind(&v0::defaults::flush_callback, std::ref(transport)));

        writer["type"] += v0::ResponseType::Direct;
        writer["name"] += _name;
        if (auto it = args.find("@cmd_tag"); it != args.end())
        {
            auto tag = std::get_if<std::string>(&it->second);
            if (tag != nullptr)
                writer["tag"] << *tag;
        }
        writer["code"] += status.code();
        writer["msg"] << status.message();
        {
            auto data = writer["data"].writer<core::ObjectWriter>();
            {
                data["lorawan_eui"] << v0::shared::lorawan_eui;
                data["lorawan_join_eui"] << v0::shared::lorawan_join_eui;
                data["lorawan_app_key"] << v0::shared::lorawan_app_key;
                data["lorawan_freq"] << v0::shared::lorawan_freq;
            }
        }
    }

        return nullptr;
    }

private:
    hal::Device *_device;
};

} // namespace v0

#endif // CMD_CFGLORAWAN_HPP
