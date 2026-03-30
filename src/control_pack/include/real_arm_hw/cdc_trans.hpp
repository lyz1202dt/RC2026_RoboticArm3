#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <type_traits>

#include <libusb-1.0/libusb.h>

namespace real_arm_hardware {

class CDCTrans {
public:
    static constexpr uint8_t EP_OUT = 0x01;
    static constexpr uint8_t EP_IN = 0x81;

    CDCTrans();
    ~CDCTrans();

    bool open(uint16_t vid, uint16_t pid);
    void close();
    int send(const uint8_t * data, int size, unsigned int timeout = 5);

    void register_recv_cb(std::function<void(const uint8_t * data, int size)> recv_cb);

    template <typename T>
    bool send_struct(const T & pack, unsigned int timeout = 5)
    {
        static_assert(std::is_standard_layout<T>::value, "pack must use standard layout");
        static_assert(std::is_trivial<T>::value, "pack must be trivial");

        return send(reinterpret_cast<const uint8_t *>(&pack), static_cast<int>(sizeof(T)), timeout) >= 0;
    }

    void process_once();

private:
    void on_hotplug(libusb_hotplug_event event);

    uint16_t last_vid_{0};
    uint16_t last_pid_{0};
    int interface_number_{1};
    std::function<void(const uint8_t * data, int size)> recv_cb_;
    std::atomic_bool disconnected_{true};
    std::atomic_bool need_reconnect_{false};
    std::atomic_bool handling_events_{true};
    uint8_t rx_buffer_[2048]{};
    libusb_transfer * recv_transfer_{nullptr};
    libusb_context * ctx_{nullptr};
    libusb_device_handle * handle_{nullptr};
    libusb_hotplug_callback_handle hotplug_handle_{};
    bool hotplug_registered_{false};
};

}  // namespace real_arm_hardware
