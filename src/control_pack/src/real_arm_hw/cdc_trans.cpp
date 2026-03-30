#include "real_arm_hw/cdc_trans.hpp"

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace real_arm_hardware {

CDCTrans::CDCTrans()
{
    handling_events_ = true;
    libusb_init(&ctx_);
}

CDCTrans::~CDCTrans()
{
    handling_events_ = false;
    close();

    if (ctx_ != nullptr) {
        libusb_exit(ctx_);
        ctx_ = nullptr;
    }
}

bool CDCTrans::open(uint16_t vid, uint16_t pid)
{
    last_vid_ = vid;
    last_pid_ = pid;

    handle_ = libusb_open_device_with_vid_pid(ctx_, vid, pid);
    RCLCPP_INFO(rclcpp::get_logger("cdc_device"), "Trying to open USB CDC device");
    if (handle_ == nullptr) {
        RCLCPP_WARN(rclcpp::get_logger("cdc_device"), "Failed to open USB CDC device");
        return false;
    }

    if (libusb_kernel_driver_active(handle_, interface_number_) == 1) {
        const int ret = libusb_detach_kernel_driver(handle_, interface_number_);
        RCLCPP_INFO(rclcpp::get_logger("cdc_device"), "Detached kernel driver on interface %d, ret=%d", interface_number_, ret);
    }

    const int claim_ret = libusb_claim_interface(handle_, interface_number_);
    if (claim_ret != LIBUSB_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("cdc_device"), "Failed to claim interface %d, ret=%d", interface_number_, claim_ret);
        libusb_close(handle_);
        handle_ = nullptr;
        return false;
    }

    recv_transfer_ = libusb_alloc_transfer(0);
    if (recv_transfer_ == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("cdc_device"), "Failed to allocate receive transfer");
        close();
        return false;
    }

    libusb_fill_bulk_transfer(
        recv_transfer_,
        handle_,
        EP_IN,
        rx_buffer_,
        sizeof(rx_buffer_),
        [](libusb_transfer * transfer) {
            auto * self = static_cast<CDCTrans *>(transfer->user_data);

            if (!self->handling_events_) {
                libusb_cancel_transfer(transfer);
                return;
            }

            if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
                self->disconnected_ = true;
                return;
            }

            if (self->recv_cb_) {
                self->recv_cb_(transfer->buffer, transfer->actual_length);
            }

            if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS) {
                self->disconnected_ = true;
            }
        },
        this,
        0);

    if (libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG) && !hotplug_registered_) {
        const int rc = libusb_hotplug_register_callback(
            ctx_,
            static_cast<libusb_hotplug_event>(LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT | LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED),
            LIBUSB_HOTPLUG_NO_FLAGS,
            vid,
            pid,
            LIBUSB_HOTPLUG_MATCH_ANY,
            [](libusb_context *, libusb_device *, libusb_hotplug_event event, void * user_data) {
                static_cast<CDCTrans *>(user_data)->on_hotplug(event);
                return 0;
            },
            this,
            &hotplug_handle_);

        hotplug_registered_ = (rc == LIBUSB_SUCCESS);
    }

    if (libusb_submit_transfer(recv_transfer_) != LIBUSB_SUCCESS) {
        RCLCPP_WARN(rclcpp::get_logger("cdc_device"), "Failed to submit receive transfer");
        close();
        return false;
    }

    disconnected_ = false;
    need_reconnect_ = false;
    handling_events_ = true;
    return true;
}

void CDCTrans::close()
{
    if (hotplug_registered_ && ctx_ != nullptr) {
        libusb_hotplug_deregister_callback(ctx_, hotplug_handle_);
        hotplug_registered_ = false;
    }

    if (recv_transfer_ != nullptr) {
        libusb_cancel_transfer(recv_transfer_);
        libusb_free_transfer(recv_transfer_);
        recv_transfer_ = nullptr;
    }

    if (handle_ != nullptr) {
        libusb_release_interface(handle_, interface_number_);
        libusb_close(handle_);
        handle_ = nullptr;
    }
}

int CDCTrans::send(const uint8_t * data, int size, unsigned int timeout)
{
    if (disconnected_ || handle_ == nullptr) {
        return -2;
    }

    int actual_size = 0;
    const int rc = libusb_bulk_transfer(handle_, EP_OUT, const_cast<uint8_t *>(data), size, &actual_size, timeout);
    if (rc != LIBUSB_SUCCESS) {
        RCLCPP_WARN(
            rclcpp::get_logger("package_comm"),
            "USB send failed: expected=%d actual=%d ret=%d",
            size,
            actual_size,
            rc);
        return -1;
    }

    return actual_size;
}

void CDCTrans::register_recv_cb(std::function<void(const uint8_t * data, int size)> recv_cb)
{
    recv_cb_ = std::move(recv_cb);
}

void CDCTrans::process_once()
{
    timeval tv{0, 50000};
    libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);

    if (disconnected_) {
        close();
    }

    if (need_reconnect_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        need_reconnect_ = false;
        open(last_vid_, last_pid_);
    }
}

void CDCTrans::on_hotplug(libusb_hotplug_event event)
{
    if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
        disconnected_ = true;
    } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
        need_reconnect_ = true;
    }
}

}  // namespace real_arm_hardware
