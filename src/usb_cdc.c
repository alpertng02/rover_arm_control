/**
 *@file usb_cdc.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-10-02
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "usb_cdc.h"
#include "tusb.h"

static uint8_t usb_cdc_rx_buffer[USB_CDC_RX_BUFFER_SIZE] = { 0 };
static size_t usb_cdc_rx_index = 0;

bool usb_cdc_send_manipulator_feedback_packet(ManipulatorFeedbackPacket* pkt) {
 
     pkt->frame_start = MANIPULATOR_FRAME_START;
     pkt->frame_end = MANIPULATOR_FRAME_END;
 
     uint32_t bytes_written = tud_cdc_write(pkt, sizeof(ManipulatorFeedbackPacket));
     tud_cdc_write_flush();
 
     if (bytes_written != sizeof(ManipulatorFeedbackPacket)) {
         return false;
     } else {
         return true;
     }
 }

bool usb_cdc_send_manipulator_state_packet(ManipulatorStatePacket* pkt) {
 
    pkt->frame_start = MANIPULATOR_FRAME_START;
    pkt->frame_end = MANIPULATOR_FRAME_END;

    uint32_t bytes_written = tud_cdc_write(pkt, sizeof(ManipulatorStatePacket));
    tud_cdc_write_flush();

    if (bytes_written != sizeof(ManipulatorStatePacket)) {
        return false;
    } else {
        return true;
    }
}
 
bool usb_cdc_receive_manipulator_command_packet(ManipulatorCommandPacket* command_packet, uint32_t timeout_ms) {
     uint32_t start = to_ms_since_boot(get_absolute_time());
 
     while (to_ms_since_boot(get_absolute_time()) - start < timeout_ms) {
         tud_task();
 
         while (tud_cdc_available()) {
             uint8_t b;
             if (tud_cdc_read(&b, 1) == 1) {
                 usb_cdc_rx_buffer[usb_cdc_rx_index++] = b;
 
                 // prevent overflow
                 if (usb_cdc_rx_index >= USB_CDC_RX_BUFFER_SIZE) {
                     usb_cdc_rx_index = 0;
                 }
 
                 // Do we have enough for a packet?
                 if (usb_cdc_rx_index >= sizeof(ManipulatorCommandPacket)) {
                     // Try to interpret last N bytes as a packet
                     ManipulatorCommandPacket* pkt =
                         (ManipulatorCommandPacket*) (usb_cdc_rx_buffer + usb_cdc_rx_index - sizeof(ManipulatorCommandPacket));
 
                     if (pkt->frame_start == MANIPULATOR_FRAME_START && pkt->frame_end == MANIPULATOR_FRAME_END) {
                         // ✅ Valid packet found
 
                         usb_cdc_rx_index = 0; // reset for next packet
                         memcpy(command_packet, pkt, sizeof(ManipulatorCommandPacket));
                         return true;
                     } else {
                         // ❌ Not a valid packet, shift buffer to resync
                         // Move window forward by 1 byte
                         memmove(usb_cdc_rx_buffer, usb_cdc_rx_buffer + 1, --usb_cdc_rx_index);
                     }
                 }
             }
         }
     }
 
     return false; // timeout
}
 
bool usb_cdc_receive_manipulator_init_packet(ManipulatorInitPacket *init_packet, uint32_t timeout_ms) {
     uint32_t start = to_ms_since_boot(get_absolute_time());
 
     while (to_ms_since_boot(get_absolute_time()) - start < timeout_ms) {
         tud_task();
 
         while (tud_cdc_available()) {
             uint8_t b;
             if (tud_cdc_read(&b, 1) == 1) {
                 usb_cdc_rx_buffer[usb_cdc_rx_index++] = b;
 
                 // prevent overflow
                 if (usb_cdc_rx_index >= USB_CDC_RX_BUFFER_SIZE) {
                     usb_cdc_rx_index = 0;
                 }
 
                 // Do we have enough for a packet?
                 if (usb_cdc_rx_index >= sizeof(ManipulatorInitPacket)) {
                     // Try to interpret last N bytes as a packet
                     ManipulatorInitPacket* pkt =
                         (ManipulatorInitPacket*) (usb_cdc_rx_buffer + usb_cdc_rx_index - sizeof(ManipulatorInitPacket));
 
                     if (pkt->frame_start == MANIPULATOR_FRAME_START && pkt->frame_end == MANIPULATOR_FRAME_END) {
                         // ✅ Valid packet found
                         usb_cdc_rx_index = 0; // reset for next packet
                         memcpy(init_packet, pkt, sizeof(ManipulatorInitPacket));
                         return true;
                     } else {
                         // ❌ Not a valid packet, shift buffer to resync
                         // Move window forward by 1 byte
                         memmove(usb_cdc_rx_buffer, usb_cdc_rx_buffer + 1, --usb_cdc_rx_index);
                     }
                 }
             }
         }
     }
 
     return false; // timeout
}

 