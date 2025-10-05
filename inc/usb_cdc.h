#ifndef __USB_CDC_H__
#define __USB_CDC_H__

#include <stdint.h>

#include "pico/stdlib.h"
#include "manipulator_packets.h"

#define USB_CDC_RX_BUFFER_SIZE (2048)

bool usb_cdc_send_manipulator_feedback_packet(ManipulatorFeedbackPacket* pkt);
bool usb_cdc_send_manipulator_state_packet(ManipulatorStatePacket* pkt);

bool usb_cdc_receive_manipulator_command_packet(ManipulatorCommandPacket* command_packet, uint32_t timeout_ms);
bool usb_cdc_receive_manipulator_init_packet(ManipulatorInitPacket* init_packet, uint32_t timeout_ms);



#endif // __USB_CDC_H__