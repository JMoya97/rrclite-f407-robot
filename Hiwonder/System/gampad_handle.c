#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_hid.h"
#include "usbh_hid_gamepad.h"
#include "cmsis_os2.h"

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
    extern osMessageQueueId_t moving_ctrl_queueHandle;
    static HID_GAMEPAD_Info_TypeDef last_info;

    switch(USBH_HID_GetDeviceType(phost)) {
        case 0xFF: {/* 手柄数据 */
            HID_GAMEPAD_Info_TypeDef *info = USBH_HID_GetGamepadInfo(phost);
			if(info == NULL) {
				break;
			}
            if(info->hat & 0x08) {
                char msg = (((info->hat & 0x07) + 1) & 0x07) + 0x41;
                osMessageQueuePut(moving_ctrl_queueHandle, &msg, 0, 0);
            }else{
				if(last_info.hat != info->hat) { /* 只在松开时发送一次松开信号 */
					osMessageQueuePut(moving_ctrl_queueHandle, "O", 0, 0);
				}
			}
			
			
			
			/** 松开时触发 */
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_TRIANGLE) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_TRIANGLE)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "J", 0, 0);
            }
			if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_CIRCLE) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_CIRCLE)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "K", 0, 0);
            }
			if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_SQUARE) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_SQUARE)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "L", 0, 0);
            }
			if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_CROSS) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_CROSS)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "M", 0, 0);
            }
			
			memcpy(&last_info, info, sizeof(HID_GAMEPAD_Info_TypeDef));
            break;
        }
        default:
            break;
    }
}