#include "Controls.h"
#include "Arduino.h"
#include "Pinout.h"
#include "Commands.h"
#include "Queues.h"
#include "Communications.h"
#include <FreeRTOS.h>

extern QueueHandle_t CommandQueue;

// Configuration
#define ABORT_REPEAT_INTERVAL_MS 500
#define ARM_AUTO_DISARM_TIMEOUT_MS 30000  // Auto-disarm after 30 seconds
#define BUZZER_BEEP_INTERVAL_MS 1000      // Beep every 1 second when armed
#define BUZZER_BEEP_DURATION_MS 100       // Buzzer beep duration

// Button state structure
typedef struct {
    uint8_t pin;
    uint8_t current_state;
    uint8_t previous_state;
    uint32_t last_abort_cmd_time_ms;
} ButtonState_t;

// Global button states
ButtonState_t arm_button = {ARM_DETECT_PIN, LOW, LOW, 0};
ButtonState_t fire_button = {FIRE_DETECT_PIN, LOW, LOW, 0};
ButtonState_t abort_button = {ABORT_DETECT_PIN, HIGH, HIGH, 0};

// System state
int armed = false;
int ready_to_arm = false;
uint32_t arm_start_time_ms = 0;  // Timestamp when armed
uint32_t last_buzzer_beep_ms = 0;  // Timestamp of last buzzer beep

void vControlsTask(void *pvParameters)
{
    // Initialize pins as inputs
    pinMode(ARM_DETECT_PIN, INPUT);
    pinMode(FIRE_DETECT_PIN, INPUT);
    pinMode(ABORT_DETECT_PIN, INPUT);
    
    while (true)
    {
        uint32_t current_time_ms = millis();
        
        // Read current button states
        arm_button.current_state = digitalRead(arm_button.pin);
        fire_button.current_state = digitalRead(fire_button.pin);
        abort_button.current_state = digitalRead(abort_button.pin);

        // ========== ARM BUTTON: LOW→HIGH transition ==========
        if (arm_button.current_state == HIGH && arm_button.previous_state == LOW && abort_button.current_state == HIGH) {
            packet_t arm_cmd;
            create_packet(&arm_cmd, MISSION_CONTROL_ID, BROADCAST_ID, CMD_ARM, NULL, 0);
            xQueueSend(CommandQueue, &arm_cmd, portMAX_DELAY);
            armed = true;
            arm_start_time_ms = current_time_ms;
            last_buzzer_beep_ms = current_time_ms;
            tone(BUZZER_PIN, 1000, 100); // Beep to indicate armed
        } else if (arm_button.current_state == LOW && arm_button.previous_state == HIGH) {
            packet_t disarm_cmd;
            create_packet(&disarm_cmd, MISSION_CONTROL_ID, BROADCAST_ID, CMD_STOP, NULL, 0);
            xQueueSend(CommandQueue, &disarm_cmd, portMAX_DELAY);
            armed = false;
            noTone(BUZZER_PIN); // Stop buzzer when disarmed
        }
        
        // ========== FIRE BUTTON: LOW→HIGH transition (only if armed) ==========
        if (fire_button.current_state == HIGH && fire_button.previous_state == LOW) {
            if (armed) {
                packet_t fire_cmd;
                create_packet(&fire_cmd, MISSION_CONTROL_ID, BROADCAST_ID, CMD_FIRE, NULL, 0);
                xQueueSend(CommandQueue, &fire_cmd, portMAX_DELAY);
                for (int i = 0; i < 3; i++) {
                    tone(BUZZER_PIN, 2000, 30); // Beep to indicate fire command sent
                    delay(100);
                }
            }
        }
        
        // ========== ABORT BUTTON: Repeat while LOW (Low means abort) ==========
        if (abort_button.current_state == LOW) {
            // Send abort command every ABORT_REPEAT_INTERVAL_MS while button is held
            if (current_time_ms - abort_button.last_abort_cmd_time_ms >= ABORT_REPEAT_INTERVAL_MS) {
                packet_t abort_cmd;
                create_packet(&abort_cmd, MISSION_CONTROL_ID, BROADCAST_ID, CMD_ABORT, NULL, 0);
                xQueueSend(CommandQueue, &abort_cmd, portMAX_DELAY);
                abort_button.last_abort_cmd_time_ms = current_time_ms;
            }
        } else if (abort_button.current_state == HIGH && abort_button.previous_state == LOW) {
            packet_t abort_release_cmd;
            create_packet(&abort_release_cmd, MISSION_CONTROL_ID, BROADCAST_ID, CMD_STOP, NULL, 0);
            xQueueSend(CommandQueue, &abort_release_cmd, portMAX_DELAY);
        }
        
        // ========== ARMED STATE MANAGEMENT ==========
        if (armed) {
            // Auto-disarm after timeout
            if (current_time_ms - arm_start_time_ms >= ARM_AUTO_DISARM_TIMEOUT_MS) {
                packet_t auto_disarm_cmd;
                create_packet(&auto_disarm_cmd, MISSION_CONTROL_ID, BROADCAST_ID, CMD_STOP, NULL, 0);
                xQueueSend(CommandQueue, &auto_disarm_cmd, portMAX_DELAY);
                armed = false;
            }
            // Beep buzzer every second while armed
            else if (current_time_ms - last_buzzer_beep_ms >= BUZZER_BEEP_INTERVAL_MS) {
                tone(BUZZER_PIN, 1000, BUZZER_BEEP_DURATION_MS);
                last_buzzer_beep_ms = current_time_ms;
            }
        }
        
        // Update previous states for next iteration
        arm_button.previous_state = arm_button.current_state;
        fire_button.previous_state = fire_button.current_state;
        abort_button.previous_state = abort_button.current_state;
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}