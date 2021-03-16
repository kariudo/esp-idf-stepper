#include <stdio.h>
#include <string.h> /* memset */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <time.h>
#include <sys/time.h>
#include "driver/gpio.h"

#define LOW 0
#define HIGH 1

#define MILLISECONDS_STEP_HOLD 1  // time in milliseconds between motor steps
#define MOTOR_PHASE_A 32          // motor phase a pin
#define MOTOR_PHASE_B 14          // motor phase b pin
#define MOTOR_PHASE_C 33          // motor phase c pin
#define MOTOR_PHASE_D 15          // motor phase d pin
#define STEPS_PER_REVOLUTION 4096 // steps for one full revolution of the stepper

#define GPIO_OUTPUT_PIN_SEL ((1ULL << MOTOR_PHASE_A) | (1ULL << MOTOR_PHASE_B) | (1ULL << MOTOR_PHASE_C) | (1ULL << MOTOR_PHASE_D))

int nDelay = 10;           // main loop delay
long nSeconds = 1;          // seconds
long nSecondsIndicated = 0; // secondsindicated by clock
struct timeval tvTimeValue; // time value structure

void loop();

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // I/O.

    // Set motor drive pins as outputs.

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // Set esp32 rtc to 00:00:00.

    memset(&tvTimeValue, 0, sizeof(tvTimeValue));
    settimeofday(&tvTimeValue, NULL);

    // End of setup.

    while (true)
    {
        loop();
    }


    for (int i = 10; i >= 0; i--)
    {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotorOff
// Turn off motor drive.
// Entry   : nothing
// Returns : nothing
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotorOff()
{
    gpio_set_level(MOTOR_PHASE_A, LOW);
    gpio_set_level(MOTOR_PHASE_B, LOW);
    gpio_set_level(MOTOR_PHASE_C, LOW);
    gpio_set_level(MOTOR_PHASE_D, LOW);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Step
// Step the stepper motor.
// Entry   : direction (>= 0 for clockwise step, < 0 for counter clockwise step)
// Returns : nothing
//
// Notes   : 1) For this stepper motor, 1 step is (1 / STEPS_PER_REVOLUTION) degrees.
//
//           2) Forward clock motion is performed in 8 steps by driving the motor phases, in sequence, as follows:
//
//              a) phase d
//              b) phase d and c
//              c) phase c
//              d) phase c and b
//              e) phase b
//              f) phase b and a
//              g) phase a
//              h) phase a and d
//
//            3) Reverse clock motion is performed in the reverse order of 2).
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Step(int nDirection)
{
    // Local variables.

    static int nPhase = 0;

    // Update phase.

    nPhase = ((nDirection < 0) ? (nPhase - 1) : (nPhase + 1)) & 7;

    // Step this phase.

    switch (nPhase)
    {
    case 0:
    {
        gpio_set_level(MOTOR_PHASE_D, HIGH);
        gpio_set_level(MOTOR_PHASE_C, LOW);
        gpio_set_level(MOTOR_PHASE_B, LOW);
        gpio_set_level(MOTOR_PHASE_A, LOW);
    }
    break;

    case 1:
    {
        gpio_set_level(MOTOR_PHASE_D, HIGH);
        gpio_set_level(MOTOR_PHASE_C, HIGH);
        gpio_set_level(MOTOR_PHASE_B, LOW);
        gpio_set_level(MOTOR_PHASE_A, LOW);
    }
    break;

    case 2:
    {
        gpio_set_level(MOTOR_PHASE_D, LOW);
        gpio_set_level(MOTOR_PHASE_C, HIGH);
        gpio_set_level(MOTOR_PHASE_B, LOW);
        gpio_set_level(MOTOR_PHASE_A, LOW);
    }
    break;

    case 3:
    {
        gpio_set_level(MOTOR_PHASE_D, LOW);
        gpio_set_level(MOTOR_PHASE_C, HIGH);
        gpio_set_level(MOTOR_PHASE_B, HIGH);
        gpio_set_level(MOTOR_PHASE_A, LOW);
    }
    break;

    case 4:
    {
        gpio_set_level(MOTOR_PHASE_D, LOW);
        gpio_set_level(MOTOR_PHASE_C, LOW);
        gpio_set_level(MOTOR_PHASE_B, HIGH);
        gpio_set_level(MOTOR_PHASE_A, LOW);
    }
    break;

    case 5:
    {
        gpio_set_level(MOTOR_PHASE_D, LOW);
        gpio_set_level(MOTOR_PHASE_C, LOW);
        gpio_set_level(MOTOR_PHASE_B, HIGH);
        gpio_set_level(MOTOR_PHASE_A, HIGH);
    }
    break;

    case 6:
    {
        gpio_set_level(MOTOR_PHASE_D, LOW);
        gpio_set_level(MOTOR_PHASE_C, LOW);
        gpio_set_level(MOTOR_PHASE_B, LOW);
        gpio_set_level(MOTOR_PHASE_A, HIGH);
    }
    break;

    case 7:
    {
        gpio_set_level(MOTOR_PHASE_D, HIGH);
        gpio_set_level(MOTOR_PHASE_C, LOW);
        gpio_set_level(MOTOR_PHASE_B, LOW);
        gpio_set_level(MOTOR_PHASE_A, HIGH);
    }
    break;
    }

    // Hold this step for MILLISECONDS_STEP_HOLD milliseconds.

    vTaskDelay(MILLISECONDS_STEP_HOLD / portTICK_PERIOD_MS);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Update
// Update stepper position.
// Entry   : nothing
// Returns : -1 if update occurred, 0 if not.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

int Update()
{
    // Local variables.

    long nClockwiseSeconds = 0;
    long nCounterClockwiseSeconds = 0;
    long nStepCount = 0;
    long nStepDirection = 0;
    long nStepPosition = 0;
    long nStepPositionN1 = 0;

    // Calculate clockwise and counterclockwise seconds required to drive indicated seconds to actual seconds.

    if (nSeconds > nSecondsIndicated)
    {
        nClockwiseSeconds = nSeconds - nSecondsIndicated;
        nCounterClockwiseSeconds = (nSecondsIndicated + 60) - nSeconds;
    }
    else if (nSeconds < nSecondsIndicated)
    {
        nClockwiseSeconds = (nSeconds + 60) - nSecondsIndicated;
        nCounterClockwiseSeconds = nSecondsIndicated - nSeconds;
    }

    // Check if update is needed.

    if ((nClockwiseSeconds) || (nCounterClockwiseSeconds))
    {
        // Update is needed, determine shortest direction.

        if (nClockwiseSeconds < nCounterClockwiseSeconds)
        {
            // Clockwise movement is shorter.

            nStepDirection = 1;
        }
        else
        {
            // Counterclockwise movement is shorter.

            nStepDirection = -1;
        }

        // Drive indicated seconds to seconds.

        while (nSeconds != nSecondsIndicated)
        {
            // Calculate n-1 step position.

            nStepPositionN1 = ((nSecondsIndicated % 60) * STEPS_PER_REVOLUTION) / 60;

            // Update seconds.

            nSecondsIndicated = (nSecondsIndicated + nStepDirection) % 60;
            nSecondsIndicated = (nSecondsIndicated < 0) ? 60 + nSecondsIndicated : nSecondsIndicated;

            // Calculate current step position.

            nStepPosition = ((nSecondsIndicated % 60) * STEPS_PER_REVOLUTION) / 60;

            // Calculate step count.

            nStepCount = ((nStepDirection > 0) ? nStepPosition - nStepPositionN1 : nStepPositionN1 - nStepPosition);
            nStepCount = (nStepCount < 0) ? STEPS_PER_REVOLUTION + nStepCount : nStepCount;

            // Step the required steps.

            while (nStepCount)
            {
                Step(nStepDirection);
                nStepCount = nStepCount - 1;
            }
        }
    }
    else
    {
        // No update performed.

        return 0;
    }

    // Update was performed, remove motor power.

    MotorOff();

    // Update performed.

    return -1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Loop.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() 
{
  // Update nSeconds from the ESP32 rtc.
  
  gettimeofday(& tvTimeValue, NULL);
  
  struct tm * tmPointer = localtime(& tvTimeValue.tv_sec);
  nSeconds = tmPointer->tm_sec;
  
  // Update the stepper position.
  
  Update();
  
  // Delay nDelay seconds.

  vTaskDelay(nDelay / portTICK_PERIOD_MS);
}