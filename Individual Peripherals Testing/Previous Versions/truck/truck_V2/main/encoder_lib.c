#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/pcnt.h"
#include <sys/cdefs.h>
#include "esp_compiler.h"
#include "rotary_encoder.h"


#define GLITCH_FILTER 1 // in useconds
#define PIN1 14
#define PIN2 27
#define PULSES_PER_REVOLUTION 2400
#define PI 3.1415
// static const char *TAG = "example";


float absolutePosition = 0;
float angle = 0;
float radius = 2.0;


// Rotary encoder underlying device is represented by a PCNT unit in this example
uint32_t pcnt_unit = 0;
int32_t encoderValue = 0;
float prevPosition = 0;
rotary_encoder_t *encoder = NULL;

void encoderInit(void){

    // Create rotary encoder instance
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, PIN1, PIN2);
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, GLITCH_FILTER));
}

void startEncoder(void){
    // Start encoder
    ESP_ERROR_CHECK(encoder->start(encoder));
}

// set radius of the wheel
void setRadius(float radiusN){
    radius = radiusN;
}

// distance traveled since previous checkpoint
float getDistanceTraveled(void)
{
    encoderValue = encoder->get_counter_value(encoder);
    absolutePosition = ((float) encoderValue/PULSES_PER_REVOLUTION)* 2*PI*radius;
    float distance = absolutePosition - prevPosition;
    prevPosition = absolutePosition;

    return distance;
}

// absolution position of the truck
float get_absolutePosition(void)
{
    encoderValue = encoder->get_counter_value(encoder);
    absolutePosition = ((float)encoderValue/PULSES_PER_REVOLUTION)* 2*PI*radius;
    prevPosition = absolutePosition;
    return absolutePosition;
}


// total number of revolutions run so far
float absoluteRevolutions(void)
{
    encoderValue = encoder->get_counter_value(encoder);
    angle = ((float)encoderValue/PULSES_PER_REVOLUTION)* 360;
    return angle;
}