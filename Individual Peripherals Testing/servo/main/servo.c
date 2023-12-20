#include"servo_lib.c"

void app_main(void)
{
    servo_init();
    int angle =-90;// SERVO_MAX_PULSEWIDTH_US;
    int step = 1;
    while (1) {
        
        //Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
        setAngle(angle);
        printf("Pulsewidth: %d\n", angle);
        angle += (step*5);


        if(angle>=90 || angle=<-90)
            step *= -1;
        

        vTaskDelay(pdMS_TO_TICKS(2000));
              
    }
}