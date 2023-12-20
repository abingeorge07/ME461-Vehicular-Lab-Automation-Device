#include "IR_lib.c"

void app_main(void){

    IR_init();

    while(1){
        uint32_t out = getIR();
        printf("Output value is %ld\n", out);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
       
}