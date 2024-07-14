#include <stdio.h>

typedef enum { BAT_OK_SOL_LOW , SOL_OK, SOL_OK_BAT_HIGH, ALL_LOW, MAX_EVENT } Event ;
typedef enum { IDLE , CHARGE_MPPT, CHARGE_TOPPING, CHARGE_FLOAT, MAX_STATE } State ;
typedef enum { OFF, ON } Run_state; 
typedef void (* Action ) ( void) ;
typedef struct {
Action to_do ; // function pointer to current-state action
State next_state ; // next-state enumerator
} Table_Cell ;
Table_Cell Table [ MAX_STATE][ MAX_EVENT ] = {  
/*  BAT_OK_SOL_LOW                 SOL_OK                         SOL_OK_BAT_HIGH                 ALL LOW               */
{ { do_idle , IDLE } , { do_charge_mppt , CHARGE_MPPT }, { do_charge_float , CHARGE_FLOAT }, { halt , IDLE } } ,         // IDLE
{ { do_idle , IDLE } , { do_charge_mppt , CHARGE_MPPT }, { do_charge_topping , CHARGE_TOPPING }, { halt , IDLE } } ,     // CHARGE_MPPT
{ { do_idle , IDLE } , { do_charge_topping , CHARGE_TOPPING }, { do_nothing , CHARGE_TOPPING }, { halt , IDLE } } ,      // CHARGE_TOPPING
{ { do_idle , IDLE } , { do_charge_float , CHARGE_FLOAT }, { do_nothing , CHARGE_TOPPING }, { halt , IDLE } } ,          // CHARGE_FLOAT
};
void do_nothing( void ){
    printf("Nothing \n");
}

void do_idle ( void ){
    printf("Idle \n"); 
    pwm_off();
    switch_load(ON);
}
void do_charge_mppt ( void ){
    printf("Charge \n"); 
}
void do_charge_topping ( void ){
    printf("Charge \n"); 
}
void do_charge_float ( void ){
    printf("Charge \n"); 
}
void switch_load ( int state ){
    if (state) {
        // turn load On
    }
    else {
        // turn load Off
    }   
}
void pwm_off( void ){
    // Turn off PWM
}
void halt ( void ){
    pwm_off();
    switch_load(OFF);
}
void update_all( void ){
    // update inputs
    // update events
}



/* main.c */
int main(int argc, char *argv[]) {
    printf("Hello");    
}