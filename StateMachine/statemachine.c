#include <stdio.h>
#include <stdlib.h>

typedef enum { VBAT_LOW, VBAT_OK, VBAT_HIGH, VSOL_LOW, VSOL_OK, IBAT_LOW, MAX_EVENT } Event ;
typedef enum { START, IDLE , CHARGE_M, CHARGE_T, CHARGE_F, MAX_STATE } State ;
typedef enum { OFF, ON } Run_state; 
typedef void (* Action ) ( void) ;
typedef struct {
Action to_do ; // function pointer to current-state action
State next_state ; // next-state enumerator
} Table_Cell ;

const char* event_names[] = { "VBAT_LOW", "VBAT_OK", "VBAT_HIGH", "VSOL_LOW", "VSOL_OK", "IBAT_LOW"};
const char* state_names[] = { "START", "IDLE" , "CHARGE_M", "CHARGE_T", "CHARGE_F", "MAX_STATE" };

void do_nothing( void );
void pwm_off( void );
void charge_m ( void );
void charge_t ( void );
void charge_f ( void );
void update_all( void );
void load_on ( void );
void load_off ( void );

Table_Cell state_table [ MAX_STATE][ MAX_EVENT ] = {  
/*    [0] VBAT_LOW             [1] VBAT_OK             [2] VBAT_HIGH             [3] VSOL_LOW             [4] VSOL_OK                [5] IBAT_LOW     <--EVENTS |         STATES */  
{ { do_nothing , START }, {  load_off , IDLE }, { do_nothing , START }, { do_nothing , START }, { do_nothing , CHARGE_M }, { do_nothing , START } } ,       // START
{ { load_off , START }, { do_nothing , IDLE }, { do_nothing , IDLE }, { do_nothing , IDLE }, { do_nothing , CHARGE_M }, { do_nothing , IDLE } } ,           // IDLE
{ { do_nothing , CHARGE_M }, { load_on , CHARGE_M }, { do_nothing , CHARGE_T }, { pwm_off , IDLE }, { do_nothing , CHARGE_M }, { do_nothing , CHARGE_M} } , // CHARGE_M
{ { do_nothing , CHARGE_T }, { do_nothing , CHARGE_T  }, { do_nothing , CHARGE_T }, { pwm_off , IDLE }, { do_nothing , CHARGE_T }, { do_nothing , CHARGE_F } } ,// CHARGE_T
{ { do_nothing , CHARGE_M }, { do_nothing , CHARGE_F }, { do_nothing , CHARGE_F }, { pwm_off , IDLE }, { do_nothing , CHARGE_F }, { do_nothing , CHARGE_F } } , // CHARGE_F
};
/* main.c */
int main(int argc, char *argv[]) {
    Table_Cell state_cell ;
    Event current_event ;
    State current_state ;
    current_state = START ; // initial state    
    int input;
    while(1){
        scanf_s("%d", &input);
        printf("Current Event: %s \n", event_names[input]);
        printf("Current State: %s \n", state_names[current_state]);        
        state_cell = state_table [ current_state ][ input ];    
        state_cell . to_do () ; // execute the appropriate action
        current_state = state_cell . next_state ; // transition to the new state
        printf("Next State: %s \n", state_names[current_state]);
    };   
}
void pwm_off( void ){    
    // Turn off PWM
    printf("pwm_off\n"); 
}
void load_on ( void ){
    // Turn load on GPIO PIN
    printf("load_on\n"); 
}
void load_off ( void ){
    // Turn load off GPIO PIN
    printf("load_off\n"); 
}
void do_nothing( void ){
    printf("do_nothing\n");
}
void charge_m ( void ){
    printf("Charg_m \n"); 
}
void charge_t ( void ){
    printf("Charge_t \n"); 
}
void charge_f ( void ){
    printf("Charge_f \n"); 
}
void update_all( void ){
    // update inputs
    // update events
}