// Host unit test for valve_ctrl.c (no AVR needed):
//   gcc -Wall -Iinclude tests/test_valve_ctrl.c src/valve_ctrl.c -o test && ./test
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "valve_ctrl.h"
static motor_dir_t hw_motor=MOTOR_STOP; static int ee=0;
void hal_motor(motor_dir_t d){ hw_motor=d; }
void hal_alarm_store(bool l){ ee=l; }
void hal_log_P(const char*m){ (void)m; /*printf("%s",m);*/ }
static uint32_t now=0; static ctrl_inputs_t in;
static void tick(void){ now+=200; in.now_ms=now; ctrl_step(&in); in.btn_reset=in.btn_close=false; }
static void ticks(int n){ while(n--) tick(); }
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s (state=%d motor=%d)\n",__LINE__,#c,ctrl_state(),ctrl_motor()); return 1;} }while(0)
int main(void){
  /* 1. boot, valve closed, no leak -> opens */
  ctrl_init(false); in=(ctrl_inputs_t){0,POS_CLOSED,0,0,0};
  ticks(5); CHECK(ctrl_state()==ST_OPENING && hw_motor==MOTOR_OPEN);
  in.pos=POS_UNKNOWN; ticks(10);
  /* 2. leak while opening mid-way -> reverses to close immediately */
  in.leak_mask=1; tick(); CHECK(ctrl_state()==ST_CLOSING && hw_motor==MOTOR_CLOSE && ctrl_alarm() && ee==1);
  in.pos=POS_CLOSED; tick(); CHECK(ctrl_state()==ST_ALARM && hw_motor==MOTOR_STOP);
  /* 3. leak clears -> stays closed (latched) */
  in.leak_mask=0; ticks(20); CHECK(ctrl_state()==ST_ALARM && hw_motor==MOTOR_STOP);
  /* 4. reset denied while wet */
  in.leak_mask=2; in.btn_reset=true; tick(); CHECK(ctrl_state()==ST_ALARM);
  in.leak_mask=0; in.btn_reset=true; tick(); CHECK(ctrl_state()==ST_OPENING && !ctrl_alarm() && ee==0);
  in.pos=POS_OPEN; tick(); CHECK(ctrl_state()==ST_OPEN && hw_motor==MOTOR_STOP);
  /* 5. closing timeout -> fault, one retry on leak, then stays fault with motor off */
  in.leak_mask=1; tick(); CHECK(ctrl_state()==ST_CLOSING);
  in.pos=POS_UNKNOWN; ticks(50); CHECK(ctrl_state()==ST_FAULT && hw_motor==MOTOR_STOP && ctrl_fault()==FAULT_CLOSE_TIMEOUT);
  tick(); CHECK(ctrl_state()==ST_CLOSING);   /* retry */
  ticks(50); CHECK(ctrl_state()==ST_FAULT && hw_motor==MOTOR_STOP);
  ticks(100); CHECK(ctrl_state()==ST_FAULT && hw_motor==MOTOR_STOP);  /* no endless retries */
  /* 6. power cycle with latched alarm and valve open -> closes */
  ctrl_init(true); hw_motor=MOTOR_STOP; in=(ctrl_inputs_t){0,POS_OPEN,0,0,now};
  ticks(5); CHECK(ctrl_state()==ST_CLOSING && hw_motor==MOTOR_CLOSE);
  /* 7. valve in middle at boot with leak -> closes (not treated as closed) */
  ctrl_init(false); hw_motor=MOTOR_STOP; in=(ctrl_inputs_t){1,POS_UNKNOWN,0,0,now};
  ticks(5); CHECK(ctrl_state()==ST_CLOSING);
  /* 8. manual close, then open by RESET */
  ctrl_init(false); in=(ctrl_inputs_t){0,POS_OPEN,0,0,now}; ticks(5); CHECK(ctrl_state()==ST_OPEN);
  in.btn_close=true; tick(); CHECK(ctrl_state()==ST_CLOSING); in.pos=POS_CLOSED; tick(); CHECK(ctrl_state()==ST_CLOSED && !ctrl_alarm());
  in.btn_reset=true; tick(); CHECK(ctrl_state()==ST_OPENING);
  /* 9. opening timeout -> fault, motor off */
  in.pos=POS_UNKNOWN; ticks(50); CHECK(ctrl_state()==ST_FAULT && ctrl_fault()==FAULT_OPEN_TIMEOUT && hw_motor==MOTOR_STOP);
  /* 10. both switches at boot -> fault */
  ctrl_init(false); in=(ctrl_inputs_t){0,POS_FAULT,0,0,now}; ticks(5); CHECK(ctrl_state()==ST_FAULT);
  in.btn_reset=true; in.pos=POS_OPEN; tick(); CHECK(ctrl_state()==ST_INIT); ticks(5); CHECK(ctrl_state()==ST_OPEN);
  /* 11. in ALARM someone opens valve by hand -> closes again */
  in.leak_mask=1; tick(); in.pos=POS_CLOSED; tick(); CHECK(ctrl_state()==ST_ALARM);
  in.pos=POS_UNKNOWN; tick(); CHECK(ctrl_state()==ST_CLOSING);
  printf("ALL TESTS PASSED\n"); return 0; }
