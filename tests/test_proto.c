// Host unit test for proto.c:
//   gcc -Wall -Iinclude tests/test_proto.c src/proto.c src/valve_ctrl.c -o tp && ./tp
#include <stdio.h>
#include <string.h>
#include "proto.h"
#include "relay.h"
#include "UART_routines.h"

/* ---- stubs ---- */
static char out[4096]; static size_t outn;
static const char *in_p;
static uint8_t relays;
void transmitByte(uint8_t c){ if(outn<sizeof out-1){ out[outn++]=(char)c; out[outn]=0; } }
void transmitString_F(const char *s){ while(*s) transmitByte((uint8_t)*s++); }
int16_t uart_getc(void){ return (in_p && *in_p) ? (uint8_t)*in_p++ : -1; }
uint8_t uart_rx_overflow_take(void){ return 0; }
void relay_on(uint8_t i){ relays |= (uint8_t)(1u<<i); }
void relay_off(uint8_t i){ relays &= (uint8_t)~(1u<<i); }
uint8_t relay_mask(void){ return relays; }
uint8_t relay_count(void){ return 7; }
void hal_motor(motor_dir_t d){ (void)d; }
void hal_alarm_store(bool l){ (void)l; }
void hal_log_P(const char *m, const char *a){ log_P(m,a); }

static uint32_t now = 1000;
static proto_status_t st = { ST_OPEN, POS_OPEN, 0, FAULT_NONE, false };
static void send(const char *s){ outn=0; out[0]=0; in_p=s; proto_poll(now,&st); }
static void advance(uint32_t ms){ while(ms){ uint32_t d = ms>200?200:ms; now+=d; ms-=d; proto_tick(now,&st);} }
static int fails;
#define EXPECT(cond) do{ if(!(cond)){ printf("FAIL %d: %s\n  out=[%s]\n",__LINE__,#cond,out); fails++; } }while(0)

/* every $-frame in out must have a valid checksum */
static int frames_valid(void){
  char *p=out;
  while((p=strchr(p,'$'))){ uint8_t cs=0; char *q=p+1; while(*q && *q!='*') cs^=(uint8_t)*q++;
    if(*q!=0x2a) return 0;
    unsigned v; if(sscanf(q+1,"%2X",&v)!=1 || v!=cs) return 0; p=q; }
  return 1; }

int main(void){
  proto_init();
  outn=0; proto_send_hello(); EXPECT(strstr(out,"$HELLO,LEAKDET,") && frames_valid());
  EXPECT(!proto_link_ok());
  send("garbage # log line\r\n"); EXPECT(outn==0 && !proto_link_ok());
  send("$HB\r\n"); EXPECT(!strchr(out,0x24) && proto_link_ok());
  send("$GS\r\n"); EXPECT(strstr(out,"$ST,OPEN,OPEN,00,NONE,0,00,1*") && frames_valid());
  send("$hb*00\r\n"); EXPECT(strstr(out,"$ER,?,CS*"));
  send("$HB*0A\r\n"); EXPECT(outn==0);
  send("$rl,2,1,5\r"); EXPECT(strstr(out,"$OK,RL*") && (relays & 4) && frames_valid());
  send("$RL,0,1\n"); EXPECT(strstr(out,"$ER,RL,DENIED") && !(relays & 1));
  send("$RL,7,1\n"); EXPECT(strstr(out,"$ER,RL,ARG"));
  send("$RL,3,2\n"); EXPECT(strstr(out,"$ER,RL,ARG"));
  send("$RL,3,1,3601\n"); EXPECT(strstr(out,"$ER,RL,ARG"));
  send("$RL,3\n"); EXPECT(strstr(out,"$ER,RL,ARG"));
  send("$RL,3,1\n"); EXPECT(strstr(out,"$OK,RL") && (relays & 8));   /* no lease */
  send("$FOO\n"); EXPECT(strstr(out,"$ER,FOO,UNK"));
  send("$AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n"); EXPECT(strstr(out,"$ER,?,LEN"));
  send("$H\x01" "B\n"); EXPECT(strstr(out,"$ER,?,FMT"));
  send("xx$GS\n"); EXPECT(strstr(out,"$ST,"));   /* resync on '$' */
  /* lease expiry: relay 2 had 5 s lease; keep link alive */
  for(int i=0;i<8;i++){ advance(1000); send("$HB\n"); }
  EXPECT(!(relays & 4) && (relays & 8));
  /* OPEN */
  st.leak_mask=1; send("$OPEN\n"); EXPECT(strstr(out,"$ER,OPEN,WET") && !proto_take_open());
  st.leak_mask=0; send("$OPEN\n"); EXPECT(strstr(out,"$OK,OPEN") && proto_take_open() && !proto_take_open());
  send("$CLOSE\n"); EXPECT(strstr(out,"$OK,CLOSE") && proto_take_close());
  /* status on change */
  outn=0; advance(200); outn=0; st.state=ST_CLOSING; advance(200); EXPECT(strstr(out,"$ST,CLOSING"));
  /* periodic */
  outn=0; out[0]=0; advance(200); EXPECT(!strstr(out,"$ST")); outn=0; advance(2000); EXPECT(strstr(out,"$ST"));
  /* link loss: relay 3 (no lease) must go off */
  outn=0; advance(10200); EXPECT(!proto_link_ok() && !(relays & 8) && strstr(out,"LOST") && strstr(out,",0*"));
  /* checksum of a real ESP frame */
  { char f[32]; uint8_t cs=0; const char *b="RL,4,1,60"; for(const char*q=b;*q;q++) cs^=(uint8_t)*q;
    snprintf(f,sizeof f,"$%s*%02X\r\n",b,cs); send(f); EXPECT(strstr(out,"$OK,RL") && (relays & 16)); }
  printf(fails ? "%d FAILED\n" : "ALL PROTO TESTS PASSED\n", fails); return fails!=0; }
