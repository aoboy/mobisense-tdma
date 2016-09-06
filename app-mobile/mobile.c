#include "contiki.h"
#include "contiki-net.h"

#include "dev/leds.h"
#include "net/rime/rimeaddr.h"
#include "net/rime/packetbuf.h"
#include "net/rime/broadcast.h"

#include "net/rime/network_l.h"

#include "node-id.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#include "../mobility.h"

static uint8_t val = 0;


PROCESS(mobile_packet_process, "Mobile Demo With Network_layer");
AUTOSTART_PROCESSES(&mobile_packet_process);

/*implements a counter,  the leds count from 0 to 7
 *param count - the value to be displayed of the leds
 */
static void leds_set(uint8_t count){

 //the shift is due to the change on the leds in the Tmote Sky platform
         if(count & LEDS_GREEN){
           leds_on(LEDS_GREEN<<1);
         }else{
            leds_off(LEDS_GREEN<<1);
         }

         if(count & LEDS_YELLOW){
           leds_on(LEDS_YELLOW>>1);
         }else{
           leds_off(LEDS_YELLOW>>1);
         }

         if(count & LEDS_RED){
          leds_on(LEDS_RED);
         }else{
          leds_off(LEDS_RED);
         }
}
/*----------------------------------------------------------------------------*/
static void inpacket(rimeaddr_t *from, void* data, uint8_t len){
    packet_t *p =(packet_t *)data;
    leds_set(p->seqno);   
}
/*----------------------------------------------------------------------------*/
PROCESS_THREAD(mobile_packet_process, ev, data)
{
      
   PROCESS_EXITHANDLER(network_close()); 

   PROCESS_BEGIN();
   static struct etimer et;
   #if RATE_CONTROL
   extern u8_t node_data_rate;
   PRINTF("RATE CONTROL IS ENABLED\n");  
   #else
   PRINTF("RATE CONTROL DISABLED\n");  
   #endif
   PRINTF("using RTIMER_SECOND = %u ticks\n", RTIMER_SECOND);
   
   static network_msg_t msg;
   
   /*You can change this up to 10 for a mobile node. If a cluster is empty
    or with only two mobile nodes, u can even request 16.*/
   static uint8_t datarate = 1;
   static rimeaddr_t addr, dest;
   addr.u8[0] = 3;
   addr.u8[1] = 0;

   dest.u8[0] = 1;
   
   /*We open the connection  for data transmission*/
   network_open();
 
   /*we set the callback function to receive incoming traffic*/
   network_recv_func(inpacket);
   /*We ask the MAC  layer to send a query to the cluster head, at what data 
    rate we want to send data.*/
   network_set_data_rate(datarate);
  
   etimer_set(&et, 10*CLOCK_SECOND);
   PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));                  

   while(1){                     
            #if RATE_CONTROL
            etimer_set(&et, CLOCK_SECOND/node_data_rate);
            #else
            etimer_set(&et, CLOCK_SECOND/datarate);
            #endif                        	    

            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));         
           
            packet_t *packet = (packet_t*)network_getbuffer(sizeof (packet_t));
            packet->seqno = val++;
            packet->type  = 2;               

            network_send(&dest);
   }

   PROCESS_END();
}

