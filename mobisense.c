/**---------------------------------------------------------------
 *KTH - The Royal Institute of Technology - STOCKHOLM - Sweden
 *   School of Electrical Engineering
 *      Automatic Control Lab
 *----------------------------------------------------------------
 * Research Project in Mobile Wireless Sensor Networks
 * Supervisor: Prof. Mikael Johansson
 * Co-Supervisor: 
 *
 *----------------------------------------------------------------
 * Author: Antonio Gonga - gonga@kth.se
 * Date Created : June 3rd, 2009
 * Revision:      Jan 21st, 2010
 * Major Revision: Feb  12th, 2010,  04:19:45'AM
 * Version: 10.7
 *----------------------------------------------------------------
 *Mobile MAC aims to provide medium access layer mobility to WSN
 *nodes. This version is aimed to support uIP mobility.
 *----------------------------------------------------------------*/

#include "contiki.h"
#include "net/mac/mobile_mac.h"
#include "net/rime/packetbuf.h"
#include "sys/rtimer.h"
#include "net/rime.h"
#include "lib/random.h"
#include "dev/leds.h"
#include "dev/cc2420.h"
#include "node-id.h"

#include <string.h>
/**-------------------------------------------------------------*/

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#if WITH_UIP6
#define PRINTMACADDR(addr) PRINTF(" fe80:0000:0000:00000:02%02x:%02x%02x:%02x%02x:%02x%02x \n",  ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7])
#endif
#else
#define PRINTF(...)
#if WITH_UIP6
#define PRINTMACADDR(addr)
#endif
#endif

/**-------------------------------------------------------------*/
/**GENERAL DECLARATIONS
 *
 *
 */
/**-------------------------------------------------------------*/
#define TIME_DIFF(a,b)     ((signed short)((a)-(b)) < 0)


#if RIMEADDR_SIZE == 2
const rimeaddr_t sink_node_addr = { { 1, 0 } };
#endif
#if RIMEADDR_SIZE == 8
//const rimeaddr_t sink_node_addr ={{0, 18, 116, 0, 17, 125, 68, 149}};
const rimeaddr_t sink_node_addr ={{0, 18, 116, 1, 0, 1, 1, 1}};
#endif

#if WITH_UIP6
#ifndef NETWORK_PERFORMANCE
#define NETWORK_PERFORMANCE 160
#endif
#endif  /**WITH_UIP6*/
/**-------------------------------------------------------------*/
#define MAX_CHILDS   8
/*#if NODE_IS_SINK
#define MAX_CHILDS   8
#endif*/
/**-------------------------------------------------------------*/
/*#if NODE_IS_ROUTER
#define MAX_MOBILES  8
#define MAX_ROUTERS  1
#define MAX_CHILDS   (MAX_MOBILES )
#endif*/
/**-------------------------------------------------------------*/
/*#if NODE_IS_MOBILE
#define MAX_CHILDS   8
#endif*/
/**-------------------------------------------------------------*/
#define COMMON_CH   26

#ifdef CONF_SINK_CH
#define SINK_CH SINK_CONF_CH
#else  /*SINK_CONF_CH*/
#define SINK_CH 25
#endif /*SINK_CONF_CH*/
/**-------------------------------------------------------------*/
#define ONE_KILO  1024
#define ONE_MSEC  (RTIMER_SECOND/ONE_KILO)
/*64 clock ticks ~8.0ms*/
#define    TS     (8*ONE_MSEC) 
#define HALF_MSEC (ONE_MSEC/2)
#define GUARD_TIME  (TS/4)        /**~2.0ms*/


#if WITH_UIP6
#ifndef NETWORK_HDR_LEN
#define NETWORK_HDR_LEN 5
#endif  /**NETWORK_HDR_LEN*/
#define UIP_NTWK_PERFORMANCE 100
#endif /**WITH_UIP6*/

/**-------------------------------------------------------------*/
enum{
    SINK_DL_TX_TS               = (8*TS),        /**8TS=62.5ms*/
    SINK_UL_RX_TS               = (64*TS),       /**500ms*/
    ROUTER_UL_TX_TS             = (32*TS),        /**250ms*/
    HDVR_TX_TS                  = (4*TS),
    ROUTER_DL_TX_TS             = (SINK_DL_TX_TS),
    BW_ROUTER_TS                = ((ROUTER_UL_TX_TS/TS) - MAX_CHILDS),
    /**-------------------------------------------------------------*/
#if CONF_SUPER_FRAME_DURATION
    SUPER_FRAME_DURATION        = (TS*CONF_SUPER_FRAME_DURATION),
#else
    SUPER_FRAME_DURATION        = (SINK_DL_TX_TS + SINK_UL_RX_TS + HDVR_TX_TS),
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_MOBILE
    MOB_AWAKE_OFFSET_1          = (SINK_UL_RX_TS - GUARD_TIME -TS),
    MOB_AWAKE_OFFSET_2          = (ROUTER_UL_TX_TS - GUARD_TIME -TS),
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER || NODE_IS_MOBILE
    AWAKE_PERIOD                = ( SUPER_FRAME_DURATION - TS),
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_MOBILE
    UL_QUEUE_SIZE               = 25,
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER
    #if WITH_UIP6
    UL_QUEUE_SIZE               = 15,
    DL_QUEUE_SIZE               = 5,
    #else
    UL_QUEUE_SIZE               = 30,
    DL_QUEUE_SIZE               = 7,   
    #endif
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_SINK
    DL_QUEUE_SIZE               = 30,
#endif

}; /*end of enumeration*/
/**-------------------------------------------------------------*/
/**
 * 
 * message types between sensor nodes.*/
enum{  
    XPTO, 
  SINK_BEACON,          /**a sink beacon message*/  
  ROUTER_BEACON,        /**a router beacon message*/  
  MOBILE_SLOT_REQ,      /**a slot request from a mobile node to a router node*/
  ROUTER_SLOT_REQ,      /**a slot request from a router node to the sink node*/  
  NODE_SLOT_RELEASE,    /**mobile requests slot release... handover in progress*/
  NODE_STATUS_UPDATE,   /**similar to KEEP alive, sent every 5 frames*/
  NODE_UPDATE_ALL,      /**upadate list of childs*/
  HANDOVER_INFO,        /**router special info to mobile nodes*/
  DATA_RATE_UPDATE,     /**mobile node data rate update*/
};
/**-------------------------------------------------------------*/
enum{
  SINK_DATA  =100,          /*data originated or forwarded by the sink node*/
  ROUTER_DATA=101,          /*data originated or forarded by a router node*/
  MOBILE_DATA=102,          /*mobile node data*/  
};
enum{
  INACTIVE = 0,
  ACTIVE   = 1,
};

/**-------------------------------------------------------------*/
typedef enum{
    NOTHING,
    NODE_SLEEP,
    NODE_TX_DOWNLINK,
    NODE_TX_UPLINK,
    NODE_RX_DOWLINK,
    NODE_RX_UPLINK,
    NODE_RX_SYNCH,
    NODE_INFO_TX_RX,
    NODE_WRITE_BUF,
    NODE_RX_INFRA,
    NODE_TX_ADMIN_INFO,
    NODE_ROUTER_SCHEDULE,
}node_state_t;
/**-------------------------------------------------------------*/
//typedef u32_t my_clock_t;
typedef rtimer_clock_t my_clock_t;

#define NORM_CLOCK(a) ((u32_t)a%(65536UL))
static  uint8_t beacon_seqno  = 0;
/**-------------------------------------------------------------*/
static volatile node_state_t node_state    = NOTHING;
#if NODE_IS_ROUTER || NODE_IS_MOBILE
static volatile node_state_t ul_buf_state  = NOTHING;
#endif /*NODE_IS_ROUTER || NODE_IS_MOBILE*/
#if NODE_IS_SINK || NODE_IS_ROUTER
static volatile node_state_t dl_buf_state  = NOTHING;
#endif /*NODE_IS_SINK || NODE_IS_ROUTER*/
/**-------------------------------------------------------------*/
/**every echanged message, is precedeed by a header. This header gives
 *the type of message, and consequently how the message will be treated.
 */
typedef struct{
  uint8_t type;                      ///MSG_TYPE  DATA, CTRL_CH1,2,3 OR NOTHING
  rimeaddr_t src;                    ///WHERE THE MESSAGE COMES FROM
  rimeaddr_t dest;                   ///TO WHICH SENSOR THE MESSAGES IS DESTINATED TO
  uint8_t pld_len;                   ///PAYLOAD LENGTH ...
}rdc_hdr_t;
/**-------------------------------------------------------------*/
/**Synchronization packet(cluster head synchronization packet)*/
typedef struct synch{   
    uint8_t seqno;        /*sequence number*/
    uint8_t n_childs;     /*n_childs*/
    uint8_t free_slots;
    uint8_t n_routers;    /*n_routers*/
    uint8_t nr_hops;
    uint8_t ch_offset;     /*router channel offset*/
}synch_msg_t;

/*--------------------------------------------------------------*/
typedef struct{
   rdc_hdr_t hdr;
   uint8_t from;     /**type of a node where it came from*/
   uint8_t hops;     /*how many hops to the sink*/
   uint8_t n_nodes;  /* number of nodes including routers*/
   uint8_t ch_id;     /*channel id: at which channel this router is operating*/
   uint8_t group_id; /*explain later*/
}admin_info_t;

/**-------------------------------------------------------------*/
/*each slot request response message contains the requester id
 *and the respective offset.
 */
typedef struct slots{
   uint8_t    ts_offset;        /*slot Index/factor 0<= P1 < Num_neighs*/
   uint8_t    assig_slots;      /*assigned slots*/
   rimeaddr_t node_id;          /*nodeID*/
}slot_dist_t;
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER
typedef struct{
 uint8_t ttl;
 uint8_t state;
 rimeaddr_t node_id;
}mobile_state_t;
/**-------------------------------------------------------------*/
typedef enum{
   INSERT,
   LOOKUP,
   REMOVE,
}operation_t;
#define HDVR_LEN        6
mobile_state_t handoff_list[HDVR_LEN]; /*I hope that all mobiles will not evaporate at the same time :)*/
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_SINK
typedef struct{
    uint8_t odd_slots;
    uint8_t even_slots;
    uint8_t free_slots;
}sink_slot_dist_t;

sink_slot_dist_t sink_slots_tbl[] = {
    { 0,  0,  64}, /*0*/
    { 28, 29, 31}, /*1*/
    { 29, 29,  8}, /*2*/
    { 29, 30,  7}, /*3*/
    { 30, 30,  6}, /*4*/
    { 30, 31,  5}, /*5*/
    { 31, 31,  4}, /*6*/
    { 31, 32,  2}, /*7*/
    { 32, 32,  0}, /*8*/
};
#endif /*NODE_IS_SINK*/
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER || NODE_IS_MOBILE
/*handover table info. Each mobile node keeps this table in order to
 *select the best candidate to connect to in advent of power drop
 */
typedef struct{
    uint8_t state;
    uint8_t ch_id;
    signed char pdbm;
    #if NODE_IS_ROUTER
      uint8_t from;     /*type of a node where it came from*/
      uint8_t hops;     /*how many hops to the sink*/
    #endif /*end of NODE_IS_ROUTER*/
}neigh_conn_info_t;
static neigh_conn_info_t neigh_adv_list[MAX_CHILDS];
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_SINK || NODE_IS_ROUTER
/*the child list is used to distribute offsets as well as for routing data
 */
typedef struct node_list{
    uint8_t       ttl;
    uint8_t       type;         //node type
    uint8_t       state;
    uint8_t       data_rate;    //or share
    uint8_t       assign_slots;
    rimeaddr_t node_id;
}node_list_t;
static node_list_t node_list[MAX_CHILDS];
#endif  /*NODE_IS_SINK || NODE_IS_ROUTER*/
/**-------------------------------------------------------------*/
#if NODE_IS_SINK || NODE_IS_ROUTER || NODE_IS_MOBILE
typedef struct{ 
  u32_t   ad_t0;  /**admission request time*/
  u32_t   ad_tn;  /**admission response time_n*/
  u32_t   hv_t0;  /**slot_release time_0*/
  u32_t   hv_tn;  /**readimission response time*/   
  u32_t   tx_packets;  
  #if !WITH_UIP6
  uint8_t    rp_type;
  uint8_t    eff_d_rate;
  u32_t   nc_t0;  /**boot time*/
  u32_t   nc_tn;  /**last admission response time*/
  u32_t   tx_bytes;
  #endif
  rimeaddr_t c_router;    /**current router: is a router that a mobile node is attached to*/
  rimeaddr_t n_router;    /**next_router: is a router a mobile node moves, will then become the current router*/
}network_statistics_t;
#endif /*NODE_IS_ROUTER || NODE_IS_MOBILE*/
/**-------------------------------------------------------------*/
#define MIN_DATA_RATE           1
#define MAX_SYNCHR_BEACONS      128
#define SLOT_MSG_LEN   sizeof(slot_dist_t)
#define ADMIN_MSG_LEN  sizeof(admin_info_t)
#define SYNCH_MSG_LEN  sizeof(synch_msg_t)
#define RDC_HDR_LEN    sizeof(rdc_hdr_t)
/**-------------------------------------------------------------*/
typedef struct{
    rdc_hdr_t hdr;
    uint8_t data[PACKETBUF_SIZE - RDC_HDR_LEN];
}rdc_msg_t;
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER
#define TH_POWER                (-80)
#endif
#if NODE_IS_MOBILE
#define TH_POWER                (-80)
#endif
/**-------------------------------------------------------------*/
/**------------------radio specific-----------------------------*/
static volatile uint8_t radio_on      = 0;
static volatile uint8_t radio_off     = 0;

/**-------------------------------------------------------------*/
/**according to cc2420 DataSheet, PdBm = rssi+(-45)*/
#if NODE_IS_ROUTER || NODE_IS_MOBILE
/**At this received power, the node starts with the handover procedure.
 *A new candicate is selected from the handover table. The handover procedure
 *is similiar to the admission procedure. Once the candidate is selected, the
 *next step is to send a request slot message, and wait for request response
 *message.
 */
static const uint8_t data_channels[] = {25, 13, 18, 21, 20, 15, 22, 17, 12, 19, 14, 11, 16};

int16_t PWR_DBM(){
    int16_t recv_rssi = cc2420_last_rssi;
   return (int16_t) ((recv_rssi > 0x80) ?  (recv_rssi - 256 - 45) : (recv_rssi - 45));
}
#endif
/**-------------------------------------------------------------*/
static volatile uint8_t  cluster_size  = 0;
static struct rtimer generic_timer;
/**-------------------------------------------------------------*/
#if NODE_IS_SINK
static volatile  my_clock_t infra_rx_to;
static volatile  my_clock_t infra_rx_tn;
static volatile  my_clock_t synch_tx_to;
static volatile  uint8_t even_window = 0, odd_window = 0;
#endif /*NODE_IS_SINK*/
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER
static volatile  my_clock_t infra_rx_to;
static volatile  my_clock_t infra_rx_tn;
static volatile  my_clock_t hdvr_tx_to;
static volatile  my_clock_t ul_tx_sch; /*at which time UL TX will be scheduled*/
static volatile  my_clock_t radio_off_to; /*if there is no data to receive*/
//static my_clock_t node_awake_time;  /*a very complex paramenter..explained later*/

static volatile  uint8_t  sum_data_rates= 1;
static volatile  uint8_t  slots_offset  = 0;

#endif /*NODE_IS_ROUTER*/
/**-------------------------------------------------------------*/
#if NODE_IS_MOBILE
static volatile uint8_t   n_routers     = 0;
static volatile uint8_t   handover      = 0;
static volatile uint8_t   occupied_ts = 0;
/*duty_cycle specific*/
static my_clock_t hdvr_rx_to;
#if RATE_CONTROL 
extern uint8_t node_data_rate;
static void (* rate_update_func)( uint8_t len);
#endif /*RATE_CONTROL*/
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER || NODE_IS_MOBILE
static volatile my_clock_t ref_timer;
static my_clock_t radio_clock;
static volatile my_clock_t synch_rx_to;
/*uplink tx timer specific*/
static volatile my_clock_t ul_tx_to;
static volatile my_clock_t ul_tx_tn;
/*Evaluation variables */
static volatile  my_clock_t   ad_t0 = 0;  /*admission request time*/
static volatile  my_clock_t   ad_tn = 0;  /*admission response time_n*/
static volatile  my_clock_t   hv_t0 = 0;  /*slot_release time_0*/
static volatile  my_clock_t   hv_tn = 0;  /*readimission response time*/
static volatile  my_clock_t   nc_t0 = 0;  /*boot time*/
static volatile  my_clock_t   nc_tn = 0;  /*last admission response time*/

static uint8_t ass_d_rate    = 0;
static u32_t n_tx_bytes   = 0;
static u32_t n_tx_packets = 0;

static volatile  uint8_t is_there_data  = 0;

static volatile  uint8_t report_type=0;
static volatile  uint8_t ad_t0_lock = 1;
static volatile  uint8_t ad_tn_lock = 0;

static volatile  uint8_t hv_t0_lock = 0;
static volatile  uint8_t hv_tn_lock = 0;

static volatile  uint8_t nc_tn_lock = 1;
static volatile  uint8_t send_stats_pkt = 0;

static rimeaddr_t curr_router;
static rimeaddr_t next_router;
    /*admission specific*/
static volatile uint8_t atempts       = 0;
static volatile uint8_t is_scanning   = 0;
static volatile uint8_t admitted      = 0;
static uint8_t data_rate     = MIN_DATA_RATE;
static volatile uint8_t data_rate_update = 0;
static volatile uint8_t is_there_pkt  = 0;
   /*synch packet specific*/
static volatile uint8_t ch_offset     = 0;
static volatile uint8_t ts_offset     = 0;
static volatile uint8_t free_slots    = 0;
static volatile uint8_t assign_slots  = 0;

static uint8_t nr_hops       = 0;
static uint8_t num_neighbors = 0;
static uint8_t group_id         = 0;
/**radio channel specific*/
static volatile uint8_t uplink_ch     = 0;
/*uplink data buffer operations specific*/
static volatile uint8_t ul_b_full            = 0;
static volatile uint8_t next_to_send_ul      = 0;
static volatile uint8_t last_queued_ul       = 0;
static rdc_msg_t  *ul_queue[UL_QUEUE_SIZE];
static rdc_msg_t  ul_queue_bufs[UL_QUEUE_SIZE];
#endif /*NODE_IS_ROUTER || NODE_IS_MOBILE*/
/**-------------------------------------------------------------*/
#if NODE_IS_SINK || NODE_IS_ROUTER
static volatile  my_clock_t dl_tx_to;     /*downlink time to*/
static volatile  my_clock_t dl_tx_tn;     /*downlink time tn. tn-to =AT= AS*/
/*downlink buffers specific*/
static volatile uint8_t dl_b_full            = 0;
static volatile uint8_t next_to_send_dl      = 0;
static volatile uint8_t last_queued_dl       = 0;
static rdc_msg_t  *dl_queue[DL_QUEUE_SIZE];
static rdc_msg_t   dl_queue_bufs[DL_QUEUE_SIZE];
#endif /*NODE_IS_SINK || NODE_IS_ROUTER*/

/**-------------------------------------------------------------*/
static const struct radio_driver *radio;
static void (* receiver_callback)(const struct mac_driver *);
/*---------------------------------------------------------------------------*/
static void (*inpacket_callback )(rimeaddr_t *, void* data, uint8_t len);
/*---------------------------------------------------------------------------*/
#if WITH_UIP6
static void(*uip_callback_func)(rimeaddr_t *, rimeaddr_t *, void* data, uint8_t len);
#endif
/**-------------------------------------------------------------*/
static int turn_radio_off();
static int turn_radio_on();
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER || NODE_IS_MOBILE
static char uplink_transmitter(struct rtimer *t, void* ptr);

static char slot_request(struct rtimer *t, void* ptr);
#endif /*NODE_IS_ROUTER || NODE_IS_MOBILE*/
/**-------------------------------------------------------------*/
#if NODE_IS_SINK || NODE_IS_ROUTER
static char tx_admin_info(struct rtimer *t, void* ptr);
static char downlink_transmitter(struct rtimer *t, void* ptr);
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_MOBILE
static char mobile_power_cycle(struct rtimer *t, void* ptr);
#endif
#if NODE_IS_ROUTER
static char router_power_cycle(struct rtimer *t, void* ptr);
static uint8_t mobile_has_left(rimeaddr_t *addr, operation_t operation);
#endif
#if NODE_IS_SINK
static char sink_power_cycle(struct rtimer *t, void* ptr);
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER || NODE_IS_MOBILE
inline static void get_node_statistics(void *ptr){
    
  network_statistics_t *stat = (network_statistics_t*)ptr;     
   #if !WITH_UIP6
  stat->rp_type     = report_type;
  stat->nc_t0       = nc_t0;
  stat->nc_tn       = nc_tn;
  stat->tx_bytes    = n_tx_bytes;
  stat->eff_d_rate  = ass_d_rate; 
  #endif
  stat->ad_t0       = ad_t0;
  stat->ad_tn       = ad_tn;
  stat->hv_t0       = hv_t0;
  stat->hv_tn       = hv_tn;
  stat->tx_packets  = n_tx_packets; /*we also count the stats packet..*/
  rimeaddr_copy(&stat->c_router, &curr_router);
  rimeaddr_copy(&stat->n_router, &next_router);
}
#endif /*NODE_IS_ROUTER || NODE_IS_MOBILE*/
/**-------------------------------------------------------------*/
void set_hdr_type(rdc_hdr_t *hdr, uint8_t type){
    hdr->type = type;
}
/**-------------------------------------------------------------*/
uint8_t get_hdr_type(rdc_hdr_t *hdr){
    return hdr->type;
}
/**-------------------------------------------------------------*/

/*turns the radio on*/
static void on(void) {
    if (radio_on == 0) {
        radio_on = 1;
        radio->on();
    }
}
/**-------------------------------------------------------------*/
/*turns the radio off*/
static void off(void) {
    if (radio_on != 0) {
        radio_on = 0;
        radio->off();
    }
}

/**-------------------------------------------------------------*/
/*initializes the data buffers*/
void data_buffers_init() {
    uint8_t k;
    #if NODE_IS_SINK || NODE_IS_ROUTER
    for (k = 0; k < DL_QUEUE_SIZE; k++) {
  	dl_queue[k] = &dl_queue_bufs[k];
    }
    #endif

    #if NODE_IS_MOBILE || NODE_IS_ROUTER
    for (k = 0; k < UL_QUEUE_SIZE; k++) {    
        ul_queue[k] = &ul_queue_bufs[k];
    }
    #endif
}
/**-------------------------------------------------------------*/
#if NODE_IS_SINK || NODE_IS_ROUTER
/*initializes the child list*/
static inline void init_child_list() {
    uint8_t k;
    node_list_t *info;
    for(k = 0; k < MAX_CHILDS; k++){
        info = &node_list[k];
        memset(info, 0, sizeof(node_list_t));
    }
}
#endif
/**-------------------------------------------------------------*/
/*returts the childs list length*/
#if NODE_IS_SINK || NODE_IS_ROUTER
static uint8_t child_list_len() {
    uint8_t k, len = 0;
    node_list_t *l_ptr;
    for(k = 0; k < MAX_CHILDS; k++){
        l_ptr = &node_list[k];
        if(l_ptr->state == ACTIVE){
            len = len+1;
        }
    }    
    return len;
}
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_SINK || NODE_IS_ROUTER
/*adds a new entry to the childs list*/
inline static void child_add(rimeaddr_t *addr, uint8_t rate, uint8_t type) {
    uint8_t k,  exists = 0;
    uint8_t pos = MAX_CHILDS;
    node_list_t *nodes;        

    for (k = 0; k < MAX_CHILDS; k++) {
        nodes = &node_list[k];
        if(nodes->state == ACTIVE){
            if(rimeaddr_cmp(&nodes->node_id, addr)){                
                nodes->ttl   = 0;                                                
                exists       = 1;     
                //PRINTF("FOUND node %d.%d at pos:%d\n",addr->u8[0], addr->u8[1], k);          
                return;
            }
        }else{
            pos = k;            
        }
    }
    if(exists == 0 && pos < MAX_CHILDS){
        node_list[pos].ttl       = 0;
	node_list[pos].type      = type;
        node_list[pos].state     = ACTIVE;
        node_list[pos].data_rate = rate;
        rimeaddr_copy(&node_list[pos].node_id, addr);

	cluster_size = cluster_size + 1;
        //PRINTF("adding %d.%d at pos:%d\n",addr->u8[0], addr->u8[1], pos);

        #if NODE_IS_ROUTER
        data_rate_update = 1;      
        #endif
    }
}
/**-------------------------------------------------------------*/
/*adds a new entry to the childs list*/
inline static uint8_t is_there_packet_to(rimeaddr_t *addr){
    uint8_t k = 0, data_len = 0;
    rdc_msg_t *dlb;

    #if NODE_IS_ROUTER
    uint8_t  count = 0;
    k = next_to_send_dl ;
    #endif
   
    for(      ;  k < DL_QUEUE_SIZE; k++){
        dlb = dl_queue[k];
        data_len = dlb->hdr.pld_len;
        if(data_len){

            #if NODE_IS_SINK
            /*let the routers filter the packet.*/
            return 1;
            #endif
            
            #if NODE_IS_ROUTER
            count++;
            if(rimeaddr_cmp(addr, &dlb->hdr.dest) ||
                    rimeaddr_cmp(&dlb->hdr.dest, &rimeaddr_null)){
                return 1;
            }
            
            if(count >= 16){
                return 0;
            }
            #endif           

            data_len = 0;
        }
     
        if((k + 1) == last_queued_dl){
            return 0;
        }
      
    }
     return 0;
}
#endif /*NODE_IS_SINK || NODE_IS_ROUTER*/
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER || NODE_IS_MOBILE
/**-------------------------------------------------------------*/
/*initializes the handover table*/
inline void neigh_adv_list_init() {
    uint8_t k;
    neigh_conn_info_t *info_init;
    for(k = 0; k < MAX_CHILDS; k++){
        info_init = &neigh_adv_list[k];
        memset(info_init, 0, sizeof(neigh_conn_info_t));
    }
}
/**-------------------------------------------------------------*/
/*returns the handover table size/length*/
static uint8_t neigh_adv_list_len() {
    uint8_t k, len = 0;
    neigh_conn_info_t *info;
    for(k = 0; k < MAX_CHILDS; k++){
        info = &neigh_adv_list[k];
        if(info->state == ACTIVE){
            len = len +1;
        }
    }
    return len;
}
/**-------------------------------------------------------------*/
inline static void neigh_adv_list_best(){
    uint8_t k;
    uint8_t new_ch_id = 0, exist = 0;
    uint8_t best_hops = 100;
    signed char power=TH_POWER;
    neigh_conn_info_t *best;

    #if NODE_IS_ROUTER
	new_ch_id = neigh_adv_list[0].ch_id; //neigh_adv_list[0].ch_id;
	uplink_ch = data_channels[new_ch_id];
	return;
    #endif
    #if NODE_IS_MOBILE
    for(k = 0; k < MAX_CHILDS; k++){
        best = &neigh_adv_list[k];
        if(best->state == ACTIVE){
	   
            if(TIME_DIFF(power, best->pdbm)){
                new_ch_id = best->ch_id;
                power     = best->pdbm;
                best->pdbm=-94;
                exist     = 1;                
            }           	   
        }
    }

    if(exist){	
        uplink_ch  = data_channels[new_ch_id];        
        ch_offset  = new_ch_id;	
	PRINTF("ch_offset_gonga: %d\n", ch_offset);
    }
    #endif	
}
/**-------------------------------------------------------------*/
/*add a new entry to the handover table*/
static void parents_add(admin_info_t  *adv_msg) {
    uint8_t exists = 0, k;
    uint8_t pos = MAX_CHILDS;
    signed char pdbm = PWR_DBM();
    
    neigh_conn_info_t *info;    
    
    #if NODE_IS_ROUTER
    /*in this short version, am only interested to connect routers to the sink*/
    if(adv_msg->hops == 0){
	neigh_adv_list[0].state = ACTIVE;
        neigh_adv_list[0].ch_id = adv_msg->ch_id;
        neigh_adv_list[0].pdbm  = pdbm;
	neigh_adv_list[0].from  = adv_msg->from;
	neigh_adv_list[0].hops  = adv_msg->hops;  /*closer router */
	return;   
    }
    #endif
    
    #if NODE_IS_MOBILE
     if(adv_msg->hops == 0){ 
        return;
     }    

    for (k = 0; k < MAX_CHILDS; k++) {
        info = &neigh_adv_list[k];
        if(info->state == ACTIVE){
            
             if (info->ch_id == adv_msg->ch_id){
		
                 if ((adv_msg->n_nodes < MAX_CHILDS) && TIME_DIFF(TH_POWER, pdbm)) {
                     info->pdbm  = pdbm;  //update                     
                 }else{
                     info->ch_id = 0;
                     info->state = INACTIVE; //remove          
                 }
   
                 exists  = 1;
               
                return;
            }
        }else{
            pos     = k;            
        }
    }
        
    if (exists == 0 && pos < MAX_CHILDS) {
        if (TIME_DIFF(TH_POWER, pdbm) && (adv_msg->n_nodes < MAX_CHILDS)) {
            neigh_adv_list[pos].state = ACTIVE;
            neigh_adv_list[pos].ch_id = adv_msg->ch_id;
            neigh_adv_list[pos].pdbm  = pdbm;
	    
            //PRINTF("...adding ch_id: %d. power:%d pos: %d\n", ch_id, pdbm, pos);
        }
    }
   #endif /*NODE_IS_MOBILE*/
}
#endif 

/**-------------------------------------------------------------*/
#if NODE_IS_SINK || NODE_IS_ROUTER
/*a mobile node is about to initiate handover, so a slot release message is
 *received at the parent router node to remove thde node from the childs list
 */
static uint8_t child_remove(rimeaddr_t *addr) {
    uint8_t k;
    node_list_t *rm_ptr;
    for (k = 0; k < MAX_CHILDS; k++) {
        rm_ptr = &node_list[k];
        if (rm_ptr->state == ACTIVE) {
            if (rimeaddr_cmp(&rm_ptr->node_id, addr)) {                
                rm_ptr->state     = INACTIVE;
                rm_ptr->data_rate = 0;			                
                return 1;
            }
        }
    }
  return 0;
}
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_SINK || NODE_IS_ROUTER
inline static void child_data_rates(){
    node_list_t *child_list;

    #if NODE_IS_SINK
    uint8_t k, idx = 0;
    even_window = odd_window = 0;
    #endif

    #if NODE_IS_ROUTER
    uint8_t k, rates=0;
    
    uint8_t childs_len      = 0;
    uint8_t available_slots = 0;
    sum_data_rates       = data_rate;    
    #endif

    for (k = 0; k < MAX_CHILDS; k++) {
        child_list = &node_list[k];
        if (child_list->state == ACTIVE){
            #if NODE_IS_ROUTER
            rates = rates + child_list->data_rate;
            sum_data_rates = sum_data_rates + rates;
            childs_len     = childs_len + 1;
            #endif

            #if NODE_IS_SINK
            if(idx % 2 == 0){
                even_window  = even_window + child_list->data_rate;
            }else{
                odd_window   = odd_window + child_list->data_rate;
            }
            idx = idx + 1;
            #endif
        }
    }
    
    #if NODE_IS_ROUTER
    slots_offset      = 0;
    //cluster_size      = childs_len;
    available_slots   = (BW_ROUTER_TS-(MAX_CHILDS/2 - childs_len/2));
    
    for (k = 0; k < MAX_CHILDS; k++) {
        child_list = &node_list[k];
	
        if (child_list->state == ACTIVE){
	    uint8_t slots_per_node  = 0;
	    #if WITH_UIP6
            slots_per_node = (uint8_t)((59375*child_list->data_rate*available_slots)/(100000 *sum_data_rates));
	    #else
	    slots_per_node = (uint8_t)((child_list->data_rate*available_slots)/(sum_data_rates));
	    #endif
	     
            /**if the proportional chunk is higher than the requested data rate
             *fix the chuck to the data rate.*/             
            if(slots_per_node >= child_list->data_rate){
                slots_per_node = (child_list->data_rate/2) + (child_list->data_rate % 2 == 1 ? 1 : 0); 		
            }	  
	    	    	    
            if((slots_per_node == 0) ||(child_list->data_rate == MIN_DATA_RATE)){
                slots_per_node  = MIN_DATA_RATE;     
            }
	    
	    child_list->assign_slots = slots_per_node;
	    
            slots_offset = slots_offset + slots_per_node;
        }
    }
    #endif
}
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER
/*used to check if a specific node exists in our childs list.
 */
static uint8_t child_exist(rimeaddr_t *addr){
    uint8_t k;
    node_list_t *ex_ptr;
    for (k = 0; k < MAX_CHILDS; k++) {
        ex_ptr = &node_list[k];
        if (ex_ptr->state == ACTIVE){
            if( rimeaddr_cmp(&ex_ptr->node_id, addr)) {
                ex_ptr->ttl = 0;
                return 1;       
            }
        }
    }
    return 0;   
}
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER
/**used to check if a specific node exists in our childs list*/
static uint8_t mobile_has_left(rimeaddr_t *addr, operation_t operation){
    uint8_t k;
    mobile_state_t *mob_list;
    uint8_t exist = 0, pos = HDVR_LEN;
   
    for(k = 0; k < HDVR_LEN; k++){
	mob_list = &handoff_list[k];
	if(mob_list->state == ACTIVE){

	   mob_list->ttl = mob_list->ttl + 1;

	   if(mob_list->ttl >= 4 && operation == REMOVE){ 
		 mob_list->ttl = 0;
		 mob_list->state = INACTIVE;
		 return 1;
	   }
	   if(rimeaddr_cmp(&mob_list->node_id, addr)){
		exist = 1;					
		if(operation == LOOKUP){		
		    return 1;
		}		
	   }
	}else{
	   pos = k;	
	}
    }

    if((operation == INSERT) &&  (exist == 0 ) && (pos < HDVR_LEN)){
	handoff_list[pos].state = ACTIVE;
	handoff_list[pos].ttl   = 0;
	rimeaddr_copy(&handoff_list[pos].node_id, addr);
	return 1;
    }
   //PRINTF("there is no such mobile node \n");
   return 0; 
}
#endif
/**-------------------------------------------------------------*/
#if NODE_IS_ROUTER || NODE_IS_SINK
void child_set_state(rimeaddr_t *addr, uint8_t state, uint8_t rate) {
    uint8_t k;
    node_list_t *c_elem;

    for (k = 0; k < MAX_CHILDS; k++ ) {
        c_elem = &node_list[k];
	if(c_elem->state == ACTIVE){
	    //the node still alive, then update it
	    if(state == NODE_STATUS_UPDATE && rimeaddr_cmp(&c_elem->node_id, addr)) {
                c_elem->ttl       = 0;
                c_elem->data_rate = rate;             
            }

	    if(state == DATA_RATE_UPDATE && rimeaddr_cmp(&c_elem->node_id, addr)) {
                c_elem->ttl       = 0;
                c_elem->data_rate = rate;
                #if NODE_IS_ROUTER
                data_rate_update = 1; //newx round, send update data rate
                #endif
               
            }
	
	    if(state == NODE_UPDATE_ALL ){
		c_elem->ttl = c_elem->ttl + 1;
                if(c_elem->ttl >= 7) {
                    c_elem->state = INACTIVE;                   
                    /**if we have not heard from the node for 3 beacon periods, purge it*/                   
                }
            }
	} /**end of state == ACTIVE*/
    } /**end of the for loop*/
}
#endif
/**---------------------------------------------------------------------------*/
#if NODE_IS_MOBILE
/**Core procedure used for scanning the medium. It's mostly used by mobile sensor
 *nodes when accessing the network, or when performing the handover procedure.
 *This the procedure that sets the new candidate up. i.e, the new best router.
 */
static char mobile_chead_selection(struct rtimer *t, void *ptr){
  uint8_t ret;

    neigh_adv_list_best();
    if (uplink_ch >= 11 && uplink_ch < 26) {
        //PRINTF("Setting cc2420 Channel ID:%d\n", uplink_ch);
        cc2420_set_channel(uplink_ch);
        is_scanning = 0;               
        
        /**a new router has been selected, we can now turn the radio off
         till we request the admission*/
        off();  
         /**get the starting point of the admission rerquest phase*/
        if(ad_t0_lock){
             ad_t0 = RTIMER_NOW();   
             ad_t0_lock = 0;
             ad_tn_lock = 1;
             //PRINTF("mobile ad_t0: %2u \n", ad_t0);
        }
        
        synch_rx_to = RTIMER_NOW() + ROUTER_DL_TX_TS+2*GUARD_TIME;

        if (ch_offset %2 == 1) {
            synch_rx_to = synch_rx_to + ROUTER_UL_TX_TS;
        }
        
	synch_rx_to = NORM_CLOCK(synch_rx_to);
	      
        node_state = NODE_RX_SYNCH;
        ret = rtimer_set(&generic_timer, synch_rx_to, 1,
                (void (*)(struct rtimer *, void *))mobile_power_cycle, NULL);
        if (ret) {
                //PRINTF("error _1:%d\n", ret);
        }
        
        return 1;
    }

    if(is_scanning){
        if(handover){
            off();
            node_state = NODE_SLEEP;           
            hdvr_rx_to        = NORM_CLOCK(hdvr_rx_to + (SUPER_FRAME_DURATION - HDVR_TX_TS));;
            ret = rtimer_set(&generic_timer, hdvr_rx_to, 1,
                (void (*)(struct rtimer *, void *))mobile_power_cycle, NULL);
        }else{
            ret = rtimer_set(&generic_timer, RTIMER_NOW() + SUPER_FRAME_DURATION, 1,
                (void (*)(struct rtimer *, void *))mobile_chead_selection, NULL);
        }
        if (ret) {
            //PRINTF("error starting mobile entry: %u\n", RTIMER_NOW());
        }        
    }
  return 1;
}
/**---------------------------------------------------------------------------*/
/**One of the core procedures. Mobile part invoke this procedure recursivelly to
 *turn the radio on/off depending on next operation to be executed.
 */
static char mobile_power_cycle(struct rtimer *t, void* ptr) {
    int ret; 
 
    if(node_state == NODE_RX_UPLINK){
	    
	  if(is_there_data && TIME_DIFF((RTIMER_NOW()+2*GUARD_TIME), ul_tx_to)){
	           is_there_data = 0;	
		   node_state = NODE_RX_UPLINK;
	           /**we still have data to receive, so that we keep on waiting for more data to arrive*/		
		    ret = rtimer_set(&generic_timer, RTIMER_NOW()+TS, 1,
                	(void (*)(struct rtimer *, void *))mobile_power_cycle, NULL);     
  	    	    if (ret) {
               		PRINTF("mobile_power_cycle_error_there is data_1:%d\n", ret);
            	    }  	
	  }else{ 
		  node_state = NODE_TX_UPLINK;
		  off();
		  PRINTF("scheduling uplink Tx..\n");
		  ret = rtimer_set(&generic_timer, ul_tx_to, 1,
                 	(void (*)(struct rtimer *, void *))uplink_transmitter, NULL);     
  	    	  if (ret) {
               		PRINTF("mobile_power_cycle_error_there is data_2:%d\n", ret);
            	  }  	
	  }
	 
	return 1;
    }
    
    if(node_state == NODE_SLEEP){
        if (handover){
            /**a power drop has been detected, then a node should enter in the
              handover mode. Swicth the channel to 26 to listen to the
              nearby routers*/
            on();
            cc2420_set_channel(COMMON_CH);

            hdvr_rx_to = RTIMER_NOW();

            if(hv_t0_lock){
                /*collect some statistics*/
                hv_t0_lock    = 0;
                hv_tn_lock    = 1;
                ad_t0_lock    = 1;
                ad_tn_lock    = 1;  /**enables gathering the readmission time*/
                hv_t0 = RTIMER_NOW();
            }
        
            /**we have to go select the new chanel after listening 4TS*/
            ret = rtimer_set(&generic_timer, RTIMER_NOW() + HDVR_TX_TS, 1,
                (void (*)(struct rtimer *, void *))mobile_chead_selection, NULL);
            if (ret) {
                PRINTF("error T_TX_RX_OFF:%d\n", ret);
            }                
        }else{   
            /**if a node does not perform handover, then it should perform the 
            normal duty-cycling*/
	    if(cc2420_get_channel() != data_channels[ch_offset]){
	       cc2420_set_channel(data_channels[ch_offset]);           
	    }
	    node_state = NODE_RX_SYNCH;
            /**after NODE_SLEEPing, we should awake the node to resynchronize to
            its cluster head*/
            ret = rtimer_set(&generic_timer, synch_rx_to, 1,
                    (void (*)(struct rtimer *, void *))mobile_power_cycle, NULL);
            if (ret) {
                //PRINTF("Mobile_error awake_on:%d\n", ret);
            }                
        }
	return 1;
    } 
    
    if(node_state == NODE_RX_SYNCH){   //awake
        if (handover) {           
	   /**Whenever we finish one Schedule, we are not admitted, therefore, 
	    if there's not info about us, we should request admission again.**/
            admitted = 0;
            is_scanning = 1;
            handover = 0;
        }
        on();               
	node_state = NODE_RX_UPLINK;
	PRINTF("synch...\n");
    }

     return 1;
}
#endif
/**---------------------------------------------------------------------------*/
#if NODE_IS_SINK || NODE_IS_ROUTER
static char tx_admin_info(struct rtimer *t, void* ptr){
   int ret;    
   admin_info_t info;
   
      node_state = NODE_TX_ADMIN_INFO;
           
      //PRINTF("NODE_TX_ADMIN_INFO \n");
      /**we must switch to the common channel so as nodes in the admission phase can listen to us*/
      if(cc2420_get_channel() != COMMON_CH){
           cc2420_set_channel((uint8_t)COMMON_CH);
      }
      /**we set the type of message so that receivers can interprect it correctly*/
      set_hdr_type(&info.hdr, HANDOVER_INFO);
      /**we also send our address*/
      rimeaddr_copy(&info.hdr.src, &rimeaddr_node_addr);
        
      #if NODE_IS_SINK
      info.from    = 0;
      info.hops    = 0; /**how many hops (sink = root => hops = 0)*/
      info.ch_id   = 0; /**we inform in which channel_index the sink operates*/
      #else
      /**if a router had the radio shut, then turn it back on*/
      on();
      info.from    = 1;            /**the advertisement is coming from a router node*/
      info.hops    = (nr_hops+1);  /**how many hops are we far to the sink*/
      info.ch_id   = ch_offset;    /**we inform in which channel we operate*/
      #endif
      /**we inform how many routers are already attached to the sink*/
      info.n_nodes = child_list_len();
      /**we send this info to allow routers to connect to the sink node
	as well as mobile nodes to find the correct parent.*/
      radio->send(&info, ADMIN_MSG_LEN);             
      /**we make sure that we will wake up on the right channel*/
      cc2420_set_channel((uint8_t)SINK_CH);
      /**we change our state to NODE_TX_DOWNLINK to synchronize our cluster*/
      #if NODE_IS_SINK
       off();  /**We turn the radio off because it is not necessary by now.*/     
      node_state = NODE_TX_DOWNLINK;
           /**we go synchronize the cluster*/
      ret = rtimer_set(&generic_timer, synch_tx_to, 1,
            (void (*)(struct rtimer *, void *))downlink_transmitter, NULL);
      if (ret) {
          //PRINTF("error scheduling tx_infra to dl_tx:%d\n", ret);
      }
      #endif
      
      #if NODE_IS_ROUTER
      /**For router nodes, any ms of radio off is useful, so we NODE_SLEEP again*/
      if(TIME_DIFF((RTIMER_NOW()+2*GUARD_TIME), synch_rx_to)){
      	   off();
      }
      node_state = NODE_RX_SYNCH;
            /**we go synchronize to the cluster*/
      ret = rtimer_set(&generic_timer, synch_rx_to, 1,
               (void (*)(struct rtimer *, void *))router_power_cycle, NULL);     
      if (ret) {
         //PRINTF("error scheduling tx_infra to dl_tx:%d\n", ret);
      }  
      #endif
      return 1;
}
#endif
/**---------------------------------------------------------------------------*/
#if NODE_IS_SINK
static char sink_power_cycle(struct rtimer *t, void* ptr){
    int ret;
    if(node_state == NODE_TX_DOWNLINK){
	node_state = NODE_RX_INFRA;
	/**we turn the radio on to listen for incoming messages during (NODE_RX_INFRA_tn - NODE_RX_INFRA_to)*/
        on();
	/**we schedule when to send the advertisement message on the common channel*/
	ret = rtimer_set(&generic_timer, infra_rx_tn, 1,
                (void (*)(struct rtimer *, void *))tx_admin_info, NULL);
	if (ret) {
               //PRINTF("error scheduling tx_dl to NODE_RX_INFRA:%d\n", ret);
        }  
    } 
   return 1; 
}
#endif
/**---------------------------------------------------------------------------*/
#if NODE_IS_ROUTER
static char router_power_cycle(struct rtimer *t, void* ptr){
  int ret;   
            
	if(node_state == NODE_RX_UPLINK){
            
	    if(is_there_data && TIME_DIFF((RTIMER_NOW()+GUARD_TIME), radio_off_to)){
			   
		is_there_data = 0;
		/**we still have data to receive, so that we keep on waiting for more data to arrive*/		
		ret = rtimer_set(&generic_timer, (RTIMER_NOW()+TS/2), 1,
                	(void (*)(struct rtimer *, void *))router_power_cycle, NULL);     
  	    	if(ret) {
               		//PRINTF("router_power_cycle_error_there is data:%d\n", ret);
            	}  		    
		return 1;
            }else{		
		/**we should schedule the router node inmediately*/    
		off();
		if((ch_offset % 2) == 1){                    
                     //PRINTF("scheduling UL_TX ch_off:%d\n", ch_offset);                         
		     node_state = NODE_TX_UPLINK;
		     ret = rtimer_set(&generic_timer, ul_tx_to, 1,
                	(void (*)(struct rtimer *, void *))uplink_transmitter, NULL);   
  
  	    	     if (ret) {
               		//PRINTF("router_power_cycle_error_NODE_TX_UPLINK:%d\n", ret);
            	     }  
		}else{
		     //node_state = NODE_TX_DOWNLINK;
		                          
                     cc2420_set_channel(data_channels[ch_offset]);
                     
		     ret = rtimer_set(&generic_timer, dl_tx_to, 1,
                	(void (*)(struct rtimer *, void *))downlink_transmitter, NULL);
  	    	     if (ret) {
               		//PRINTF("router_power_cycle_error_NODE_TX_DOWNLINK:%d\n", ret);
            	     }                       
		}
	    } /**end of MTIMER_*/
            return 1;
	} /**end of node_state == NODE_RX_UPLINK*/
        
        if(node_state == NODE_TX_DOWNLINK){
            on();
            node_state = NODE_RX_INFRA;
            //PRINTF("TX_DL to NODE_RX_INFRA\n");
            ret = rtimer_set(&generic_timer, infra_rx_tn, 1,
                    (void (*)(struct rtimer *, void *))router_power_cycle, NULL);
            
            if (ret) {
                //PRINTF("router_power_cycle_error_tx_dl_to_NODE_RX_INFRA:%d\n", ret);
            }
            return 1;
        }
        
        if(node_state == NODE_RX_INFRA){
            /**we turn the radio off again*/                
            off();
            /**if the router is even-synchronized, do the uplink tx*/
            if (ch_offset % 2 == 0) {
                node_state = NODE_TX_UPLINK;
                
                ret = rtimer_set(&generic_timer, ul_tx_sch, 1,
                        (void (*)(struct rtimer *, void *))uplink_transmitter, NULL);
                if (ret) {
                    //PRINTF("error infra_to_uplink_tx:%d\n", ret);
                }
                //PRINTF("ROUTER: scheduling uplink_tx:%d \n",ret);
            }else{
                /**if the router is odd-synchronized, do Handover TX_INFO */
                node_state = NODE_TX_ADMIN_INFO;
                ret = rtimer_set(&generic_timer, hdvr_tx_to, 1,
                        (void (*)(struct rtimer *, void *))tx_admin_info, NULL);
                if (ret) {
                    //PRINTF("error infra_rx_to_NODE_TX_ADMIN_INFO: %d\n", ret);
                }
            }
            return 1;
        } /**end of node_state == NODE_RX_INFRA*/
    	
        if(node_state == NODE_RX_SYNCH){
            node_state = NODE_RX_UPLINK;
            /**we set the channel where th cluster head is operating in
             *and consequently, we turn the radio on to listen to*/                   
            on();   
            /**update bandwidth*/
            child_data_rates();  //make sure that sum(rate[i]) is computed before
            return 1;
        } /*end of node_state == NODE_RX_UPLINK*/

        if(node_state == NOTHING){ 
	   /**we have boot quite recently, and therefore, we must select a parent*/            
	   neigh_adv_list_best();
	   if((uplink_ch >= 11) && (uplink_ch <= 26)){
		/*we have found a parent node. First we change our state*/
		node_state = NODE_RX_SYNCH;
		
                cc2420_set_channel(uplink_ch); /*we set our channel to the parent channel*/
                
                /*any ms of saved energy is useful :)*/
		off();	             
	   }           
           ret = rtimer_set(&generic_timer, ((node_state == NODE_RX_SYNCH) ? RTIMER_NOW(): RTIMER_NOW()+ROUTER_DL_TX_TS), 1,
                   (void (*)(struct rtimer *, void *))router_power_cycle, NULL);
           if (ret) {
               //PRINTF("error scheduling tx_infra to dl_tx:%d\n", ret);
           }
	} /*end of node_state == NOTHING*/         

  return 1;
}
#endif

/**---------------------------------------------------------------------------*/
#if NODE_IS_ROUTER || NODE_IS_MOBILE
uint8_t mobile_node_data_rate(void){
    return data_rate;
}
#endif
/**---------------------------------------------------------------------------*/
#if NODE_IS_ROUTER ||  NODE_IS_MOBILE
static char slot_request(struct rtimer *t, void* ptr) {
    uint8_t ret;
    rdc_hdr_t hdr;
   
    //PRINTF(" slot request\n");
    #if NODE_IS_ROUTER
    set_hdr_type(&hdr, ROUTER_SLOT_REQ);
    hdr.pld_len = data_rate; //MIN_DATA_RATE;
    ad_t0 = RTIMER_NOW();   
    ad_t0_lock = 0;
    ad_tn_lock = 1;
    //PRINTF("router ad_t0: %2u \n", ad_t0);
    #endif

    #if NODE_IS_MOBILE
    set_hdr_type(&hdr, MOBILE_SLOT_REQ);
    hdr.pld_len = data_rate;
    #endif
    
    rimeaddr_copy(&hdr.src, &rimeaddr_node_addr);
    rimeaddr_copy(&hdr.dest, &rimeaddr_null);       
        
    radio->send(&hdr, RDC_HDR_LEN);

   #if NODE_IS_MOBILE
    node_state = NODE_RX_SYNCH;
    ret = rtimer_set(&generic_timer, (synch_rx_to-TS/2), 1,
            (void (*)(struct rtimer *, void *))mobile_power_cycle, NULL);
    if (ret) {
        //PRINTF("error MOB_AWAKE_OFFSET_1:%d\n", ret);
    }    
    #endif

    #if NODE_IS_ROUTER
    node_state = NODE_RX_SYNCH;
    ret = rtimer_set(&generic_timer, synch_rx_to-GUARD_TIME, 1,
            (void (*)(struct rtimer *, void *))router_power_cycle, NULL);
    if (ret) {
        //PRINTF("error MOB_AWAKE_OFFSET_1:%d\n", ret);
    }
    #endif

    off();

   return 1;

}
#endif

/*-----------------------------------------------------------------------------*/
#if NODE_IS_ROUTER || NODE_IS_MOBILE
/*Generic Uplink TX function. This function is Is used by routers and mobile
 * nodes to send data upstream. This function is also part of the duty-cycling*/
static char uplink_transmitter(struct rtimer *t, void* ptr) {
  uint8_t ret;

    if (TIME_DIFF((RTIMER_NOW()), ul_tx_to)) {
        off();
        node_state = NODE_SLEEP;
        ret = rtimer_set(&generic_timer, ul_tx_to, 1,
                    (void (*)(struct rtimer *, void *))uplink_transmitter, NULL);
        if (ret) {
            //PRINTF("TX_UL_error awake_on:%d\n", ret);
        }
        return 1;
    }

    on();

    node_state = NODE_TX_UPLINK;

#if NODE_IS_MOBILE
     PRINTF("transmitting UL\n");
    if (handover) {
        rdc_hdr_t c_hdr;

        c_hdr.pld_len = 0;       
	set_hdr_type(&c_hdr, NODE_SLOT_RELEASE);
	rimeaddr_copy(&c_hdr.src, &rimeaddr_node_addr);      
                
        /*if(hv_t0_lock){
            //collect some statistics
            hv_t0_lock    = 0;  
            hv_tn_lock    = 1;            
            ad_t0_lock    = 1;
            ad_tn_lock    = 1;  //enables gathering the readmission time
            hv_t0 = RTIMER_NOW();
        }*/
         
        radio->send(&c_hdr, RDC_HDR_LEN);
    } else {
        if(data_rate_update){
            rdc_hdr_t c_hdr;
	    data_rate_update = 0;	    
            c_hdr.pld_len = data_rate;
            set_hdr_type(&c_hdr, DATA_RATE_UPDATE);
            rimeaddr_copy(&c_hdr.src, &rimeaddr_node_addr);
	    
            radio->send(&c_hdr, RDC_HDR_LEN);
        }else{
            /*a kind of keep-alive message :)*/
            if (beacon_seqno % 5 == 0) {
                rdc_hdr_t c_hdr;
                c_hdr.pld_len = data_rate;
		set_hdr_type(&c_hdr, NODE_STATUS_UPDATE);
                rimeaddr_copy(&c_hdr.src, &rimeaddr_node_addr);
		
                radio->send(&c_hdr, RDC_HDR_LEN);
            }
        }
    }
#endif

#if NODE_IS_ROUTER
    if (uplink_ch != SINK_CH || (cc2420_get_channel() != SINK_CH)) {
        cc2420_set_channel((uint8_t)SINK_CH);
        uplink_ch = SINK_CH;
    }

    if(data_rate_update) {
        child_data_rates();  /*bandwidth update ...*/
        rdc_hdr_t c_hdr;
        c_hdr.pld_len = /**data_rate +*/ sum_data_rates;
        data_rate_update = 0;
	set_hdr_type(&c_hdr, DATA_RATE_UPDATE);
        rimeaddr_copy(&c_hdr.src, &rimeaddr_node_addr);
        radio->send(&c_hdr, RDC_HDR_LEN);
    }
#endif

    #if NODE_IS_ROUTER || NODE_IS_MOBILE
        if(send_stats_pkt){  
                uint8_t dlen = 0;
                rdc_msg_t pack;
                send_stats_pkt = 0;                                        
               		
		rimeaddr_copy(&pack.hdr.dest,  &sink_node_addr); 		
                rimeaddr_copy(&pack.hdr.src,   &rimeaddr_node_addr);
                
                
                /**we increment the number of packets originated on this node.*/
		 
		  #if NODE_IS_MOBILE
		  set_hdr_type(&pack.hdr, MOBILE_DATA);
		  #else		 
		  set_hdr_type(&pack.hdr, ROUTER_DATA);
		  #endif

		#if !WITH_UIP6		  
		  network_hdr_t n_hdr;
		  /*5 refers to the info header*/
		  dlen  = (5 + sizeof(network_statistics_t));
		  pack.hdr.pld_len = dlen;
                                               
		  n_hdr.seqno     = 0;
		  n_hdr.hops      = 1;
		  n_hdr.pack_type = NETWORK_PERFORMANCE;  /*network performance*/
		  #if NODE_IS_MOBILE
		  n_hdr.node_type = MOBILE_NODE;
		  #else		 
		  n_hdr.node_type = ROUTER_NODE;
		  #endif
		  memcpy((void*)&pack.data[0], (void*)&n_hdr, NETWORK_HDR_LEN);		   
		  n_tx_bytes += (dlen + RDC_HDR_LEN); /*number of bytes*/
		  /*we need to collect some results, so we ask the mac to do so*/
		  get_node_statistics((void*)&pack.data[NETWORK_HDR_LEN]);
                
                 /*we ask the radio driver to send the packet*/
		  radio->send((void*)&pack, (dlen + RDC_HDR_LEN));
		#else
		  PRINTF("sending statistics packet...TO:");		
		  PRINTMACADDR(&sink_node_addr);
		  pack.data[0]  = UIP_NTWK_PERFORMANCE;
		  dlen  	= (1 + sizeof(network_statistics_t)); 
		  pack.hdr.pld_len = dlen;
		  get_node_statistics((void*)&pack.data[1]);
		  
                 /*we ask the radio driver to send the packet*/
		  radio->send((void*)&pack, (dlen + RDC_HDR_LEN));
		#endif /**WITH_UIP6*/
               

		
        }     
    #endif
            
  
    while (((next_to_send_ul != last_queued_ul) && (ul_b_full == 0)) && TIME_DIFF((RTIMER_NOW() + GUARD_TIME), ul_tx_tn)) {
   
        uint8_t pkt_len = ul_queue[next_to_send_ul]->hdr.pld_len;
        if(pkt_len){                 
            /**We ask the radio driver to send the packet, and we free the packet
             only when we have succeeded on transmitting that packet*/
            radio->send((void*)ul_queue[next_to_send_ul], (pkt_len + RDC_HDR_LEN)); //RADIO_TX_OK
            ul_queue[next_to_send_ul]->hdr.pld_len  = 0;            
            /*if(radio->send((void*)ul_queue[next_to_send_ul], (pkt_len + RDC_HDR_LEN))  == RADIO_TX_OK){
                ul_queue[next_to_send_ul]->hdr.pld_len  = 0;
            }*/
        }  

        if(++next_to_send_ul >= UL_QUEUE_SIZE){
	    next_to_send_ul  = 0;
	}
        
        if(ul_b_full){
            ul_b_full = 0;
        }
   
    }  /**end of the while loop*/
  

    if(ul_b_full){
        ul_b_full = 0;
    }

#if NODE_IS_MOBILE
    off();
    PRINTF("tx_ul_finished, back mob_pc\n");
    node_state = NODE_SLEEP;
    ret = rtimer_set(&generic_timer, hdvr_rx_to, 1,
            (void (*)(struct rtimer *, void *))mobile_power_cycle, NULL);
    if (ret) {
        PRINTF("error NODE_TX_UPLINK_MOB_AWAKE_OFFSET_1:%d\n", ret);
    }
#endif


 #if NODE_IS_ROUTER
    if(TIME_DIFF((RTIMER_NOW()), hdvr_tx_to)){
         off();         
    }

    if (ch_offset % 2 == 1) {
        node_state = NODE_TX_DOWNLINK;
        ret = rtimer_set(&generic_timer, dl_tx_to, 1,
                (void (*)(struct rtimer *, void *))downlink_transmitter, NULL);
        if (ret) {
            //PRINTF("error downlink_tx:%d\n", ret);
        }       
    }else{
        node_state = NODE_TX_ADMIN_INFO;
        ret = rtimer_set(&generic_timer, hdvr_tx_to, 1,
                (void (*)(struct rtimer *, void *))tx_admin_info, NULL);
        if (ret) {
            //PRINTF("error shceduling hdvr_tx: %d\n", ret);
        }        
    }
 #endif

    return 1;
}
#endif   /** NODE_IS_ROUTER || NODE_IS_MOBILE init*/
/*-----------------------------------------------------------------------------*/

#if NODE_IS_ROUTER || NODE_IS_SINK
static char downlink_transmitter(struct rtimer *t, void *ptr) {
    uint8_t* b_ptr;
    uint8_t ret;
    uint8_t cluster_len = 0;
    uint8_t ass_slots   = 0;
    uint8_t admin_slots = 0;
    rdc_msg_t packet;
    synch_msg_t beacon;
    uint8_t data_len, k, idx=0;

    node_state = NODE_TX_DOWNLINK; //set the state we are
   
    
    //PRINTF("DL-Transmitter Radio is on, and state is NODE_TX_DOWNLINK\n");
    
    #if NODE_IS_ROUTER
    uint8_t nodes_tx_offset = 0;
    uint8_t available_slots;  //total available slots during round i
    #endif

    #if NODE_IS_SINK
    my_clock_t ref_to = RTIMER_NOW(); 
    synch_tx_to   = NORM_CLOCK(ref_to + SUPER_FRAME_DURATION); /*to+76TS*/
    infra_rx_to   = NORM_CLOCK(ref_to + (SINK_DL_TX_TS));        /*to+8TS*/
    infra_rx_tn   = NORM_CLOCK(ref_to + SINK_DL_TX_TS + SINK_UL_RX_TS);     /*to+8TS+64TS*/
    dl_tx_tn      = infra_rx_to; 
        
    uint8_t odd_slots;   //odd_part slots
    uint8_t even_slots;  //even_part slots
    uint8_t odd_offset  = 0;   //offset between neighbors
    uint8_t even_offset = 0;   //offset between neighbors
    #endif

    node_list_t *nodes;        
    
    on();

    //cluster_size = 0;
    cluster_len  = child_list_len();
    //cluster_size = cluster_len;
    //PRINTF("Cluster size: %d\n", cluster_size);
    /**set the source address*/
    rimeaddr_copy(&packet.hdr.src, &rimeaddr_node_addr);

    /**set the destination address(broadcast)*/
    rimeaddr_copy(&packet.hdr.dest, &rimeaddr_null);

    /**send the sequence number and number of neighbors.*/
    beacon.n_childs = cluster_len;
    beacon.seqno    = beacon_seqno;
    

    #if NODE_IS_SINK

        set_hdr_type(&packet.hdr, SINK_BEACON);

	child_data_rates();  //make sure that sum(BW[i]) is computed before
        admin_slots =  sink_slots_tbl[cluster_len].free_slots;
        odd_slots   =  sink_slots_tbl[cluster_len].odd_slots;
        even_slots  =  sink_slots_tbl[cluster_len].even_slots;

	beacon.nr_hops = 0;  /*the root must advertise 0 hops!*/
    #endif

    #if NODE_IS_ROUTER  /**switch RF channel*/
        if((beacon_seqno % 13 == 0) && (cluster_size != 0)){
            child_set_state(NULL, NODE_UPDATE_ALL, 0);
        }
	/*a Mobile nodes that has left our cluster will still be able 
          to receive data for a short while*/	
	mobile_has_left(&rimeaddr_node_addr, REMOVE);	
	/*we turn the radio on*/                
	/*we make sure that we are on the right channel*/    
  	cc2420_set_channel(data_channels[ch_offset]);
	/*we make sure that we send the right message*/
        set_hdr_type(&packet.hdr,  ROUTER_BEACON);

        beacon.n_routers  = num_neighbors;
        beacon.ch_offset  = ch_offset;
	beacon.nr_hops    = (nr_hops +1);         
        PRINTF("ch_offset: %d, ch_len: %d\n", ch_offset, cluster_size);
        admin_slots       = (MAX_CHILDS/4 - cluster_size/4); 
        available_slots   = (BW_ROUTER_TS - admin_slots);                                               
    #endif
  
    
    beacon.free_slots  = admin_slots;
    packet.hdr.pld_len = SYNCH_MSG_LEN + cluster_len*SLOT_MSG_LEN;

    beacon_seqno++;
    
     
     /**copy the cluster info into the synchronization packet*/
    memcpy(&packet.data[0], &beacon, SYNCH_MSG_LEN);
    
    //PRINTF("Cluster_size: %d\n", cluster_size);
    /*fill the payload, but first acquire the first byte of the payload*/
    b_ptr = (uint8_t*)&packet.data[SYNCH_MSG_LEN];
    
    //PRINTF("cluster len:%d\n",cluster_len);
    /**fill the node's schdeule into the synchronization packet*/
    for (k = 0, idx = 0;  k < MAX_CHILDS  ; k++) {
        nodes = &node_list[k];
        
        if (nodes->state == ACTIVE) {
            slot_dist_t *msg = (slot_dist_t*) (b_ptr + idx * SLOT_MSG_LEN);
                            
            #if NODE_IS_SINK
            uint8_t flag = 0;
            uint8_t mask = 0;
            ass_slots = MIN_DATA_RATE;  /*set default data rate*/            
            if(idx % 2 == 0){
                if((even_window != 0) && nodes->data_rate != MIN_DATA_RATE){                    
                    ass_slots = (nodes->data_rate*even_slots)/even_window;
                }

                mask = (mask | (even_offset & 0x1F));                
                mask = (mask | ((idx+1) << 5)); /*channel ID*/		                            

                msg->ts_offset   = mask;

                mask = 0;
                flag = is_there_packet_to(&nodes->node_id);
                mask = (mask | ( ass_slots & 0x1F));
                mask = (mask | ((flag & 0x01) << 5));
                msg->assig_slots = mask;
               
                even_offset = even_offset + ass_slots;
                rimeaddr_copy(&msg->node_id, &nodes->node_id);
                
                idx++;                                                       
            }else{
                
                if((odd_window != 0) && nodes->data_rate != MIN_DATA_RATE){
                     ass_slots = (nodes->data_rate*odd_slots)/odd_window;
                }

                mask = (mask | (odd_offset & 0x1F));
                mask = (mask | ((idx+1) << 5)); /*channel ID*/
		//mask = (mask | ((MAX_CHILDS-k) << 5)); /*channel ID*/                                                            

                msg->ts_offset   = mask;

                mask = 0;
                //flag = is_there_packet_to(&nodes->node_id);
                mask = (mask | ( ass_slots & 0x1F));
                mask = (mask | ((flag & 0x01) << 5));
                msg->assig_slots = mask;
                //msg->assig_slots = ass_slots;
                odd_offset = odd_offset + ass_slots;
                rimeaddr_copy(&msg->node_id, &nodes->node_id);
                idx++;
            }            
            #endif

            #if NODE_IS_ROUTER
              uint8_t flag = 0;
              uint8_t mask = 0;
              ass_slots = MIN_DATA_RATE;
              //flag = is_there_packet_to(&nodes->node_id);
              /**compute the share for each mobile node based on the rate/weight */           	  
           
              mask = (mask | (nodes_tx_offset & 0x1F));
              mask = (mask | ((flag & 0x01) << 5));  /*packet tag*/

              msg->ts_offset   = mask;           
              msg->assig_slots = nodes->assign_slots;                       
              rimeaddr_copy(&msg->node_id, &nodes->node_id);
	      nodes_tx_offset = nodes_tx_offset + nodes->assign_slots;    
	      idx++;                   
            #endif           
        }
    }
    
    #if NODE_IS_ROUTER
    //PRINTF("nodes_tx_offset: %d\n", nodes_tx_offset);
    #endif
   
    /*calculate how big is the synchronization packet*/
    data_len = RDC_HDR_LEN + packet.hdr.pld_len;


    /**send synchronization packet.*/
    radio->send((void*)&packet, data_len); 
    
    //PRINTF("Beacon transmitted\n");
    data_len = 0;

    /**Flush data downstream as fast as possible, and terminate when the buffer
     is empty of the downlink time has finished*/     
    while ((next_to_send_dl != last_queued_dl) && TIME_DIFF(RTIMER_NOW()+GUARD_TIME, dl_tx_tn)){
        data_len = dl_queue[next_to_send_dl]->hdr.pld_len;
        if(data_len){           
            /*ask the radio driver to send the packet */
            radio->send((void*)dl_queue[next_to_send_dl], (data_len + RDC_HDR_LEN));
            
            dl_queue[next_to_send_dl]->hdr.pld_len = 0;

        }else{
	   break;
	}
        /*we loop to the next packet*/
        next_to_send_dl = (next_to_send_dl + 1) % DL_QUEUE_SIZE;
        /*if the buffer was locked, unlock it so that it can accept data from the application layer*/
        if(dl_b_full){
            dl_b_full = 0;
        }
    }

    /*if the buffer was locked, unlock it so that it can accept <gonga> data from the application layer*/
    if(dl_b_full){
            dl_b_full = 0;
    }

    if(TIME_DIFF(RTIMER_NOW(), dl_tx_tn)){
          off();
	  //PRINTF("radio is off\n");
    }
        
    #if NODE_IS_SINK
    /*the sink enters in the recption mode, and schedule a new synch packed*/    
    node_state = NODE_TX_DOWNLINK;
    ret = rtimer_set(&generic_timer, infra_rx_to, 1,
            (void (*)(struct rtimer *, void *))sink_power_cycle, NULL);
    if (ret) {
        //PRINTF("send periodic beacon error #2: %d\n", ret);
    }
    #endif

    #if NODE_IS_ROUTER
    node_state = NODE_TX_DOWNLINK; //set the state we are	 node_state = NODE_RX_INFRA;
    ret = rtimer_set(&generic_timer, infra_rx_to, 1,
            (void (*)(struct rtimer *, void *))router_power_cycle, NULL);
    if (ret) {
        //PRINTF("error_tx_dl_to_infra_rx:%d\n", ret);
    }
    #endif
    
    return 1;
}
#endif
/**---------------------------------------------------------------------------*/
inline static void create_rdc_packet(rdc_msg_t* msg, rimeaddr_t *dest,
             u8_t uplink, u8_t data_len, u8_t type){

     if(uplink){
         #if NODE_IS_ROUTER || NODE_IS_MOBILE
         msg = ul_queue[last_queued_ul];
         #endif
     }else{
         #if NODE_IS_ROUTER || NODE_IS_SINK
         msg = dl_queue[last_queued_dl];
         #endif
     }

     msg->hdr.type     = type;
     msg->hdr.pld_len  = data_len;
     rimeaddr_copy(&msg->hdr.src, &rimeaddr_node_addr);
     rimeaddr_copy(&msg->hdr.dest, dest);
     memcpy((char*)msg->data, packetbuf_dataptr(), msg->hdr.pld_len);


    if(uplink){
       #if NODE_IS_ROUTER || NODE_IS_MOBILE
       last_queued_ul = (last_queued_ul + 1) % UL_QUEUE_SIZE;
       if(last_queued_ul == next_to_send_ul){
           ul_b_full = 1;
       }
       #endif
    }else{
       #if NODE_IS_ROUTER || NODE_IS_SINK
       last_queued_dl = (last_queued_dl + 1) % DL_QUEUE_SIZE;
       if(last_queued_dl == next_to_send_dl){
            dl_b_full   = 1;
       }
       #endif
    }
}
/**---------------------------------------------------------------------------*/
static int packet_out(void) { 
    
    uint8_t data_len;
    rimeaddr_t dest;
    rdc_msg_t *msg;
  
    data_len  = packetbuf_datalen();
    rimeaddr_copy(&dest, packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
    
    #if !WITH_UIP6
    #if NODE_IS_ROUTER || NODE_IS_MOBILE
    n_tx_packets++; /*The number of packets originated on this node*/
    n_tx_bytes += (data_len + RDC_HDR_LEN); /*number of bytes*/
    if(n_tx_packets % 100 == 0){
      report_type = 3;
      send_stats_pkt = 1;
    }
    #endif
    #endif /*!WITH_UIP6*/
        
    #if NODE_IS_ROUTER
    
    if (rimeaddr_cmp(&dest, &rimeaddr_null) || child_exist(&dest)) {
        if (dl_b_full == 0 && (dl_buf_state != NODE_WRITE_BUF)) {
            dl_buf_state = NODE_WRITE_BUF;
            
            msg = (rdc_msg_t*)dl_queue[last_queued_dl];
            
            msg->hdr.type    = ROUTER_DATA;
            msg->hdr.pld_len  = data_len;
            rimeaddr_copy(&msg->hdr.src, &rimeaddr_node_addr);
            rimeaddr_copy(&msg->hdr.dest, &dest);
            memcpy(msg->data, packetbuf_dataptr(), msg->hdr.pld_len);                                    
            
            last_queued_dl = (last_queued_dl + 1) % DL_QUEUE_SIZE;
            if(last_queued_dl == next_to_send_dl){
                dl_b_full   = 1;
            }
            //create_rdc_packet(msg, &dest, 0, data_len, ROUTER_DATA);
            dl_buf_state = NOTHING;
        }
    }

    if (rimeaddr_cmp(&dest, &rimeaddr_null) || (child_exist(&dest) == 0)) {
        if (ul_b_full == 0 && (ul_buf_state != NODE_WRITE_BUF)) {
            ul_buf_state        = NODE_WRITE_BUF;
            
            msg = ul_queue[last_queued_ul];
            msg->hdr.type     = ROUTER_DATA;
            msg->hdr.pld_len  = data_len;
            rimeaddr_copy(&msg->hdr.src, &rimeaddr_node_addr);
            rimeaddr_copy(&msg->hdr.dest, &dest);
            memcpy((char*)msg->data, packetbuf_dataptr(), msg->hdr.pld_len);                       
            
            last_queued_ul = (last_queued_ul + 1) % UL_QUEUE_SIZE;
            if(last_queued_ul == next_to_send_ul){
                ul_b_full = 1;
            }
            //create_rdc_packet(msg, &dest, 0, data_len, ROUTER_DATA);
            ul_buf_state = NOTHING;
        }
    }    
   #endif

   #if NODE_IS_MOBILE
    if (ul_b_full == 0){
        msg = ul_queue[last_queued_ul];
        msg->hdr.type     = MOBILE_DATA;
        msg->hdr.pld_len  = data_len;
        rimeaddr_copy(&msg->hdr.src, &rimeaddr_node_addr);
        rimeaddr_copy(&msg->hdr.dest, &dest);
        memcpy((void*)msg->data, packetbuf_dataptr(), msg->hdr.pld_len);             
        
        last_queued_ul = (last_queued_ul + 1) % UL_QUEUE_SIZE;
        if(last_queued_ul == next_to_send_ul){
                ul_b_full = 1;
        }
        //create_rdc_packet(msg, &dest, 1, data_len, MOBILE_DATA);
       //PRINTF("dest: %d.%d.%d.%d\n", dest.u8[4], dest.u8[5], dest.u8[6], dest.u8[7]);
    }
    #endif

   #if NODE_IS_SINK
    if ((dl_buf_state != NODE_WRITE_BUF) && dl_b_full == 0) {
            dl_buf_state = NODE_WRITE_BUF;
            msg = dl_queue[last_queued_dl];
            msg->hdr.type    = SINK_DATA;
            msg->hdr.pld_len  = data_len;
            rimeaddr_copy(&msg->hdr.src, &rimeaddr_node_addr);
            rimeaddr_copy(&msg->hdr.dest, &dest);
            memcpy((void*)msg->data, packetbuf_dataptr(), msg->hdr.pld_len);
       
            last_queued_dl = (last_queued_dl + 1) % DL_QUEUE_SIZE;

            if(last_queued_dl == next_to_send_dl){
                dl_b_full   = 1;
            }
            //create_rdc_packet(msg, &dest, 0, data_len, SINK_DATA);
            dl_buf_state = NOTHING;
            //PRINTF("dest: %d.%d.%d.%d\n", dest.u8[4], dest.u8[5], dest.u8[6], dest.u8[7]);
    }
    #endif

    return 1;
}
/*-----------------------------------------------------------------------------
 *when a synch packet is received, each node sets up its reference time to the
 *time at which the synch packet was received. In this way, we expect to synchronize
 * the mobile nodes.
 */
static int packet_in(void) {
    uint8_t len;
    static rdc_hdr_t hdr;
    static rdc_msg_t msg;

    len = radio->read((void*) & msg, PACKETBUF_SIZE);

    if (len > 0) {

        memcpy((char*) & hdr, (char*) & msg, RDC_HDR_LEN);

#if NODE_IS_MOBILE
        PRINTF("packet_type: %d\n", hdr.type);
#endif

#if NODE_IS_SINK
        if (hdr.type == ROUTER_SLOT_REQ) {
            /**process request localy*/

            if (child_list_len() < MAX_CHILDS) {
                /*the 'pld_len' in this case contains the number of childs
                 *attached to that router.
                 */
                child_add(&hdr.src, hdr.pld_len, 1);
            }
            return 1;
        }

        /*update data rate*/
        if (hdr.type == DATA_RATE_UPDATE) {
            child_set_state(&hdr.src, hdr.type, hdr.pld_len);
            //data_rate_update = 1;
            return 1;
        }

        if (hdr.type == ROUTER_DATA) {
            /**data packet received, signal upper layers*/
            uint8_t dlen = hdr.pld_len;
            if (!rimeaddr_cmp(&(hdr.src), &rimeaddr_node_addr) &&
                    (rimeaddr_cmp(&(hdr.dest), &rimeaddr_node_addr) || rimeaddr_cmp(&(hdr.dest), &rimeaddr_null))) {
#if WITH_UIP6                    			   
                if ((msg.data[0] == UIP_NTWK_PERFORMANCE) && (dlen == (sizeof (network_statistics_t) + 1))) {
                    if (inpacket_callback != NULL) {
                        //PRINTF("network performance packet...from: ");
                        PRINTMACADDR(&hdr.src);
                        inpacket_callback(&hdr.src, (void*) & msg.data[1], hdr.pld_len);
                    }
                } else {
                    if (uip_callback_func != NULL) {
                        /*PRINTF("packet received from: ");
                        PRINTAMACDDR(&hdr.src);*/
                        uip_callback_func(&hdr.src, &hdr.dest, (void*) & msg.data[0], hdr.pld_len);
                    }
                }
#else
                if (inpacket_callback != NULL) {
                    inpacket_callback(&hdr.src, (void*) & msg.data[0], hdr.pld_len);
                }
#endif                   
            } else {
                /*get the number of hops and increment it*/

                if ((dl_buf_state != NODE_WRITE_BUF) && (dl_b_full == 0)) {
                    dl_buf_state = NODE_WRITE_BUF;

                    //leds_toggle(LEDS_BLUE);
#if !WITH_UIP6
                    uint8_t *hops = (uint8_t*) (&msg.data[2]);
                    *hops = *hops + 1;
#endif                   

                    set_hdr_type(&msg.hdr, SINK_DATA);

                    memcpy(dl_queue[last_queued_dl], &msg, sizeof (rdc_msg_t));
                    last_queued_dl = (last_queued_dl + 1) % DL_QUEUE_SIZE;
                    if (last_queued_dl == next_to_send_dl) {
                        dl_b_full = 1;
                    }

                    dl_buf_state = NOTHING;
                }

            } //end of else
        }
#endif

#if NODE_IS_ROUTER
        if (hdr.type == MOBILE_SLOT_REQ) {
            /**process request localy*/
            if (child_list_len() < MAX_CHILDS) {
                /*the 'pld_len' paramenter is the mobile node data rate*/
                child_add(&hdr.src, hdr.pld_len, 3);
            }

            return 1;
        }
        /*update child status including data rate*/
        if (hdr.type == NODE_STATUS_UPDATE) {
            child_set_state(&hdr.src, hdr.type, hdr.pld_len);
        }
        /*release a slot, node is in handover mode*/
        if (hdr.type == NODE_SLOT_RELEASE) {
            if (child_remove(&hdr.src)) {
                data_rate_update = 1;
                mobile_has_left(&hdr.src, INSERT);
                cluster_size = (cluster_size > 0) ? (cluster_size - 1) : 0;
            }
            return 1;
        }
        /*update data rate*/
        if (hdr.type == DATA_RATE_UPDATE) {
            child_set_state(&hdr.src, hdr.type, hdr.pld_len);
            data_rate_update = 1;

            return 1;
        }

        if (hdr.type == SINK_DATA || hdr.type == MOBILE_DATA) {
            /**data packet received, signal upper layers*/
            uint8_t dlen = hdr.pld_len;
            is_there_data = 1; /*we have received data, so we must keep on waiting for more 1TS before we go NODE_SLEEP*/
            /*this is a data addressed to a mobile node or is a local-broadcast*/

            if ((!rimeaddr_cmp(&(hdr.src), &rimeaddr_node_addr)) &&
                    (rimeaddr_cmp(&(hdr.dest), &rimeaddr_node_addr) || rimeaddr_cmp(&(hdr.dest), &rimeaddr_null))) {
                /*this trick is necessary to allow cluster broadcast*/
                static rdc_msg_t msg_cpy;
                memcpy(&msg_cpy, &msg, (RDC_HDR_LEN + hdr.pld_len));
#if WITH_UIP6
                if (uip_callback_func != NULL) {
                    uip_callback_func(&msg_cpy.hdr.src, &msg_cpy.hdr.dest, (void*) & msg_cpy.data[0], dlen);
                }
#else
                if (inpacket_callback != NULL) {
                    inpacket_callback(&msg_cpy.hdr.src, (void*) & msg_cpy.data[0], dlen);
                }
#endif                                      
            }

            if (child_exist(&hdr.dest) ||
                    (rimeaddr_cmp(&hdr.dest, &rimeaddr_null) && hdr.type == MOBILE_DATA)) {
                if ((dl_buf_state != NODE_WRITE_BUF) && (dl_b_full == 0)) {
                    dl_buf_state = NODE_WRITE_BUF;

                    set_hdr_type(&msg.hdr, ROUTER_DATA);
                    //get the number of hops and increment it
#if !WITH_UIP6
                    uint8_t *hops = (uint8_t*) (&msg.data[2]);
                    *hops = *hops + 1;
#endif                           
                    memcpy((void*) dl_queue[last_queued_dl], (void*) & msg, (RDC_HDR_LEN + hdr.pld_len));

                    last_queued_dl = (last_queued_dl + 1) % DL_QUEUE_SIZE;

                    if (last_queued_dl == next_to_send_dl) {
                        dl_b_full = 1;
                    }
                    dl_buf_state = NOTHING;
                    leds_toggle(LEDS_BLUE);
#if WITH_UIP6
                    PRINTF("Routing DL: dst");
                    PRINTMACADDR(&hdr.dest);
                    PRINTF("src: ");
                    PRINTMACADDR(&hdr.src);
#endif
                } /**dl_buf_state*/

            } else {
                /*the destination was not found in the router's list, if the data is from a mobile node
                  this should be forwarded upstream*/
                if (((!rimeaddr_cmp(&hdr.dest, &rimeaddr_node_addr)) &&
                        (child_exist(&hdr.dest) == 0) && hdr.type == MOBILE_DATA) || mobile_has_left(&hdr.dest, LOOKUP)) {
                    if ((ul_b_full == 0) && (ul_buf_state != NODE_WRITE_BUF)) {
                        ul_buf_state = NODE_WRITE_BUF;

                        set_hdr_type(&msg.hdr, ROUTER_DATA);
                        //get the number of hops and increment it
#if !WITH_UIP6
                        uint8_t *hops = (uint8_t*) (&msg.data[2]);
                        *hops = *hops + 1;
#endif 

                        memcpy((void*) ul_queue[last_queued_ul], (void*) & msg, (RDC_HDR_LEN + hdr.pld_len));
                        last_queued_ul = (last_queued_ul + 1) % UL_QUEUE_SIZE;

                        if (last_queued_ul == next_to_send_ul) {
                            ul_b_full = 1;
                        }
                        ul_buf_state = NOTHING;

                        leds_toggle(LEDS_BLUE);

#if WITH_UIP6
                        PRINTF("Routing UL: dst");
                        PRINTMACADDR(&hdr.dest);
                        PRINTF("src: ");
                        PRINTMACADDR(&hdr.src);
#endif                            
                    }
                } /**end of child_exist(&hdr.dest)) && hdr.type == MOBILE_DATA) */
            }

        } /*end of hdr.type == SINK_DATA || hdr.type == MOBILE_DATA*/

#endif

#if NODE_IS_ROUTER || NODE_IS_MOBILE
        if (hdr.type == HANDOVER_INFO) {
            admin_info_t adv;
            memcpy(&adv, &msg, ADMIN_MSG_LEN);

#if NODE_IS_MOBILE  /*MOBILE NODEs connect only to routers, so we must skip the sink msg*/
            if (adv.hops != 0) {
                parents_add(&adv);
            }
#else  /*a router can also add other routers*/
            parents_add(&adv);
#endif		
            return 1;
        }
#endif  /*NODE_IS_ROUTER || NODE_IS_MOBILE*/

#if NODE_IS_MOBILE
        if ((!is_scanning) && (hdr.type == ROUTER_DATA)) {
            /**data packet received, signal upper layers*/
            uint8_t dlen = hdr.pld_len;
            /*we have received data, so we must keep on waiting for more 1TS before we go NODE_SLEEP*/
            is_there_data = 1; 

            /* PRINTF("data received: dst");
            PRINTMACADDR(&hdr.dest);
            PRINTF("src: ");
            PRINTMACADDR(&hdr.src); */

            if ((rimeaddr_cmp(&hdr.src, &rimeaddr_node_addr) == 0) &&
                    (rimeaddr_cmp(&hdr.dest, &rimeaddr_node_addr) || rimeaddr_cmp(&hdr.dest, &rimeaddr_null))) {

#if WITH_UIP6		    
                if (uip_callback_func != NULL) {
                    uip_callback_func(&msg.hdr.src, &msg.hdr.dest, (void*) & msg.data[0], dlen);
                }
#else
                if (inpacket_callback != NULL) {
                    inpacket_callback(&hdr.src, (void*) & msg.data[0], dlen);
                }
#endif                                                              
            }
            return 1;
        }
#endif /*NODE_IS_MOBILE hdr.type == ROUTER_DATA */

#if NODE_IS_MOBILE || NODE_IS_ROUTER
        uint8_t ret;
        ref_timer = RTIMER_NOW(); //reference timer

        if (hdr.type == SINK_BEACON || hdr.type == ROUTER_BEACON) {
            /**don't signal upper layers, synchronize*/

            synch_msg_t beacon;
            uint8_t k, schedule_len = hdr.pld_len - SYNCH_MSG_LEN;
	    
            memcpy((char*) & beacon, (char*) & msg.data[0], sizeof (synch_msg_t));

#if NODE_IS_ROUTER
            if (hdr.type == ROUTER_BEACON) {
                return 1; //skip router beacons
            }
            num_neighbors = beacon.n_childs;
            free_slots = beacon.free_slots;
            beacon_seqno = beacon.seqno;
            nr_hops = beacon.nr_hops;

#endif

#if NODE_IS_MOBILE
            signed char base_rssi;

            occupied_ts = 0;

            if (hdr.type == SINK_BEACON) {
                return 1; //skip sink beacons
            }
            base_rssi = PWR_DBM();

            free_slots = beacon.free_slots;
            beacon_seqno = beacon.seqno;
            num_neighbors = beacon.n_childs;
            ch_offset = beacon.ch_offset;
            nr_hops = beacon.nr_hops;
            uplink_ch = data_channels[ch_offset];
            /**the received power has dropped to an unacceptable level, we instruct the mobile node to start handover*/
            if (admitted) {
                if (TIME_DIFF(base_rssi, TH_POWER)) {
                    if (neigh_adv_list_len()) {
                        handover = 1;
                        hv_t0_lock = 1;
                    }
                }
            }
	    
	    PRINTF("sched_len: %d\n", schedule_len);
#endif  //NODE_IS_MOBILE

            if (hdr.pld_len > SYNCH_MSG_LEN) {
                uint8_t* b_ptr;

                b_ptr = &msg.data[SYNCH_MSG_LEN];
                schedule_len = hdr.pld_len - SYNCH_MSG_LEN;
#if NODE_IS_MOBILE
  
#endif
                for (k = 0; k < schedule_len; k += SLOT_MSG_LEN) {

                    slot_dist_t* slot = (slot_dist_t*) (b_ptr + k);

                    /*we must count how many occupied slots exist on the cluster head*/
#if NODE_IS_MOBILE
                    occupied_ts = occupied_ts + slot->assig_slots;
#endif

                    if (rimeaddr_cmp(&slot->node_id, &rimeaddr_node_addr)) {

#if NODE_IS_ROUTER
                        is_there_pkt = 1;
                        ts_offset = (slot->ts_offset & 0x1F);
                        /*we add one because the channel offset cannot be zero
                         *to avoid having the same channel with the sink node*/
                        ch_offset = (1 + (slot->ts_offset >> 5));
                        PRINTF("router_ch_offset: %d\n", ch_offset);
                        //uplink_ch    = data_channels[ch_offset];
                        //is_there_pkt = (slot->assig_slots >> 5);
                        assign_slots = (slot->assig_slots & 0x1F);
#endif

#if NODE_IS_MOBILE
                        is_there_pkt = 0;
                        ts_offset = (slot->ts_offset & 0x1F);
                        is_there_pkt = (slot->ts_offset >> 5);

                        assign_slots = slot->assig_slots;

                        PRINTF("ch_off/ass_slots:%d/%d\n", ch_offset, assign_slots);
#endif								

                        admitted = 1;
                        is_scanning = 0;

                        /*statistics data*/
                        ass_d_rate = assign_slots;
#if NODE_IS_MOBILE & RATE_CONTROL
                        if (rate_update_func != NULL) {
                            if (node_data_rate != assign_slots) {
                                rate_update_func(ass_d_rate);
                            }
                        }
#endif

                        /*get admission time*/
                        if (ad_tn_lock) {
                            ad_tn_lock = 0;
                            ad_tn = RTIMER_NOW();

                            /*we enable this info to reach the sink*/
                            send_stats_pkt = 1;

                            if (nc_tn_lock) {
                                nc_tn_lock = 0;
                                nc_tn = ad_tn;
                                /**The node was admitted for the first time. then take the current
                                 router as the cluster head*/
                                rimeaddr_copy(&curr_router, &hdr.src);
                                rimeaddr_copy(&next_router, &hdr.src);
                            }
                            //PRINTF("ad_tn: %2u \n",ad_tn);
                            if (hv_tn_lock) {
                                hv_tn = ad_tn;
                                report_type = 2;
                                /** The nodes has finished handover, get the next*/
                                rimeaddr_copy(&curr_router, &next_router);
                                rimeaddr_copy(&next_router, &hdr.src);
                            }
                        } /*end of ad_tn_lock*/
                    } /* end of rimeaddr_cmp(&slot->node_id, &rimeaddr_node_addr)*/
                } //end of for


            } /*end of if (hdr.pld_len > 0)*/

            if (admitted == 0) {
                uint8_t idx;

#if NODE_IS_MOBILE
                signed char base_rssi = PWR_DBM();
                if ((TIME_DIFF(base_rssi, TH_POWER) || (num_neighbors == MAX_CHILDS))
                        || (atempts >= 4  || free_slots == 0)) {

                    atempts     = 0;
                    uplink_ch   = 0;
                    is_scanning = 1;
                    
                    handover   = 1;
                    node_state = NODE_SLEEP;
		    
		    if (ch_offset % 2 == 0) {
			//Depending on the cluster synchronization, we should fall in the HDVR_SLOTS
                        hdvr_rx_to  = NORM_CLOCK(ref_timer + MOB_AWAKE_OFFSET_1);
		    }else{
		         //Depending on the cluster synchronization, we should fall in the HDVR_SLOTS
			hdvr_rx_to = NORM_CLOCK(ref_timer + MOB_AWAKE_OFFSET_2);		    
		    }
                    off();
                    ret = rtimer_set(&generic_timer, hdvr_rx_to, 1,
                            (void (*)(struct rtimer *, void *))mobile_power_cycle, NULL);
                    if (ret) {
                        //PRINTF("ERROR_MOBILE_admission: %d %u\n",r, RTIMER_NOW());
                    }

                    return 1;
                } /**end of TIME_DIFF(base_rssi, TH_POWER)*/
#endif

                atempts++;
                node_state = NODE_RX_SYNCH;

                if (free_slots != 0) {
                    idx = (uint8_t) (random_rand() % free_slots);
                } else {
                    idx = 0;
                }

#if NODE_IS_MOBILE
                radio_clock = 0;
                uint8_t rand_offset = 0;
                if(idx != 0){
                    rand_offset = (uint8_t) (random_rand() % (2*idx));
                }else{
                    rand_offset = 1;
                }
                radio_clock = NORM_CLOCK(ref_timer + ROUTER_DL_TX_TS + occupied_ts * TS + rand_offset * (TS / 2)); /*and offset is added to the*/
                synch_rx_to = NORM_CLOCK(ref_timer + AWAKE_PERIOD); //SUPER_FRAME_DURATION;
               
                off();
                PRINTF("mobile node not admitted...Requesting admission\n");
                /**If a node is requesting */
                ret = rtimer_set(&generic_timer, radio_clock, 1,
                        (void (*)(struct rtimer *, void *))slot_request, NULL);
                if (ret) {
                    //PRINTF("err_ send_slot_req:%d free_slots/idx:%d/%d\n", ret, free_slots, idx);
                }
#endif /**NODE_IS_MOBILE*/

#if NODE_IS_ROUTER
                if (num_neighbors == 0) {
                    //idx = idx % 63;
                    radio_clock = ref_timer + (ROUTER_DL_TX_TS + (random_rand() % 63) * TS);
                } else {
                    if (num_neighbors == 1) {
                        radio_clock = ref_timer + ROUTER_DL_TX_TS + (ROUTER_UL_TX_TS - idx * TS);
                    } else if (num_neighbors >= 2 && num_neighbors != 7) {
                        radio_clock = ref_timer + ROUTER_DL_TX_TS + (1 + idx % 2) * ROUTER_UL_TX_TS - (idx / 2) * TS;
                    } else if (num_neighbors == 7) {
                        radio_clock = ref_timer + (ROUTER_DL_TX_TS + ROUTER_UL_TX_TS - TS);
                    }
                }

		radio_clock = NORM_CLOCK(radio_clock);
                
		
                ret = rtimer_set(&generic_timer, radio_clock, 1,
                        (void (*)(struct rtimer *, void *))slot_request, NULL);
                if (ret) {
                    //PRINTF("err_ send_slot_req:%d free_slots/idx:%d/%d\n", ret, free_slots, idx);
                }

                off();
                synch_rx_to = NORM_CLOCK(ref_timer + AWAKE_PERIOD);
                PRINTF("router node tot admitted...Requesting admission\n");
#endif

            } else { /**adtmitted == 1*/
                /*the node was admitted... calculate some parameters*/

                //PRINTF("Admitted....\n");
                node_state = NODE_RX_UPLINK;

#if NODE_IS_MOBILE
                /*--INIT OF schedule mobile node transmission------*/

                /**The time when to start transmitting upstreams*/
                ul_tx_to = NORM_CLOCK(ref_timer + ROUTER_DL_TX_TS + (ts_offset * TS));
                /**The time when to finish transmitting upstreams.
                Note that we also subtract the GUARD_TIME*/
                ul_tx_tn = NORM_CLOCK(ul_tx_to + (assign_slots * TS - GUARD_TIME));                

                if (ch_offset % 2 == 0) {
                    /**Depending on the cluster synchronization, we should fall in the HDVR_SLOTS*/
                    hdvr_rx_to  = NORM_CLOCK(ref_timer + MOB_AWAKE_OFFSET_1);
                    /**If no handover occurs, synchronize the back to the CH at this time*/
                    synch_rx_to = ref_timer + MOB_AWAKE_OFFSET_1 + (HDVR_TX_TS + ROUTER_DL_TX_TS + 2 * GUARD_TIME);
                } else {
                    /**Depending on the cluster synchronization, we should fall in the HDVR_SLOTS*/
                    hdvr_rx_to = NORM_CLOCK(ref_timer + MOB_AWAKE_OFFSET_2);
                    /**If no handover occurs, synchronize the back to the CH at this time*/
                    synch_rx_to = ref_timer + MOB_AWAKE_OFFSET_2 + (HDVR_TX_TS + ROUTER_DL_TX_TS + ROUTER_UL_TX_TS + 2 * GUARD_TIME);
                }
		
		synch_rx_to  = NORM_CLOCK(synch_rx_to);  //ref_timer + TS
  
		PRINTF("synch_time: %li/%li\n", synch_rx_to, ref_timer);
                //off();
                /*OBS: seting this flag to zero means that no downlink data will be
                 received*/
                is_there_data = 0;
                ret = rtimer_set(&generic_timer, RTIMER_NOW()+1 /*ref_timer + TS*/, 1,
                        (void (*)(struct rtimer *, void *))mobile_power_cycle, NULL);
                if (ret) {
                    PRINTF("mobile error power_cycle:%d\n", ret);
                }

                /*---END OF schedule mobile node transmission-----*/
#endif  /*NODE_IS_MOBILE*/

#if NODE_IS_ROUTER
                /*---INIT OF schedule router node transmission-----*/

                if (ch_offset % 2 == 1) {

                    /*we get the time when to start and end transmitting upstream*/
                    ul_tx_to = NORM_CLOCK(ref_timer + ROUTER_DL_TX_TS + (ts_offset * TS));
                    ul_tx_tn = NORM_CLOCK(ul_tx_to + assign_slots*TS);
                    /*we schedule when the dl_tx should occur*/
                    dl_tx_to = NORM_CLOCK(ref_timer + SINK_DL_TX_TS + ROUTER_UL_TX_TS);
                    dl_tx_tn = NORM_CLOCK(dl_tx_to + ROUTER_DL_TX_TS);

                    radio_clock = NORM_CLOCK(ref_timer + TS / 2);

                    radio_off_to = NORM_CLOCK(ref_timer + (ROUTER_DL_TX_TS - TS));

                    infra_rx_to = dl_tx_tn;
                    infra_rx_tn = NORM_CLOCK(dl_tx_tn + ((MAX_CHILDS / 3 - cluster_size / 3) + slots_offset) * TS);

                } else {
                    /*--INIT schedule downlink Tx and Rx---*/

                    dl_tx_to = NORM_CLOCK(ref_timer + SINK_DL_TX_TS);
                    dl_tx_tn = NORM_CLOCK(dl_tx_to + ROUTER_DL_TX_TS);

                    /*we get the time when to start and end transmitting upstream*/
                    ul_tx_to  = NORM_CLOCK(ref_timer + ROUTER_DL_TX_TS + ROUTER_UL_TX_TS + (ts_offset * TS));
                    ul_tx_tn  = NORM_CLOCK(ul_tx_to + assign_slots*TS);
                    ul_tx_sch = NORM_CLOCK(ref_timer + ROUTER_DL_TX_TS + ROUTER_UL_TX_TS);

                    radio_clock = NORM_CLOCK(ref_timer + TS / 2);                  

                    infra_rx_to  = dl_tx_tn;
                    infra_rx_tn  = NORM_CLOCK(dl_tx_tn + ((MAX_CHILDS / 3 - cluster_size / 3) + slots_offset) * TS); //here is the nightmare :)

                    radio_off_to = NORM_CLOCK(ref_timer + (ROUTER_DL_TX_TS - TS));
                }

                hdvr_tx_to = ref_timer + (SINK_DL_TX_TS + SINK_UL_RX_TS) + (ch_offset * 2*ONE_MSEC);
                synch_rx_to = ref_timer + (AWAKE_PERIOD);
		
		hdvr_tx_to   = NORM_CLOCK(hdvr_tx_to);
		synch_rx_to  = NORM_CLOCK(synch_rx_to);

               /*OBS: seting this flag to zero means that no downlink data will be
                 received*/
                is_there_data = 0;
                ret = rtimer_set(&generic_timer, NORM_CLOCK(ref_timer + TS), 1,
                        (void (*)(struct rtimer *, void *))router_power_cycle, NULL);
                if (ret) {
                    //PRINTF("error scheduling router power_cycle:%d\n", ret);
                }
                /*---END OF schedule router node transmission-----*/
#endif     /*---END OF NODE_IS_ROUTER----*/
            } /*---END OF ELSE(admitted==0)---*/

        } /*---END of if (hdr.type == SINK_BEACON || hdr.type == ROUTER_BEACON)---*/

#endif  /*--END OF NODE_IS_MOBILE || NODE_IS_ROUTER--*/

    } else {
        PRINTF("Received packet length not valid\n");
    }

    return 0;
}

/**---------------------------------------------------------------------------*/
void mobile_set_recv_callback(void (* func)(rimeaddr_t *sd, void* data, uint8_t len)) {
    inpacket_callback  = func;  /*we need to pass the data directly to the application layer*/
}
/**---------------------------------------------------------------------------*/
#if WITH_UIP6
void
mobile_uip_set_recv_func(void (* func)(rimeaddr_t *src, rimeaddr_t *dest, void* data, uint8_t len)){
   uip_callback_func = func;
}
#endif
/**---------------------------------------------------------------------------*/
#if NODE_IS_ROUTER || NODE_IS_MOBILE
void mobile_set_data_rate(uint8_t new_rate){
    if(data_rate == new_rate || new_rate == 0){
        return; //no update needed
    }
    if(new_rate < MIN_DATA_RATE){
        data_rate = MIN_DATA_RATE;
    }else{
        data_rate = new_rate;
    }
    data_rate_update = 1;
}
#endif
/**---------------------------------------------------------------------------*/
 #if NODE_IS_MOBILE & RATE_CONTROL
void mobile_set_rate_update_func(void (* rfunc)( uint8_t len)) {
    rate_update_func = rfunc;
}
#endif
/**---------------------------------------------------------------------------*/
static int turn_radio_on() {
    if (!radio_on) {
        radio_on = 1;
        radio_off = 0;
        return radio->on();
    }
    return 0;
}

/**---------------------------------------------------------------------------*/
static int turn_radio_off() {
    if (radio_on) {
        radio_on = 0;
        radio_off = 1;
        return radio->off();
    }
    return 0;
}

/**---------------------------------------------------------------------------*/
static void input_packet(const struct radio_driver *d) {
    if(receiver_callback) {
      receiver_callback(&mobile_driver);
    }
}

/**---------------------------------------------------------------------------*/
static void
set_receive_function(void (* recv)(const struct mac_driver *)) {
    receiver_callback = recv;
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return 0;
}
/**---------------------------------------------------------------------------*/
const struct mac_driver *mobile_mac_init(const struct radio_driver *radio_dr) {

    uint8_t ret;

    data_buffers_init(); /**Initialize the data buffers*/
    
    radio = radio_dr;
    radio->set_receive_function(input_packet);

    turn_radio_on();
    
#if NODE_IS_SINK
    init_child_list();
    node_state = NODE_TX_DOWNLINK;
    ret = rtimer_set(&generic_timer, RTIMER_NOW() + ROUTER_DL_TX_TS, 1,
            (void (*)(struct rtimer *, void *))downlink_transmitter, NULL);
    if (ret) {
        //PRINTF("error starting periodic beacon at: %u\n", RTIMER_NOW());
    }

#endif

#if NODE_IS_ROUTER
    node_state  = NOTHING;
    init_child_list();
    neigh_adv_list_init();
    ret = rtimer_set(&generic_timer, RTIMER_NOW() + ROUTER_DL_TX_TS/2, 1,
            (void (*)(struct rtimer *, void *))router_power_cycle, NULL);
    if (ret) {
        //PRINTF("error starting mobile entry: %u\n", RTIMER_NOW());
    }  
#endif

#if NODE_IS_MOBILE
    /**when starting as mobile node, first scan the medium to locate router
     *nodes.initialize list of routers in the vicinity*/
    node_state  = NOTHING;
    neigh_adv_list_init();
    is_scanning = 1;

    ret = rtimer_set(&generic_timer, RTIMER_NOW() + ROUTER_DL_TX_TS, 1,
            (void (*)(struct rtimer *, void *))mobile_chead_selection, NULL);
    if (ret) {
        PRINTF("error starting mobile entry: %u\n", RTIMER_NOW());
    }    
#endif
 
#if NODE_IS_ROUTER || NODE_IS_MOBILE
    report_type = 1;
    ad_t0_lock = 1; /*enable getting admission time t0*/
    nc_t0 = RTIMER_NOW();    
 #endif
    return &mobile_driver;
}
/**---------------------------------------------------------------------------*/
const struct mac_driver mobile_driver = {
    "mobisenseMAC",
    mobile_mac_init,
    packet_out,
    packet_in,
    set_receive_function,
    turn_radio_on,
    turn_radio_off,
    channel_check_interval,
};


/**---------------------------------------------------------------------------*/
///MAKEFILES MOBILE MAKEFILE
CONTIKI_PROJECT = mobile
ifndef TARGET
	TARGET=sky
endif
all: $(CONTIKI_PROJECT)

DEFINES=WITH_MOBILEMAC NODE_IS_MOBILE RATE_CONTROL  #TDMA_OPTIMIZED



CONTIKI = ../../..
include $(CONTIKI)/Makefile.include
/**---------------------------------------------------------------------------*/
///MAKEFILES SINK MAKEFILE
CONTIKI_PROJECT = sink

ifndef TARGET
	TARGET=sky
endif

all: $(CONTIKI_PROJECT)

DEFINES=WITH_MOBILEMAC NODE_IS_SINK SINK_SERIAL
#TDMA_OPTIMIZED


CONTIKI = ../../..
include $(CONTIKI)/Makefile.include

/**---------------------------------------------------------------------------*/
///MAKEFILES ROUTER MAKEFILE
CONTIKI_PROJECT = router

ifndef TARGET
	TARGET=sky
endif

all: $(CONTIKI_PROJECT)

DEFINES=WITH_MOBILEMAC NODE_IS_ROUTER   #WITH_NULLMAC #TDMA_OPTIMIZED


CONTIKI = ../../..
include $(CONTIKI)/Makefile.include
/**---------------------------------------------------------------------------*/
