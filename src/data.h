/* data.h — IPSC GROUP_DATA/PVT_DATA <-> HBP data bursts. Separate from the voice translator. */
#ifndef DATA_H
#define DATA_H

#include <stdint.h>
#include "config.h"
#include "eventloop.h"

typedef struct datapath datapath;
struct ipsc;
struct hbp;
struct translator;

datapath *datapath_new(const Config *cfg, ev_loop *loop);
void datapath_set_protocols(datapath *dp, struct ipsc *ip, struct hbp *hb, struct translator *tr);
void datapath_free(datapath *dp);

/* A whole 0x83/0x84 packet from a registered peer. */
void datapath_ipsc_received(datapath *dp, const uint8_t *pkt, int len);

/* DMRD carrying a CSBK, data header or rate 1/2 / 3/4 block; voice and its LC header/terminator are not. */
int  datapath_is_data_frame(const uint8_t *dmrd, int len);
/* Queued per slot: sent once the repeater has been quiet for a while, one burst per TDMA frame. */
void datapath_hbp_received(datapath *dp, const uint8_t *dmrd, int len);

#endif
