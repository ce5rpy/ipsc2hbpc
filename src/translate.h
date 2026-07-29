/* translate.h — CallTranslator: bidirectional IPSC <-> HBP translation. */
#ifndef TRANSLATE_H
#define TRANSLATE_H

#include <stdint.h>
#include "config.h"
#include "eventloop.h"

typedef struct translator translator;
struct ipsc;
struct hbp;

translator *translator_new(const Config *cfg, ev_loop *loop);
void translator_set_protocols(translator *tr, struct ipsc *ip, struct hbp *hb);
void translator_free(translator *tr);

/* Effective HBP repeater ID: the sole connected IPSC repeater's own radio ID
 * if exactly one is registered (master mode) and cfg->ignore_login_repeater_id
 * is not set -- otherwise the configured hbp_repeater_id. Re-evaluated by
 * hbp.c at each login attempt (RPTL), not live-updated mid-session: an
 * already-connected HBP session keeps its login identity until the next
 * reconnect. */
uint32_t translator_repeater_id(translator *tr);

/* IPSC-side callbacks */
void translator_peer_joined(translator *tr);
void translator_peer_lost(translator *tr);
void translator_ipsc_voice_received(translator *tr, const uint8_t *data, int len,
                                    int ts, int burst_type);
void translator_check_call_timeouts(translator *tr);

/* HBP-side callbacks */
void translator_hbp_connected(translator *tr);
void translator_hbp_disconnected(translator *tr);
void translator_hbp_voice_received(translator *tr, const uint8_t *dmrd, int len);

#endif
