/* test_repeater_id_from_login.c — translator_repeater_id(): use the sole
 * registered IPSC repeater's own radio ID when exactly one is connected,
 * fall back to the configured hbp_repeater_id otherwise (0 or 2+ repeaters,
 * or ignore_login_repeater_id set).
 *
 * Links the REAL ipsc.c (not stubbed) over real loopback UDP + the real
 * event loop, so real MASTER_REG_REQ registrations drive the peer table.
 * hbp.c is stubbed (this feature doesn't touch it directly -- hbp_connect()
 * just calls translator_repeater_id(), tested here in isolation). */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "../src/translate.h"
#include "../src/config.h"
#include "../src/eventloop.h"
#include "../src/ipsc.h"
#include "../src/ipsc_const.h"
#include "../src/net.h"

int  hbp_is_connected(struct hbp *hb) { (void)hb; return 0; }
void hbp_activate(struct hbp *hb) { (void)hb; }
void hbp_deactivate(struct hbp *hb) { (void)hb; }
void hbp_send_dmrd(struct hbp *hb, const uint8_t *d, int n) { (void)hb; (void)d; (void)n; }

static void stop_cb(ev_loop *loop, void *ud) { (void)ud; ev_stop(loop); }
static void pump(ev_loop *loop)
{
    ev_timer_after(loop, 0.05, stop_cb, loop);
    ev_run(loop);
}

/* MASTER_REG_REQ: opcode(1) + radio_id(4) + mode(1) + version(4) = 10 bytes. */
static void register_repeater(int fd, const char *ip, int port, uint32_t radio_id)
{
    uint8_t pkt[10];
    pkt[0] = MASTER_REG_REQ;
    pkt[1] = (uint8_t)(radio_id >> 24); pkt[2] = (uint8_t)(radio_id >> 16);
    pkt[3] = (uint8_t)(radio_id >> 8);  pkt[4] = (uint8_t)radio_id;
    pkt[5] = 0x6A;
    pkt[6] = pkt[7] = pkt[8] = pkt[9] = 0x00;
    udp_sendto(fd, pkt, sizeof pkt, ip, port);
}

int main(void)
{
    Config cfg; char err[4096];
    if (config_load("tests/repeater_id_from_login.toml", &cfg, err, sizeof err)) {
        fprintf(stderr, "config: %s\n", err); return 2;
    }

    ev_loop *loop = ev_new();
    translator *tr = translator_new(&cfg, loop);
    ipsc *ip = ipsc_new(&cfg, tr, loop);
    translator_set_protocols(tr, ip, (struct hbp *)1);
    if (ipsc_start(ip) != 0) { fprintf(stderr, "ipsc_start failed\n"); return 2; }

    int fake_fd = udp_socket();
    int checks = 0, fails = 0;

    /* 1. No repeaters registered yet -> config's hbp_repeater_id. */
    checks++;
    if (translator_repeater_id(tr) != cfg.hbp_repeater_id) {
        fprintf(stderr, "FAIL: expected config id %u with 0 peers, got %u\n",
                cfg.hbp_repeater_id, translator_repeater_id(tr));
        fails++;
    }

    /* 2. Exactly one repeater registered -> its own radio ID. */
    register_repeater(fake_fd, cfg.ipsc_bind_ip, cfg.ipsc_bind_port, 3120001);
    pump(loop);
    checks++;
    if (translator_repeater_id(tr) != 3120001) {
        fprintf(stderr, "FAIL: expected repeater's own id 3120001 with 1 peer, got %u\n",
                translator_repeater_id(tr));
        fails++;
    }

    /* 3. A second repeater registers -> ambiguous again, back to config id. */
    register_repeater(fake_fd, cfg.ipsc_bind_ip, cfg.ipsc_bind_port, 3120002);
    pump(loop);
    checks++;
    if (translator_repeater_id(tr) != cfg.hbp_repeater_id) {
        fprintf(stderr, "FAIL: expected config id %u with 2 peers, got %u\n",
                cfg.hbp_repeater_id, translator_repeater_id(tr));
        fails++;
    }

    /* 4. ignore_login_repeater_id forces config id even with exactly one peer. */
    {
        Config cfg2 = cfg;
        cfg2.ignore_login_repeater_id = 1;
        translator *tr2 = translator_new(&cfg2, loop);
        translator_set_protocols(tr2, ip, (struct hbp *)1);
        /* ip currently has 2 registered peers (from steps above) -- de-register
         * one so exactly one remains, to prove the flag (not peer count) gates this. */
        uint32_t dereg_id = 3120002;
        uint8_t dereg[5];
        dereg[0] = DE_REG_REQ;
        dereg[1] = (uint8_t)(dereg_id >> 24); dereg[2] = (uint8_t)(dereg_id >> 16);
        dereg[3] = (uint8_t)(dereg_id >> 8);  dereg[4] = (uint8_t)dereg_id;
        udp_sendto(fake_fd, dereg, sizeof dereg, cfg.ipsc_bind_ip, cfg.ipsc_bind_port);
        pump(loop);
        checks++;
        if (translator_repeater_id(tr2) != cfg2.hbp_repeater_id) {
            fprintf(stderr, "FAIL: expected config id %u with ignore_login_repeater_id set, got %u\n",
                    cfg2.hbp_repeater_id, translator_repeater_id(tr2));
            fails++;
        }
        translator_free(tr2);
    }

    printf("Repeater ID from login: %d checks, %d failures\n", checks, fails);

    close(fake_fd);
    ipsc_free(ip); translator_free(tr); ev_free(loop);
    return fails ? 1 : 0;
}
