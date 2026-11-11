/* config.h — parsed, validated configuration (mirrors config.py Config). */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stddef.h>

#define CFG_MAX_PEERS_LIST 64

typedef struct {
    int      log_level;          /* LOG_* enum */

    /* [ipsc] */
    char     ipsc_bind_ip[64];
    int      ipsc_bind_port;
    uint32_t ipsc_master_id;

    uint32_t allowed_peer_ids[CFG_MAX_PEERS_LIST];
    int      n_allowed_peer_ids;
    char     allowed_peer_ips[CFG_MAX_PEERS_LIST][16];
    int      n_allowed_peer_ips;

    int      auth_enabled;
    uint8_t  auth_key[20];
    int      keepalive_watchdog;
    char     status_file[256];   /* optional; empty = disabled, see ipsc.c write_status_file().
                                  * Rewritten at least every keepalive_watchdog seconds even with
                                  * no state change, so a monitor can tell a dead ipsc2hbpc
                                  * process from one with nothing to report. */

    /* [ipsc.capabilities] — computed wire bytes */
    uint8_t  ipsc_mode_byte;        /* 1 byte */
    uint8_t  ipsc_flags_bytes[4];   /* 4 bytes */
    uint8_t  ipsc_version[4];       /* 4 bytes */

    int      ipsc_ts_prefer_call_info;

    /* connection mode */
    char     ipsc_mode[8];          /* "MASTER" | "PEER" */
    char     ipsc_master_ip[256];
    int      ipsc_master_port;
    int      keepalive_interval;
    int      keepalive_missed_max;

    /* [hbp] */
    char     hbp_role[8];           /* "CLIENT" | "GATEWAY" */
    char     hbp_master_ip[256];    /* CLIENT role only */
    int      hbp_master_port;       /* CLIENT role only */
    char     hbp_bind_ip[64];       /* GATEWAY role only */
    int      hbp_bind_port;         /* GATEWAY role only */
    char     hbp_gateway_ip[64];    /* GATEWAY role only */
    int      hbp_gateway_port;      /* GATEWAY role only */
    uint32_t hbp_repeater_id;
    int      ignore_login_repeater_id;  /* MASTER-mode IPSC: force hbp_repeater_id even
                                          * with exactly one repeater registered */
    char     hbp_passphrase[256];
    int      hbp_passphrase_len;
    char     hbp_mode[16];          /* "TRACKING" | "PERSISTENT" — CLIENT role only */
    int      jitter_buffer_depth;   /* HBP->IPSC delivery delay in 60 ms slots */

    /* RPTC announcement fields */
    char     options[512];
    char     callsign[64];
    char     rx_freq[32];
    char     tx_freq[32];
    char     tx_power[16];
    char     colorcode[16];
    char     latitude[32];
    char     longitude[32];
    char     height[16];
    char     location[64];
    char     description[64];
    char     url[256];
    char     software_id[64];
    char     package_id[64];
} Config;

/* Load and validate a TOML config file.  Returns 0 on success; on failure
 * returns -1 and fills err with a human-readable message. */
int config_load(const char *path, Config *cfg, char *err, size_t errlen);

#endif
