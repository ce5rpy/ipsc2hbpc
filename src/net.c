/* net.c — UDP socket helpers. */
#include "net.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>

/* Same remedy as new-adn-server's udp_rcvbuf.py (DEFAULT_UDP_RCVBUF): raise
 * both directions well past the Linux default (~208KB) so a burst of DMR
 * traffic doesn't overflow the kernel socket buffer and get silently
 * dropped (counted as UdpRcvbufErrors) before this process ever sees it.
 * Best-effort -- a capped/refused setsockopt just leaves the OS default. */
#define UDP_BUFSIZE (4 * 1024 * 1024)

static void widen_bufsize(int fd)
{
    int sz = UDP_BUFSIZE;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &sz, sizeof sz);
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sz, sizeof sz);
}

int udp_socket(void)
{
    return socket(AF_INET, SOCK_DGRAM, 0);
}

int udp_connect(const char *host, int port)
{
    char portstr[16];
    snprintf(portstr, sizeof portstr, "%d", port);
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof hints);
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(host, portstr, &hints, &res) != 0 || !res)
        return -1;
    int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd < 0) { freeaddrinfo(res); return -1; }
    widen_bufsize(fd);
    if (connect(fd, res->ai_addr, res->ai_addrlen) < 0) {
        close(fd);
        freeaddrinfo(res);
        return -1;
    }
    freeaddrinfo(res);
    return fd;
}

int udp_bind_connect(const char *bind_ip, int bind_port, const char *host, int port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    widen_bufsize(fd);

    struct sockaddr_in la;
    memset(&la, 0, sizeof la);
    la.sin_family = AF_INET;
    la.sin_port = htons((uint16_t)bind_port);
    const char *bip = (!bind_ip || !*bind_ip) ? "0.0.0.0" : bind_ip;
    if (inet_pton(AF_INET, bip, &la.sin_addr) != 1) { close(fd); return -1; }
    if (bind(fd, (struct sockaddr *)&la, sizeof la) < 0) { close(fd); return -1; }

    char portstr[16];
    snprintf(portstr, sizeof portstr, "%d", port);
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof hints);
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(host, portstr, &hints, &res) != 0 || !res) { close(fd); return -1; }
    if (connect(fd, res->ai_addr, res->ai_addrlen) < 0) {
        close(fd);
        freeaddrinfo(res);
        return -1;
    }
    freeaddrinfo(res);
    return fd;
}

int udp_bind(const char *ip, int port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    widen_bufsize(fd);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((uint16_t)port);
    if (!ip || !*ip) ip = "0.0.0.0";
    if (inet_pton(AF_INET, ip, &a.sin_addr) != 1) { close(fd); return -1; }
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0) { close(fd); return -1; }
    return fd;
}

int udp_sendto(int fd, const void *buf, size_t len, const char *ip, int port)
{
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip, &a.sin_addr) != 1) return -1;
    return (int)sendto(fd, buf, len, 0, (struct sockaddr *)&a, sizeof a);
}

int udp_recvfrom(int fd, void *buf, size_t cap, char *src_ip, int *src_port)
{
    struct sockaddr_in a;
    socklen_t alen = sizeof a;
    int n = (int)recvfrom(fd, buf, cap, 0, (struct sockaddr *)&a, &alen);
    if (n < 0) return -1;
    if (src_ip) {
        if (!inet_ntop(AF_INET, &a.sin_addr, src_ip, INET_ADDRSTRLEN))
            src_ip[0] = 0;
    }
    if (src_port) *src_port = ntohs(a.sin_port);
    return n;
}
