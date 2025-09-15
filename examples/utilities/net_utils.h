#ifndef _NET_UTIL_H_
#define _NET_UTIL_H_

#ifdef __cplusplus
extern "C"
{
#endif
    int get_ip(char *devname, char *ipaddr);

    int get_ip_auto(char *ipaddr);
#ifdef __cplusplus
}
#endif
#endif