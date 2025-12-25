#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Статусы такие же как у Cube-шаблона */
#define  ETH_PHY_STATUS_LINK_DOWN            ((int32_t) 0)
#define  ETH_PHY_STATUS_10MBITS_FULLDUPLEX   ((int32_t) 1)
#define  ETH_PHY_STATUS_10MBITS_HALFDUPLEX   ((int32_t) 2)
#define  ETH_PHY_STATUS_100MBITS_FULLDUPLEX  ((int32_t) 3)
#define  ETH_PHY_STATUS_100MBITS_HALFDUPLEX  ((int32_t) 4)
#define  ETH_PHY_STATUS_AUTONEGO_NOT_DONE    ((int32_t) 6)
#define  ETH_PHY_STATUS_LINK_ERROR           ((int32_t)-1)

int32_t eth_phy_init(void);
int32_t eth_phy_get_link_state(void);

#ifdef __cplusplus
}
#endif
