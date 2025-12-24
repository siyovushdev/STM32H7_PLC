#include "eth_custom_phy_interface.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "logf.h"

extern ETH_HandleTypeDef heth;

#define ETH_LOG_TAG "ETH"

// На таких LAN8720 модулях почти всегда PHY addr = 0.
// Если PHY ID читается как 0xFFFF/0xFFFF — попробуй 1.
#ifndef LAN8720_PHY_ADDR
#define LAN8720_PHY_ADDR  1
#endif

// MII registers
#define PHY_BCR           0x00
#define PHY_BSR           0x01
#define PHY_ID1           0x02
#define PHY_ID2           0x03

// BCR bits
#define BCR_RESET         (1U << 15)
#define BCR_AUTONEG_EN    (1U << 12)
#define BCR_RESTART_AN    (1U << 9)

// BSR bits
#define BSR_LINK_STATUS   (1U << 2)

// LAN8720 specific: PHY Special Control/Status Register (PHYSCSR)
#define LAN8720_PHYSCSR   0x1F

static int mdio_read(uint32_t reg, uint32_t *val)
{
    return (HAL_ETH_ReadPHYRegister(&heth, LAN8720_PHY_ADDR, reg, val) == HAL_OK) ? 0 : -1;
}

static int mdio_write(uint32_t reg, uint32_t val)
{
    return (HAL_ETH_WritePHYRegister(&heth, LAN8720_PHY_ADDR, reg, val) == HAL_OK) ? 0 : -1;
}

int32_t eth_phy_init(void)
{
    uint32_t id1 = 0, id2 = 0;
    int found = -1;

    for (int addr = 0; addr < 32; addr++) {
        if (HAL_ETH_ReadPHYRegister(&heth, addr, PHY_ID1, &id1) != HAL_OK) continue;
        if (HAL_ETH_ReadPHYRegister(&heth, addr, PHY_ID2, &id2) != HAL_OK) continue;

        if (id1 != 0xFFFFu && id1 != 0x0000u) {
            LOGT("ETH", "PHY found addr=%d ID1=0x%04lX ID2=0x%04lX", addr, id1, id2);
            found = addr;
            break;
        }
    }

    if (found < 0) {
        LOGT("ETH", "PHY NOT FOUND via MDIO (all addrs invalid)");
        return ETH_PHY_STATUS_LINK_DOWN;
    }


    return ETH_PHY_STATUS_LINK_DOWN; // фактический линк проверяется в ethernet_link_thread()
}

int32_t eth_phy_get_link_state(void)
{
    uint32_t bsr = 0;

    if (mdio_read(PHY_BSR, &bsr) != 0) {
        return ETH_PHY_STATUS_LINK_DOWN;
    }

    // ВАЖНО: 0xFFFF/0x0000 = недостоверное чтение (MDIO не работает)
    if (bsr == 0xFFFFu || bsr == 0x0000u) {
        return ETH_PHY_STATUS_LINK_DOWN;
    }

    if ((bsr & BSR_LINK_STATUS) == 0) {
        return ETH_PHY_STATUS_LINK_DOWN;
    }

    uint32_t scsr = 0;
    if (mdio_read(LAN8720_PHYSCSR, &scsr) != 0) {
        return ETH_PHY_STATUS_100MBITS_FULLDUPLEX;
    }

    if (scsr == 0xFFFFu || scsr == 0x0000u) {
        // режим не смогли прочитать — но линк есть
        return ETH_PHY_STATUS_100MBITS_FULLDUPLEX;
    }

    uint32_t mode = (scsr >> 2) & 0x7U;
    switch (mode) {
        case 0x1: return ETH_PHY_STATUS_10MBITS_HALFDUPLEX;
        case 0x2: return ETH_PHY_STATUS_10MBITS_FULLDUPLEX;
        case 0x5: return ETH_PHY_STATUS_100MBITS_HALFDUPLEX;
        case 0x6: return ETH_PHY_STATUS_100MBITS_FULLDUPLEX;
        default:  return ETH_PHY_STATUS_100MBITS_FULLDUPLEX;
    }
}

