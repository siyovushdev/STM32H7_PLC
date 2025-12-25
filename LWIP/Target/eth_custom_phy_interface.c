#include "eth_custom_phy_interface.h"
#include "ethernetif.h"   // тут обычно объявлены ETH_PHY_IO_* функции
#include <stdint.h>

/* Clause 22 стандартные регистры */
#define PHY_REG_BMCR     0x00u
#define PHY_REG_BMSR     0x01u
#define PHY_REG_PHYID1   0x02u
#define PHY_REG_PHYID2   0x03u

/* LAN8720: PHY Special Control/Status (PSCSR) = 0x1F */
#define LAN8720_REG_PSCSR 0x1Fu

/* BMCR bits */
#define BMCR_RESET       (1u << 15)
#define BMCR_AN_EN       (1u << 12)
#define BMCR_RESTART_AN  (1u << 9)

/* BMSR bits */
#define BMSR_LINK_STATUS (1u << 2)
#define BMSR_AN_COMPLETE (1u << 5)

/* PSCSR HCDSPEED field (как у Oryx для LAN8720) */
#define PSCSR_HCDSPEED_MASK        0x001Cu
#define PSCSR_10BT_HD              0x0004u
#define PSCSR_100BTX_HD            0x0008u
#define PSCSR_10BT_FD              0x0014u
#define PSCSR_100BTX_FD            0x0018u

/* Если ты не уверен в PHY адресе — включи автоскан */
#define PHY_ADDR_AUTOSCAN 1

static uint32_t g_phyAddr = 0xFFFFFFFFu;

static int phy_read(uint32_t addr, uint32_t reg, uint32_t *val)
{
    return (ETH_PHY_IO_ReadReg(addr, reg, val) == 0) ? 0 : -1;
}

static int phy_write(uint32_t addr, uint32_t reg, uint32_t val)
{
    return (ETH_PHY_IO_WriteReg(addr, reg, val) == 0) ? 0 : -1;
}

static uint32_t ms_now(void)
{
    return (uint32_t)ETH_PHY_IO_GetTick();
}

static int scan_phy_addr(uint32_t *outAddr)
{
    for (uint32_t a = 0; a < 32; a++)
    {
        uint32_t id1 = 0, id2 = 0;
        if (phy_read(a, PHY_REG_PHYID1, &id1) == 0 &&
            phy_read(a, PHY_REG_PHYID2, &id2) == 0)
        {
            if (id1 != 0x0000u && id1 != 0xFFFFu)
            {
                *outAddr = a;
                return 0;
            }
        }
    }
    return -1;
}

int32_t eth_phy_init(void)
{
    if (ETH_PHY_IO_Init() != 0)
        return -1;

#if PHY_ADDR_AUTOSCAN
    if (scan_phy_addr(&g_phyAddr) != 0)
        return -1;
#else
    /* Если хочешь фиксированный адрес — задай тут (обычно 0 или 1) */
  g_phyAddr = 0;
#endif

    /* Soft reset */
    uint32_t bmcr = 0;
    if (phy_read(g_phyAddr, PHY_REG_BMCR, &bmcr) != 0) return -1;
    if (phy_write(g_phyAddr, PHY_REG_BMCR, bmcr | BMCR_RESET) != 0) return -1;

    uint32_t t0 = ms_now();
    while (1)
    {
        if (phy_read(g_phyAddr, PHY_REG_BMCR, &bmcr) != 0) return -1;
        if ((bmcr & BMCR_RESET) == 0) break;
        if ((ms_now() - t0) > 500u) return -1;
    }

    /* Enable autoneg + restart */
    if (phy_read(g_phyAddr, PHY_REG_BMCR, &bmcr) != 0) return -1;
    bmcr |= (BMCR_AN_EN | BMCR_RESTART_AN);
    if (phy_write(g_phyAddr, PHY_REG_BMCR, bmcr) != 0) return -1;

    return 0;
}

int32_t eth_phy_get_link_state(void)
{
    if (g_phyAddr == 0xFFFFFFFFu)
        return ETH_PHY_STATUS_LINK_ERROR;

    uint32_t bmsr1 = 0, bmsr2 = 0;

    /* ВАЖНО: BMSR читаем два раза (latched) — как в Oryx :contentReference[oaicite:4]{index=4} */
    if (phy_read(g_phyAddr, PHY_REG_BMSR, &bmsr1) != 0) return ETH_PHY_STATUS_LINK_ERROR;
    if (phy_read(g_phyAddr, PHY_REG_BMSR, &bmsr2) != 0) return ETH_PHY_STATUS_LINK_ERROR;

    if ((bmsr2 & BMSR_LINK_STATUS) == 0)
        return ETH_PHY_STATUS_LINK_DOWN;

    uint32_t pscsr = 0;
    if (phy_read(g_phyAddr, LAN8720_REG_PSCSR, &pscsr) != 0)
        return ETH_PHY_STATUS_LINK_ERROR;

    switch (pscsr & PSCSR_HCDSPEED_MASK)
    {
        case PSCSR_100BTX_FD: return ETH_PHY_STATUS_100MBITS_FULLDUPLEX;
        case PSCSR_100BTX_HD: return ETH_PHY_STATUS_100MBITS_HALFDUPLEX;
        case PSCSR_10BT_FD:   return ETH_PHY_STATUS_10MBITS_FULLDUPLEX;
        case PSCSR_10BT_HD:   return ETH_PHY_STATUS_10MBITS_HALFDUPLEX;
        default:              return ETH_PHY_STATUS_AUTONEGO_NOT_DONE;
    }
}
