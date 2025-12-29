#include "eth_custom_phy_interface.h"
#include "main.h"

/*
 * Эти функции уже реализованы в твоём ethernetif.c (CubeMX):
 *  - ETH_PHY_IO_Init()
 *  - ETH_PHY_IO_ReadReg()
 *  - ETH_PHY_IO_WriteReg()
 *  - ETH_PHY_IO_GetTick()
 */
extern int32_t ETH_PHY_IO_Init(void);
extern int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
extern int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
extern uint32_t ETH_PHY_IO_GetTick(void);

/* Standard IEEE PHY registers */
#define PHY_BMCR                (0x00U)  /* Basic Mode Control Register */
#define PHY_BMSR                (0x01U)  /* Basic Mode Status Register */
#define PHY_ID1                 (0x02U)  /* PHY Identifier 1 */
#define PHY_ID2                 (0x03U)  /* PHY Identifier 2 */
#define PHY_ANAR                (0x04U)  /* Auto-Negotiation Advertisement */
#define PHY_ANLPAR              (0x05U)  /* Auto-Negotiation Link Partner Ability */

/* BMCR bits */
#define BMCR_RESET              (1U << 15)
#define BMCR_AUTONEG_ENABLE     (1U << 12)
#define BMCR_RESTART_AUTONEG    (1U << 9)
#define BMCR_DUPLEX_MODE        (1U << 8)
#define BMCR_SPEED_SELECT       (1U << 13) /* 1=100M, 0=10M */

/* BMSR bits */
#define BMSR_LINK_STATUS        (1U << 2)
#define BMSR_AUTONEG_COMPLETE   (1U << 5)

/* ANAR/ANLPAR ability bits */
#define AN_10HALF               (1U << 5)
#define AN_10FULL               (1U << 6)
#define AN_100HALF              (1U << 7)
#define AN_100FULL              (1U << 8)

/* Timeouts */
#define PHY_RESET_TIMEOUT_MS    (200U)
#define PHY_AN_TIMEOUT_MS       (4000U)

/* Cached PHY address found by scan */
static uint32_t s_phyAddr = 0xFFFFFFFFU;

static int32_t phy_read(uint32_t reg, uint32_t *val)
{
    if (s_phyAddr == 0xFFFFFFFFU) return -1;
    return ETH_PHY_IO_ReadReg(s_phyAddr, reg, val);
}

static int32_t phy_write(uint32_t reg, uint32_t val)
{
    if (s_phyAddr == 0xFFFFFFFFU) return -1;
    return ETH_PHY_IO_WriteReg(s_phyAddr, reg, val);
}

/* Try to find a PHY on MDIO bus by scanning PHYID registers */
static int32_t phy_find_address(uint32_t *outAddr)
{
    uint32_t id1 = 0, id2 = 0;

    for (uint32_t addr = 0; addr < 32U; addr++)
    {
        if (ETH_PHY_IO_ReadReg(addr, PHY_ID1, &id1) != 0) continue;
        if (ETH_PHY_IO_ReadReg(addr, PHY_ID2, &id2) != 0) continue;

        /* Filter out invalid "no device" reads */
        if (id1 == 0x0000U || id1 == 0xFFFFU) continue;
        if (id2 == 0x0000U || id2 == 0xFFFFU) continue;

        *outAddr = addr;
        return 0;
    }
    return -1;
}

int32_t eth_phy_init(void)
{
    if (ETH_PHY_IO_Init() != 0)
    {
        return -1;
    }

    if (s_phyAddr == 0xFFFFFFFFU)
    {
        uint32_t found = 0;
        if (phy_find_address(&found) != 0)
        {
            return -1; /* No PHY detected */
        }
        s_phyAddr = found;
    }

    /* Reset PHY */
    uint32_t bmcr = 0;
    if (phy_read(PHY_BMCR, &bmcr) != 0)
    {
        return -1;
    }

    if (phy_write(PHY_BMCR, bmcr | BMCR_RESET) != 0)
    {
        return -1;
    }

    uint32_t t0 = ETH_PHY_IO_GetTick();
    do
    {
        if (phy_read(PHY_BMCR, &bmcr) != 0)
        {
            return -1;
        }
        if ((bmcr & BMCR_RESET) == 0U)
        {
            break;
        }
    } while ((ETH_PHY_IO_GetTick() - t0) < PHY_RESET_TIMEOUT_MS);

    if ((bmcr & BMCR_RESET) != 0U)
    {
        return -1; /* Reset timeout */
    }

    /* Enable autoneg + restart */
    if (phy_read(PHY_BMCR, &bmcr) != 0)
    {
        return -1;
    }
    bmcr |= (BMCR_AUTONEG_ENABLE | BMCR_RESTART_AUTONEG);
    if (phy_write(PHY_BMCR, bmcr) != 0)
    {
        return -1;
    }

    /*
     * Важно:
     * Не ждём здесь линк/AN до конца жёстко — этим занимается ethernet_link_thread().
     * Но мягко подождём чуть-чуть AN complete, чтобы первые секунды были стабильнее.
     */
    uint32_t bmsr = 0;
    t0 = ETH_PHY_IO_GetTick();
    do
    {
        if (phy_read(PHY_BMSR, &bmsr) != 0) break;
        /* read twice for latched bits correctness */
        (void)phy_read(PHY_BMSR, &bmsr);

        if ((bmsr & BMSR_AUTONEG_COMPLETE) != 0U)
        {
            break;
        }
    } while ((ETH_PHY_IO_GetTick() - t0) < PHY_AN_TIMEOUT_MS);

    return 0;
}

int32_t eth_phy_get_link_state(void)
{
    if (s_phyAddr == 0xFFFFFFFFU)
    {
        /* If someone calls before init, try minimal detection */
        uint32_t found = 0;
        if (phy_find_address(&found) != 0)
        {
            return ETH_PHY_STATUS_LINK_ERROR;
        }
        s_phyAddr = found;
    }

    uint32_t bmsr = 0;
    if (phy_read(PHY_BMSR, &bmsr) != 0)
    {
        return ETH_PHY_STATUS_LINK_ERROR;
    }
    /* read twice (BMSR link can be latched) */
    if (phy_read(PHY_BMSR, &bmsr) != 0)
    {
        return ETH_PHY_STATUS_LINK_ERROR;
    }

    if ((bmsr & BMSR_LINK_STATUS) == 0U)
    {
        return ETH_PHY_STATUS_LINK_DOWN;
    }

    uint32_t bmcr = 0;
    if (phy_read(PHY_BMCR, &bmcr) != 0)
    {
        return ETH_PHY_STATUS_LINK_ERROR;
    }

    const uint32_t autoneg = (bmcr & BMCR_AUTONEG_ENABLE);

    if (autoneg != 0U)
    {
        if ((bmsr & BMSR_AUTONEG_COMPLETE) == 0U)
        {
            return ETH_PHY_STATUS_AUTONEGO_NOT_DONE;
        }

        uint32_t anar = 0, anlpar = 0;
        if (phy_read(PHY_ANAR, &anar) != 0)   return ETH_PHY_STATUS_LINK_ERROR;
        if (phy_read(PHY_ANLPAR, &anlpar) != 0) return ETH_PHY_STATUS_LINK_ERROR;

        uint32_t common = (anar & anlpar);

        /* Highest priority first */
        if ((common & AN_100FULL) != 0U) return ETH_PHY_STATUS_100MBITS_FULLDUPLEX;
        if ((common & AN_100HALF) != 0U) return ETH_PHY_STATUS_100MBITS_HALFDUPLEX;
        if ((common & AN_10FULL)  != 0U) return ETH_PHY_STATUS_10MBITS_FULLDUPLEX;
        if ((common & AN_10HALF)  != 0U) return ETH_PHY_STATUS_10MBITS_HALFDUPLEX;

        /* If common is empty (редко, но бывает), fallback to BMCR forced bits */
        uint32_t speed100 = (bmcr & BMCR_SPEED_SELECT);
        uint32_t duplex   = (bmcr & BMCR_DUPLEX_MODE);
        if (speed100 != 0U)
        {
            return (duplex != 0U) ? ETH_PHY_STATUS_100MBITS_FULLDUPLEX : ETH_PHY_STATUS_100MBITS_HALFDUPLEX;
        }
        else
        {
            return (duplex != 0U) ? ETH_PHY_STATUS_10MBITS_FULLDUPLEX : ETH_PHY_STATUS_10MBITS_HALFDUPLEX;
        }
    }
    else
    {
        /* Forced mode (no autoneg) */
        uint32_t speed100 = (bmcr & BMCR_SPEED_SELECT);
        uint32_t duplex   = (bmcr & BMCR_DUPLEX_MODE);

        if (speed100 != 0U)
        {
            return (duplex != 0U) ? ETH_PHY_STATUS_100MBITS_FULLDUPLEX : ETH_PHY_STATUS_100MBITS_HALFDUPLEX;
        }
        else
        {
            return (duplex != 0U) ? ETH_PHY_STATUS_10MBITS_FULLDUPLEX : ETH_PHY_STATUS_10MBITS_HALFDUPLEX;
        }
    }
}
