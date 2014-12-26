#ifndef __RFM12_H__
#define __RFM12_H__

/* ************************************************************************ */

#define RFM12_PKT_SYNC_SIZE         2
#define RFM12_PKT_HEADER_SIZE       4
#define RFM12_PKT_MAX_DATA_SIZE     42

struct rfm12_packet
{
    /* tx-only sync bytes */
    uint8_t sync[RFM12_PKT_SYNC_SIZE];

    /* Header */
    uint8_t dest_address;
    uint8_t source_address;
    uint8_t data_length;
    uint8_t header_checksum;

    /* Data */
    uint8_t data[RFM12_PKT_MAX_DATA_SIZE];
    uint16_t data_crc;
};

/* ************************************************************************ */

void                    rfm12_init      (uint8_t own_address);
void                    rfm12_tick      (uint8_t channel_free_time);

struct rfm12_packet *   rfm12_get_txpkt (void);
uint8_t                 rfm12_start_tx  (void);

struct rfm12_packet *   rfm12_get_rxpkt (void);
void                    rfm12_clear_rx  (void);

/* ************************************************************************ */

#endif /* __RFM12_H__ */
