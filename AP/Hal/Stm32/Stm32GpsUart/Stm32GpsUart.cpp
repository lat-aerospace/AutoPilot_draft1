// ======================================================================
// \title  AP/Hal/Stm32/Stm32GpsUart/Stm32GpsUart.cpp
// \brief  NEO-M9N UBX-NAV-PVT GPS parser + initialization
// ======================================================================
#include <AP/Hal/Stm32/Stm32GpsUart/Stm32GpsUart.hpp>
#include <Fw/Types/Assert.hpp>
#include <cstring>
#include <cmath>

namespace Ap {

Stm32GpsUart::Stm32GpsUart(const char* const compName)
    : Stm32GpsUartComponentBase(compName) {
    (void)memset(m_rx_buf, 0, sizeof(m_rx_buf));
}

// ---------------------------------------------------------------------------
// configure + initGps + startRx
// ---------------------------------------------------------------------------
void Stm32GpsUart::configure(UART_HandleTypeDef* huart) {
    m_huart = huart;
}

void Stm32GpsUart::sendUbxCommand(uint8_t cls, uint8_t id,
                                    const uint8_t* payload, uint16_t len) {
    // Build UBX frame: sync + class + id + length(LE) + payload + CK_A + CK_B
    uint8_t header[6] = {
        UBX_SYNC_1, UBX_SYNC_2,
        cls, id,
        static_cast<uint8_t>(len & 0xFFU),
        static_cast<uint8_t>((len >> 8U) & 0xFFU)
    };

    // Compute Fletcher-8 checksum over class+id+length+payload
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; i++) {
        ck_a += header[i];
        ck_b += ck_a;
    }
    for (uint16_t i = 0; i < len; i++) {
        ck_a += payload[i];
        ck_b += ck_a;
    }

    HAL_UART_Transmit(m_huart, header, 6, 100);
    if (len > 0U && payload != nullptr) {
        HAL_UART_Transmit(m_huart, const_cast<uint8_t*>(payload), len, 100);
    }
    uint8_t ck[2] = { ck_a, ck_b };
    HAL_UART_Transmit(m_huart, ck, 2, 100);
}

void Stm32GpsUart::initGps() {
    FW_ASSERT(m_huart != nullptr);

    // NEO-M9N factory default is 38400 baud. Send baud rate change to 57600.
    // UBX-CFG-PRT (class=0x06, id=0x00): configure UART port
    uint8_t cfg_prt[20] = {};
    cfg_prt[0] = 0x01U;                           // Port ID = UART1
    cfg_prt[4] = 0xC0U; cfg_prt[5] = 0x08U;       // charLen=8bit, parity=none, stopBits=1
    // Baud rate = 57600 = 0x0000E100 (little-endian)
    cfg_prt[8]  = 0x00U;
    cfg_prt[9]  = 0xE1U;
    cfg_prt[10] = 0x00U;
    cfg_prt[11] = 0x00U;
    // inProtoMask = UBX only (bit0)
    cfg_prt[12] = 0x01U;
    // outProtoMask = UBX only (bit0)
    cfg_prt[14] = 0x01U;
    sendUbxCommand(0x06U, 0x00U, cfg_prt, 20);
    HAL_Delay(100);

    // Reinitialize UART at 57600
    m_huart->Init.BaudRate = 57600;
    HAL_UART_Init(m_huart);
    HAL_Delay(100);

    // Disable all NMEA messages: UBX-CFG-MSG for each NMEA class/id
    // NMEA standard messages: GGA, GLL, GSA, GSV, RMC, VTG
    static const uint8_t nmea_ids[][2] = {
        {0xF0U, 0x00U},  // GGA
        {0xF0U, 0x01U},  // GLL
        {0xF0U, 0x02U},  // GSA
        {0xF0U, 0x03U},  // GSV
        {0xF0U, 0x04U},  // RMC
        {0xF0U, 0x05U},  // VTG
    };
    for (const auto& nid : nmea_ids) {
        uint8_t cfg_msg[8] = {};
        cfg_msg[0] = nid[0];  // class
        cfg_msg[1] = nid[1];  // id
        // rate[0..5] = 0 → disabled on all ports
        sendUbxCommand(0x06U, 0x01U, cfg_msg, 8);
        HAL_Delay(10);
    }

    // Enable UBX-NAV-PVT on UART port
    {
        uint8_t cfg_msg[8] = {};
        cfg_msg[0] = 0x01U;  // NAV class
        cfg_msg[1] = 0x07U;  // PVT id
        cfg_msg[2] = 0x01U;  // rate on port 1
        sendUbxCommand(0x06U, 0x01U, cfg_msg, 8);
        HAL_Delay(10);
    }

    // Set measurement rate: 10 Hz → measRate=100ms
    // UBX-CFG-RATE (class=0x06, id=0x08)
    {
        uint8_t cfg_rate[6] = {};
        cfg_rate[0] = 0x64U; cfg_rate[1] = 0x00U;  // measRate = 100 ms (LE)
        cfg_rate[2] = 0x01U; cfg_rate[3] = 0x00U;  // navRate = 1
        cfg_rate[4] = 0x01U; cfg_rate[5] = 0x00U;  // timeRef = GPS
        sendUbxCommand(0x06U, 0x08U, cfg_rate, 6);
        HAL_Delay(10);
    }

    // Save configuration: UBX-CFG-CFG
    {
        uint8_t cfg_cfg[13] = {};
        // saveMask = all (bytes 4–7)
        cfg_cfg[4] = 0xFFU; cfg_cfg[5] = 0xFFU;
        cfg_cfg[6] = 0x00U; cfg_cfg[7] = 0x00U;
        // deviceMask = BBR + Flash
        cfg_cfg[12] = 0x17U;
        sendUbxCommand(0x06U, 0x09U, cfg_cfg, 13);
        HAL_Delay(100);
    }
}

void Stm32GpsUart::startRx() {
    FW_ASSERT(m_huart != nullptr);
    HAL_UART_Receive_DMA(m_huart, m_rx_buf,
                         static_cast<uint16_t>(RX_BUF_SIZE));
    m_initialized = true;
}

// ---------------------------------------------------------------------------
// Ring buffer helpers
// ---------------------------------------------------------------------------
uint16_t Stm32GpsUart::available() const {
    const uint16_t ndtr  = static_cast<uint16_t>(
        __HAL_DMA_GET_COUNTER(m_huart->hdmarx));
    const uint16_t write = static_cast<uint16_t>(RX_BUF_SIZE - ndtr);
    if (write >= m_rx_read) { return write - m_rx_read; }
    return static_cast<uint16_t>(RX_BUF_SIZE - m_rx_read + write);
}

uint8_t Stm32GpsUart::peek(uint16_t offset) const {
    return m_rx_buf[(m_rx_read + offset) % RX_BUF_SIZE];
}

void Stm32GpsUart::advance(uint16_t n) {
    m_rx_read = static_cast<uint16_t>((m_rx_read + n) % RX_BUF_SIZE);
}

// ---------------------------------------------------------------------------
// UBX Fletcher-8 checksum
// ---------------------------------------------------------------------------
bool Stm32GpsUart::checksum(const uint8_t* payload, uint16_t len,
                              uint8_t expected_a, uint8_t expected_b) {
    uint8_t ck_a = 0, ck_b = 0;
    for (uint16_t i = 0; i < len; i++) {
        ck_a += payload[i];
        ck_b += ck_a;
    }
    return (ck_a == expected_a) && (ck_b == expected_b);
}

// ---------------------------------------------------------------------------
// parseNavPvt – scan ring buffer for a complete UBX-NAV-PVT frame
// ---------------------------------------------------------------------------
bool Stm32GpsUart::parseNavPvt(Ap::GpsData& out) {
    // Total UBX frame size: 2 sync + 1 class + 1 id + 2 len + payload + 2 ck
    static constexpr uint16_t FRAME_LEN =
        2U + 1U + 1U + 2U + UBX_PVT_LEN + 2U;  // = 100 bytes

    // Scan for valid sync pair
    while (available() >= FRAME_LEN) {
        if (peek(0) != UBX_SYNC_1 || peek(1) != UBX_SYNC_2) {
            advance(1U);
            continue;
        }
        if (peek(2) != UBX_CLASS_NAV || peek(3) != UBX_ID_PVT) {
            advance(1U);
            continue;
        }
        // Check length field (LE U16)
        const uint16_t payload_len =
            static_cast<uint16_t>(peek(4)) |
            (static_cast<uint16_t>(peek(5)) << 8U);
        if (payload_len != UBX_PVT_LEN) {
            advance(1U);
            continue;
        }

        // Extract payload into a contiguous buffer (handle ring-buffer wrap)
        uint8_t frame[FRAME_LEN];
        for (uint16_t i = 0; i < FRAME_LEN; i++) {
            frame[i] = peek(i);
        }

        // Verify checksum: covers class+id+length+payload (bytes 2..97)
        const uint8_t ck_a = frame[FRAME_LEN - 2U];
        const uint8_t ck_b = frame[FRAME_LEN - 1U];
        if (!checksum(&frame[2], 4U + UBX_PVT_LEN, ck_a, ck_b)) {
            advance(1U);
            continue;
        }

        // Valid frame – decode payload (starts at frame[6])
        const uint8_t* p = &frame[6];

        // lon/lat: I4 in units of 1e-7 degrees
        int32_t lon_raw, lat_raw, height_msl;
        int32_t velN_raw, velE_raw, velD_raw;
        (void)memcpy(&lon_raw,     &p[24], 4);
        (void)memcpy(&lat_raw,     &p[28], 4);
        (void)memcpy(&height_msl,  &p[36], 4);
        (void)memcpy(&velN_raw,    &p[48], 4);
        (void)memcpy(&velE_raw,    &p[52], 4);
        (void)memcpy(&velD_raw,    &p[56], 4);

        const double lat_deg  = lat_raw * 1.0e-7;
        const double lon_deg  = lon_raw * 1.0e-7;
        const double alt_m    = height_msl * 1.0e-3;

        // Populate GpsData
        out.set_lat_deg(lat_deg);
        out.set_lon_deg(lon_deg);
        out.set_alt_msl_m(alt_m);

        // Use velN/velE/velD directly from NAV-PVT (NEO-M9N provides these)
        Ap::Vec3 vel_ned;
        vel_ned.set_x(static_cast<float>(velN_raw) * 1.0e-3F);
        vel_ned.set_y(static_cast<float>(velE_raw) * 1.0e-3F);
        vel_ned.set_z(static_cast<float>(velD_raw) * 1.0e-3F);
        out.set_vel_ned_mps(vel_ned);

        advance(FRAME_LEN);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// schedIn_handler – 10 Hz drain + parse
// ---------------------------------------------------------------------------
void Stm32GpsUart::schedIn_handler(FwIndexType /*portNum*/,
                                    U32 /*context*/) {
    if (!m_initialized) { return; }

    Ap::GpsData gps;
    if (parseNavPvt(gps)) {
        gpsOut_out(0, gps);
    }
}

}  // namespace Ap
