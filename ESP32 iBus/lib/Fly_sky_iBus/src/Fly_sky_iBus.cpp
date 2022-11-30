#include "Fly_sky_iBus.h"

Fly_sky_iBus iBus;

void Fly_sky_iBus::begin(HardwareSerial& serial)
{
    serial.begin(115200, SERIAL_8N1, 26, 16); //RX TX
    begin((Stream&)serial);
}

void Fly_sky_iBus::begin(Stream& stream) {
    this->stream = &stream;
    this->state = DISCARD;
    this->last = millis();
    this->ptr = 0;
    this->len = 0;
    this->chksum = 0;
    this->lchksum = 0;
}

void Fly_sky_iBus::loop() {
    while (stream->available() > 0) {
        uint32_t now = millis();
        if (now - last >= PROTOCOL_TIMEGAP)
            state = GET_LENGTH;
        last = now;
        uint8_t value = stream->read();
        switch (state)
        {
        case GET_LENGTH:
            if (value <= PROTOCOL_LENGTH) {
                ptr = 0;
                len = value - PROTOCOL_OVERHEAD;
                chksum = 0xFFFF - value;
                state = GET_DATA;
            }
            else
                state = DISCARD;
            break;

        case GET_DATA:
            buffer[ptr++] = value;
            chksum -= value;
            if (ptr == len)
                state = GET_CHKSUML;
            break;
            
        case GET_CHKSUML:
            lchksum = value;
            state = GET_CHKSUMH;
            break;

        case GET_CHKSUMH:
            // Validate checksum
            if (chksum == (value << 8) + lchksum) {
                // Execute command - we only know command 0x40
                switch (buffer[0])
                {
                    case PROTOCOL_COMMAND40:
                        // Valid - extract channel data
                        for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2)
                            channel[i / 2] = buffer[i] | (buffer[i + 1] << 8);
                        break;
                    default:
                        break;
                }
            }
            state = DISCARD;
            break;
        case DISCARD:
        default:
            break;
        }
    }
}

uint16_t Fly_sky_iBus::readChannel(uint8_t channelNr) {
    if (channelNr < PROTOCOL_CHANNELS)
        return channel[channelNr];
    return 0;
}