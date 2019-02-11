#include <nRF24L01.h>

static nRF24L01 *isrObject0;
void nRF24L01_isrHandler0() { isrObject0->isrHandler(); }

static nRF24L01 *isrObject1;
void nRF24L01_isrHandler1() { isrObject1->isrHandler(); }

static nRF24L01 *isrObject2;
void nRF24L01_isrHandler2() { isrObject2->isrHandler(); }

static nRF24L01 *isrObject3;
void nRF24L01_isrHandler3() { isrObject3->isrHandler(); }

static nRF24L01 *isrObject4;
void nRF24L01_isrHandler4() { isrObject4->isrHandler(); }

static nRF24L01 *isrObject5;
void nRF24L01_isrHandler5() { isrObject5->isrHandler(); }

static nRF24L01 *isrObject6;
void nRF24L01_isrHandler6() { isrObject6->isrHandler(); }

static nRF24L01 *isrObject7;
void nRF24L01_isrHandler7() { isrObject7->isrHandler(); }

int nRF24L01::isrHandlerCounter = 0;

nRF24L01::nRF24L01(DGSPI &spi, int csn, int ce, int intr) {
    _spi = &spi;
    _csn = csn;
    _ce = ce;
    _intr = intr;
    _status = 0;
}

void nRF24L01::begin(uint8_t ad0, uint8_t ad1, uint8_t ad2, uint8_t ad3, uint8_t ad4, uint8_t chan, uint8_t width) {
    _spi->begin();
    _spi->setSpeed(10000000UL);
    _addr[0] = ad0;
    _addr[1] = ad1;
    _addr[2] = ad2;
    _addr[3] = ad3;
    _addr[4] = ad4;
    _pipeWidth = width;
    pinMode(_csn, OUTPUT);
    pinMode(_ce, OUTPUT);
    pinMode(_intr, INPUT);
    digitalWrite(_csn, HIGH);
    digitalWrite(_ce, HIGH);
    setChannel(chan);
    regSet(REG_CONFIG, 2);
    uint8_t zero = 0x00;
    regWrite(REG_EN_AA, &zero, 1);
    _bc[0] = 0xFF;
    _bc[1] = 0xFF;
    _bc[2] = 0xFF;
    _bc[3] = 0xFF;
    _bc[4] = 0xFF;
    enablePipe(0, _bc, false);
    enablePipe(1, _addr, true);
    selectRX();
    switch (isrHandlerCounter) {
        case 0:
            isrObject0 = this;
            attachInterrupt(_intr, nRF24L01_isrHandler0, FALLING);
            break;
        case 1:
            isrObject1 = this;
            attachInterrupt(_intr, nRF24L01_isrHandler1, FALLING);
            break;
        case 2:
            isrObject2 = this;
            attachInterrupt(_intr, nRF24L01_isrHandler2, FALLING);
            break;
        case 3:
            isrObject3 = this;
            attachInterrupt(_intr, nRF24L01_isrHandler3, FALLING);
            break;
        case 4:
            isrObject4 = this;
            attachInterrupt(_intr, nRF24L01_isrHandler4, FALLING);
            break;
        case 5:
            isrObject5 = this;
            attachInterrupt(_intr, nRF24L01_isrHandler5, FALLING);
            break;
        case 6:
            isrObject6 = this;
            attachInterrupt(_intr, nRF24L01_isrHandler6, FALLING);
            break;
        case 7:
            isrObject7 = this;
            attachInterrupt(_intr, nRF24L01_isrHandler7, FALLING);
            break;
    }
    isrHandlerCounter++;
    uint32_t s = disableInterrupts();
    digitalWrite(_csn, LOW);
    _status = _spi->transfer(CMD_TX_FLUSH);
    digitalWrite(_csn, HIGH);
    digitalWrite(_csn, LOW);
    _status = _spi->transfer(CMD_RX_FLUSH);
    digitalWrite(_csn, HIGH);
    restoreInterrupts(s);

    
    uint8_t isrstat = 0x70;
    regWrite(REG_STATUS, &isrstat, 1);
    isrstat = 0xFF; 
    regWrite(REG_SETUP_RETR, &isrstat, 1);

    enablePower();
}

void nRF24L01::regRead(uint8_t reg, uint8_t *buffer, uint8_t len) {
    uint32_t s = disableInterrupts();
    digitalWrite(_csn, LOW);
    _status = _spi->transfer(reg & 0x1F);
    for (int i = 0; i < len; i++) {
        buffer[i] = _spi->transfer(0xFF);
    }
    digitalWrite(_csn, HIGH);
    restoreInterrupts(s);
}

void nRF24L01::regWrite(uint8_t reg, uint8_t *buffer, uint8_t len) {
    uint32_t s = disableInterrupts();
    digitalWrite(_csn, LOW);
    _status = _spi->transfer((reg & 0x1F) | 0x20);
    for (int i = 0; i < len; i++) {
        _spi->transfer(buffer[i]);
    }
    digitalWrite(_csn, HIGH);
    restoreInterrupts(s);
}

void nRF24L01::regSet(uint8_t reg, uint8_t bit) {
    uint8_t val;
    regRead(reg, &val, 1);
    val |= (1<<bit);
    regWrite(reg, &val, 1);
}

void nRF24L01::regClr(uint8_t reg, uint8_t bit) {
    uint8_t val;
    regRead(reg, &val, 1);
    val &= ~(1<<bit);
    regWrite(reg, &val, 1);
}

void nRF24L01::selectRX() {
    _mode = 0;
    regSet(REG_CONFIG, 0);
    digitalWrite(_ce, HIGH);
}

void nRF24L01::selectTX() {
    if (_mode == 1) {
        return;
    }
    _mode = 1;
    regClr(REG_CONFIG, 0);
    digitalWrite(_ce, LOW);
}

void nRF24L01::enablePipe(int pipe, uint8_t *addr, boolean aa) {
    uint8_t pw = 0x03;
    regSet(REG_EN_RXADDR, pipe);
    regWrite(REG_SETUP_AW, &pw, 1);
    regWrite(REG_RX_ADDR_P0 + pipe, addr, 5);
    regWrite(REG_RX_PW_P0 + pipe, &_pipeWidth, 1);
    if (aa) {
        regSet(REG_EN_AA, pipe);
    } else {
        regClr(REG_EN_AA, pipe);
    }
    
}

void nRF24L01::enablePower() {
    regSet(REG_CONFIG, 1);
}

void nRF24L01::disablePower() {
    regClr(REG_CONFIG, 1);
}

void nRF24L01::isrHandler() {
    uint8_t isrstat = 0;
    uint8_t fifostat = 0;
    regRead(REG_STATUS, &isrstat, 1);
    regRead(REG_FIFO_STATUS, &fifostat, 1);


    // TX done
    if (isrstat & (1<<5)) {
        selectRX();
        enablePipe(0, _bc, false);
        enablePipe(1, _addr, true);
    }

    // Did we receive data?
    if (isrstat & (1<<6)) {
        // We're not actually going to do anything here.
    }

    if (isrstat & (1<<4)) {
        // Too many retries
        selectRX();
        enablePipe(0, _bc, false);
        enablePipe(1, _addr, true);
        uint32_t s = disableInterrupts();
        digitalWrite(_csn, LOW);
        _status = _spi->transfer(CMD_TX_FLUSH);
        digitalWrite(_csn, HIGH);
        restoreInterrupts(s);
    }

    // Clear interrupts
    isrstat = 0x70;
    regWrite(REG_STATUS, &isrstat, 1);
}

void nRF24L01::broadcastPacket(uint8_t *packet) {
    uint8_t stat = 0;
    regRead(REG_FIFO_STATUS, &stat, 1);

    uint32_t timeout = millis();
    while (_mode == 1 && millis() - timeout < 1000); // Wait for it to not be transmitting
    if (_mode == 1) {
        selectRX();
    }

    regWrite(REG_TX_ADDR, _bc, 5);
    enablePipe(0, _addr, false);
    enablePipe(1, _addr, false);
    uint8_t zero = 0x00;
    regWrite(REG_EN_AA, &zero, 1);

    selectTX();
    uint32_t s = disableInterrupts();
    digitalWrite(_csn, LOW);
    _status = _spi->transfer(CMD_TX);
    for (int i = 0; i < _pipeWidth; i++) {
        _spi->transfer(packet[i]);
    }
    digitalWrite(_csn, HIGH);
    restoreInterrupts(s);

    digitalWrite(_ce, HIGH);
    delayMicroseconds(20);
    digitalWrite(_ce, LOW);
}

void nRF24L01::unicastPacket(uint8_t *addr, uint8_t *packet) {
    uint8_t stat = 0;
    regRead(REG_FIFO_STATUS, &stat, 1);

    uint32_t timeout = millis();
    while (_mode == 1 && millis() - timeout < 1000); // Wait for it to not be transmitting
    if (_mode == 1) {
        selectRX();
    }

    regWrite(REG_TX_ADDR, addr, 5);
    enablePipe(0, addr, true);
    enablePipe(1, _addr, true);

    selectTX();
    uint32_t s = disableInterrupts();
    digitalWrite(_csn, LOW);
    _status = _spi->transfer(CMD_TX);
    for (int i = 0; i < _pipeWidth; i++) {
        _spi->transfer(packet[i]);
    }
    digitalWrite(_csn, HIGH);
    restoreInterrupts(s);

    digitalWrite(_ce, HIGH);
    delayMicroseconds(20);
    digitalWrite(_ce, LOW);
}

int nRF24L01::available() {
    uint8_t fifostat = 0;
    regRead(REG_FIFO_STATUS, &fifostat, 1);
    if (fifostat & (1<<0)) {
        return 0;
    }
    return 1;
}

void nRF24L01::readPacket(uint8_t *buffer) {
    uint32_t s = disableInterrupts();
    digitalWrite(_ce, LOW);
    digitalWrite(_csn, LOW);
    _status = _spi->transfer(CMD_RX);

    for (int i = 0; i < _pipeWidth; i++) {
        buffer[i] = _spi->transfer(0xFF);
    }
    digitalWrite(_csn, HIGH);
    digitalWrite(_ce, HIGH);
    restoreInterrupts(s);
}

uint8_t nRF24L01::getStatus() {
    return _status;
}

void nRF24L01::setChannel(uint8_t chan) {
    regWrite(REG_RF_CH, &chan, 1);
}

void nRF24L01::setDataRate(uint8_t mhz) {
    switch (mhz) {
        case RATE_1MHZ:
            regClr(REG_RF_SETUP, 3);
            break;
        case RATE_2MHZ:
            regSet(REG_RF_SETUP, 3);
            break;
    }
}

void nRF24L01::setTXPower(uint8_t power) {
    switch (power) {
        case RF_TX_18DBM:
            regClr(REG_RF_SETUP, 1);
            regClr(REG_RF_SETUP, 2);
            break;
        case RF_TX_12DBM:
            regSet(REG_RF_SETUP, 1);
            regClr(REG_RF_SETUP, 2);
            break;
        case RF_TX_6DBM:
            regSet(REG_RF_SETUP, 1);
            regClr(REG_RF_SETUP, 2);
            break;
        case RF_TX_0DBM:
            regSet(REG_RF_SETUP, 1);
            regSet(REG_RF_SETUP, 2);
            break;
    }
}

int nRF24L01::getHardwareAddress(uint8_t *buffer) {
    buffer[0] = _addr[0];
    buffer[1] = _addr[1];
    buffer[2] = _addr[2];
    buffer[3] = _addr[3];
    buffer[4] = _addr[4];
    return 5;
}
