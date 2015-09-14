#ifndef _L2_H
#define _L2_H

/* Layer 2 device interface */

class L2 {
    public:
        /*! Send a packet to a specific host */
        virtual void unicastPacket(uint8_t *addr, uint8_t *data) = 0;
        /*! Send a packet to all directly connected hosts */
        virtual void broadcastPacket(uint8_t *data) = 0;
        /*! Returns if (and maybe how many) any packets are available to read */
        virtual int available() = 0;
        /*! Read one packet into the buffer */
        virtual void readPacket(uint8_t *buffer) = 0;
        /*! Place the devices hardware address into buffer returning the length of the address */
        virtual int getHardwareAddress(uint8_t *buffer) = 0;
};

#endif
