#ifndef _MESH_H
#define _MESH_H

#include <Arduino.h>
#include <L2.h>

/* The mesh class defines a layer three mesh system */

struct device {
    L2 *dev;
    struct device *next;
};

struct host {
    uint16_t id;
    uint8_t *hwaddr;
    L2 *device;
    uint16_t nexthop;
    uint8_t cost;
    uint32_t lastseen;
    struct host *next;
};

struct packet {
    union {
        struct {
            uint16_t sender;
            uint16_t receiver;
            uint8_t type;
            uint8_t ttl;
            uint8_t datalen;
            uint8_t data[24];
            uint8_t csum;
        } __attribute__((packed));
        uint8_t bytes[32];
    };
} __attribute__((packed));

class Mesh : public Printable {
    public: // Constants
        static const uint16_t Broadcast = 0xFFFF;
        static const uint16_t Direct = 0x0000;
        static const uint8_t  MTU = 24;

        // Packet times 0xF0 to 0xFF are reserved for
        // system use.  The user can use packet types
        // below 0xF0.
        static const uint8_t IAM  = 0xF0; // I am this ID
        static const uint8_t ICAN = 0xF1; // I can route to these IDs

    private:

        uint8_t _ledpin;
        struct device *_devlist;
        struct host *_hostlist;
        uint16_t _id;
        void (*_broadcastCallback)(uint16_t, uint8_t, uint8_t *, uint8_t);
        void (*_unicastCallback)(uint16_t, uint8_t, uint8_t *, uint8_t);
        uint32_t _lastMGMTSend;

        void sendIAM();
        void sendICAN();
        void deleteHost(struct host *hst);
        void expireHosts();
        struct host *getHost(uint16_t id, uint16_t nexthop);

        void housekeeping() {
            expireHosts();
        }

        void addHostFromPacket(struct packet *pkt, L2 *dev) {
            addRoute(pkt->sender, Direct, 1, dev, pkt->datalen, pkt->data);
        }

        void addRoute(uint16_t id, uint16_t nexthop, uint8_t cost, L2 *dev, uint8_t hwlen, uint8_t *hwaddr);
        void addRoutesFromPacket(struct packet *pkt, L2 *dev);


        struct host *getLeastCostRoute(uint16_t dest);
        void processPacket(struct packet *pkt, L2 *dev);
        void receivePackets();
        void sendManagementData();
        void calcCS(struct packet *p);
        uint8_t checkCS(struct packet *p);


    public:


        Mesh() : _ledpin(255), _hostlist(NULL), _devlist(NULL), _id(65535) {}

        void addDevice(L2 &dev);
        void removeDevice(L2 &dev) { } // todo
        boolean sendPacket(int destination, uint8_t type, uint8_t *data, int len);
        boolean knowHost(uint16_t id);
        size_t printTo(Print &p) const;

        void setLEDPin(uint8_t p) { _ledpin = p; pinMode(_ledpin, OUTPUT); }

        void setID(uint16_t id) {
            if (id == Direct || id == Broadcast) {
                return;
            }
            _id = id;
            sendIAM();
        }

        void process() {
            housekeeping();
            receivePackets();
            sendManagementData();
        }
            
        void addUnicastCallback(void (*func)(uint16_t, uint8_t, uint8_t *, uint8_t)) {
            _unicastCallback = func;
        }

        void addBroadcastCallback(void (*func)(uint16_t, uint8_t, uint8_t *, uint8_t)) {
            _broadcastCallback = func;
        }


};

#endif
