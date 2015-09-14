
#include <Mesh.h>

void Mesh::sendIAM() {
    for (struct device *d = _devlist; d; d = d->next) {
        struct packet pkt;
        pkt.sender = _id;
        pkt.receiver = Broadcast;
        pkt.type = IAM;
        pkt.ttl = 1; // Never forward
        pkt.datalen = d->dev->getHardwareAddress(pkt.data);
        calcCS(&pkt);
        d->dev->broadcastPacket((uint8_t *)&pkt);
    }
}

void Mesh::sendICAN() {
    for (struct device *d = _devlist; d; d = d->next) {
        struct packet pkt = {_id, Broadcast, ICAN, 1, 0, 0};
        for (struct host *h = _hostlist; h; h = h->next) {
            pkt.data[pkt.datalen++] = h->id >> 8;
            pkt.data[pkt.datalen++] = h->id & 0xFF;
            pkt.data[pkt.datalen++] = h->cost + 1;
            if (pkt.datalen == 24) {
            calcCS(&pkt);
                d->dev->broadcastPacket((uint8_t *)&pkt);
                pkt.datalen = 0;
            }
        }
        if (pkt.datalen > 0) {
            calcCS(&pkt);
            d->dev->broadcastPacket((uint8_t *)&pkt);
        }
    }
}

void Mesh::deleteHost(struct host *hst) {
    if (_hostlist == hst) {
        _hostlist = _hostlist->next;
        free(hst->hwaddr);
        free(hst);
        return;
    }
    for (struct host *h = _hostlist; h->next; h = h->next) {
        if (h->next == hst) {
            h->next = hst->next;
            free(hst->hwaddr);
            free(hst);
            return;
        }
    }
}

void Mesh::expireHosts() {
    for (struct host *h = _hostlist; h; h = h->next) {
        if (millis() - h->lastseen > 30000) {
            deleteHost(h);
            return;
        }
    }
}

struct host *Mesh::getHost(uint16_t id, uint16_t nexthop) {
    for (struct host *h = _hostlist; h; h = h->next) {
        if (h->id == id && h->nexthop == nexthop) {
            return h;
        }
    }
    return NULL;
}

void Mesh::addRoute(uint16_t id, uint16_t nexthop, uint8_t cost, L2 *dev, uint8_t hwlen, uint8_t *hwaddr) {
    // Point blank refuse to add myself!

    if (id == _id) {
        return;
    }

    // Not interested in routes we're directly connected to
    if (nexthop != Direct) {
        struct host *direct = getHost(id, Direct);
        if (direct != NULL) {
            return;
        }
    }

    struct host *exist = getHost(id, nexthop);
    if (exist != NULL) {
        if (nexthop == Direct) {
            exist->hwaddr = (uint8_t *)realloc(exist->hwaddr, hwlen);
            memcpy(exist->hwaddr, hwaddr, hwlen);
        }
        exist->device = dev;
        exist->nexthop = nexthop;
        exist->cost = cost;
        exist->lastseen = millis();
        return;
    }

    struct host *newhost = (struct host *)malloc(sizeof(struct host));
    newhost->id = id;
    newhost->device = dev;
    newhost->lastseen = millis();
    newhost->nexthop = nexthop;
    newhost->cost = cost;
    if (nexthop == Direct) {
        newhost->hwaddr = (uint8_t *)malloc(hwlen);
        memcpy(newhost->hwaddr, hwaddr, hwlen);
    }
    newhost->next = NULL;
    
    if (_hostlist == NULL) {
        _hostlist = newhost;
    } else {
        struct host *h = _hostlist;
        while (h->next) {
            h = h->next;
        }
        h->next = newhost;
    }
}

void Mesh::addRoutesFromPacket(struct packet *pkt, L2 *dev) {
    for (int i = 0; i < pkt->datalen; i += 3) {
        uint16_t id = (pkt->data[i] << 8) | pkt->data[i+1];
        uint8_t cost = pkt->data[i+2];

        addRoute(id, pkt->sender, cost, dev, 0, NULL);
    }
}

struct host *Mesh::getLeastCostRoute(uint16_t dest) {
    struct host *least = NULL;
    uint8_t leastcost = 255;
    for (struct host *h = _hostlist; h; h = h->next) {
        if (h->id == dest) {
            if (h->cost < leastcost) {
                leastcost = h->cost;
                least = h;
            }
        }
    }
    if (least->nexthop == Direct) {
        return least;
    }
    
    for (struct host *h = _hostlist; h; h = h->next) {
        if (h->id == least->nexthop && h->nexthop == Direct) {
            return h;
        }
    }
    return NULL;
}

void Mesh::processPacket(struct packet *pkt, L2 *dev) {
    if (_ledpin != 255) { digitalWrite(_ledpin, HIGH); }

    if (pkt->receiver == Broadcast) {
        switch (pkt->type) {
            case IAM:
                addHostFromPacket(pkt, dev);
                break;
            case ICAN:
                addRoutesFromPacket(pkt, dev);
                break;
            default:
                if (_broadcastCallback) {
                    _broadcastCallback(pkt->sender, pkt->type, pkt->data, pkt->datalen);
                }
        }
    } else {
        if (pkt->receiver != _id) {
            pkt->ttl--;
            if (pkt->ttl > 0) {
                struct host *hop = getLeastCostRoute(pkt->receiver);
                if (hop != NULL) {
                    // Need to recalculate checksum after reducing TTL
                    calcCS(pkt);

                    hop->device->unicastPacket(hop->hwaddr, (uint8_t *)pkt);
                }
            }
        } else {
            switch (pkt->type) {
                default:
                    if (_unicastCallback) {
                        _unicastCallback(pkt->sender, pkt->type, pkt->data, pkt->datalen);
                    }
            }
        }
    }
    if (_ledpin != 255) { digitalWrite(_ledpin, LOW); }
}

void Mesh::receivePackets() {
    for (struct device *d = _devlist; d; d = d->next) {
        if (d->dev->available()) {
            struct packet pkt;
            d->dev->readPacket((uint8_t *)&pkt);
            if (checkCS(&pkt)) {
                processPacket(&pkt, d->dev);
            }
        }
    }
}

void Mesh::sendManagementData() {
    if (millis() - _lastMGMTSend > 5000) {
        _lastMGMTSend = millis();
        if (_id != Direct && _id != Broadcast) {
            sendIAM();
            sendICAN();
        }
    }
}

void Mesh::addDevice(L2 &dev) {
    for (struct device *d = _devlist; d; d = d->next) {
        if (d->dev == &dev) {
            return;
        }
    }
    struct device *newdev = (struct device *)malloc(sizeof(struct device));
    newdev->dev = &dev;
    newdev->next = NULL;
    if (_devlist == NULL) {
        _devlist = newdev;
    } else {
        struct device *d = _devlist;
        while (d->next) {
            d = d->next;
        }
        d->next = newdev;
    }
}

boolean Mesh::sendPacket(int destination, uint8_t type, uint8_t *data, int len) {
    struct host *h = getLeastCostRoute(destination);
    if (h == NULL) {
        return false;
    }
    struct packet pkt;
    pkt.sender = _id;
    pkt.receiver = destination;
    pkt.type = type;
    pkt.ttl = 255;
    pkt.datalen = len;
    if (len > 0) {
        memcpy(pkt.data, data, min(len, MTU));
    }
    calcCS(&pkt);
    h->device->unicastPacket(h->hwaddr, (uint8_t *)&pkt);
    return true;
}

boolean Mesh::knowHost(uint16_t id) {
    if (id == Direct || id == Broadcast) {
        return false;
    }
    for (struct host *h = _hostlist; h; h = h->next) {
        if (h->id == id) {
            return true;
        }
    }
    return false;
}


size_t Mesh::printTo(Print &p) const {
    size_t l = 0;
    l += p.print("My host ID: ");
    l += p.println(_id);
    l += p.println("Devices:");
    for (struct device *d = _devlist; d; d = d->next) {
        uint8_t buf[MTU];
        int hwlen = d->dev->getHardwareAddress(buf);
        l += p.print("    ");
        for (int i = 0; i < hwlen; i++) {
            if (i > 0) {
                l += p.print(":");
            }
            if (buf[i] < 16) {
                l += p.print("0");
            }
            l += p.print(buf[i], HEX);
        }
        l += p.println();
    }
    l += p.println("Directly connected hosts:");

    for (struct host *h = _hostlist; h; h = h->next) {
        if (h->nexthop == Direct) {
            l += p.print("    ");
            l += p.print(h->id);
            l += p.print(" on ");

            uint8_t buf[MTU];
            int hwlen = h->device->getHardwareAddress(buf);
            for (int i = 0; i < hwlen; i++) {
                if (i > 0) {
                    l += p.print(":");
                }
                if (h->hwaddr[i] < 16) {
                    l += p.print("0");
                }
                l += p.print(h->hwaddr[i], HEX);
            }
            l += p.print(" via ");

            hwlen = h->device->getHardwareAddress(buf);
            for (int i = 0; i < hwlen; i++) {
                if (i > 0) {
                    l += p.print(":");
                }
                if (buf[i] < 16) {
                    l += p.print("0");
                }
                l += p.print(buf[i], HEX);
            }
            l += p.println();
        }
    }

    l += p.println("Known remote hosts:");

    for (struct host *h = _hostlist; h; h = h->next) {
        if (h->nexthop != Direct) {
            l += p.print("    ");
            l += p.print(h->id);
            l += p.print(" via ");
            l += p.print(h->nexthop);
            l += p.print(" (cost ");
            l += p.print(h->cost);
            l += p.println(")");
        }
    }
    return l;
}

void Mesh::calcCS(struct packet *pkt) {
    uint8_t cs = 0;
    pkt->csum = 0;
    for (int i = 0; i < 32; i++) {
        cs += pkt->bytes[i];
    }
    pkt->csum = (256 - cs);
}

boolean Mesh::checkCS(struct packet *pkt) {
    uint8_t cs = 0;
    for (int i = 0; i < 32; i++) {
        cs += pkt->bytes[i];
    }
    return cs == 0;
}
