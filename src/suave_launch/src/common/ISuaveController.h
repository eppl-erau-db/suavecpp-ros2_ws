#ifndef ISUAVECONTROLLER_H
#define ISUAVECONTROLLER_H

#include <cstdint>  // MAVSDK 1.4 headers rely on it transitively
#include <mavsdk/mavsdk.h>

class ISuaveController
{
public:
    ISuaveController() = default;

    virtual ~ISuaveController() = default;

    virtual void start() = 0;
    virtual void shutdown() = 0;

protected:
    mavsdk::Mavsdk m_mavsdk{};
};

#endif //ISUAVECONTROLLER_H
