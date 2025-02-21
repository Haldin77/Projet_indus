#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <msgpack.hpp>

struct MessagePhantom {
    struct Velocity {
        double vx;
        double vy;
        double vz;
        double wx;
        double wy;
        double wz;

        MSGPACK_DEFINE(vx, vy, vz, wx, wy, wz)
    };

    Velocity vel;
    double time;

    MSGPACK_DEFINE(vel, time) // Macro pour Msgpack
};

struct MessageUR {
    struct Force {
        double x;
        double y;
        double z;

        MSGPACK_DEFINE(x, y, z)
    };

    Force f;
    double time;

    MSGPACK_DEFINE(f, time) // Macro pour Msgpack
};


#endif 