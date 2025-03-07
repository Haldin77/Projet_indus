#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <msgpack.hpp>

struct MessagePhantom {
    struct Velocity {
        double vx;
        double vy;
        double vz;
        MSGPACK_DEFINE(vx, vy, vz)
    };
    struct Position {
        double x;
        double y;
        double z;
        double w;
        MSGPACK_DEFINE(x, y, z, w)
    };

    Velocity vel;
    Position pos;
    double time;

    MSGPACK_DEFINE(vel,pos, time) // Macro pour Msgpack
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