#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <msgpack.hpp>

struct MessageHaply {
    struct Velocity {
        float vx;
        float vy;
        float vz;
        MSGPACK_DEFINE(vx, vy, vz)
    };
    struct Position {
        float x;
        float y;
        float z;
        float w;
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