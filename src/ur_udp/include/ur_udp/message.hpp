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
    int button;

    MSGPACK_DEFINE(vel,pos, time,button) // Macro pour Msgpack
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
    double transmissionTime;

    MSGPACK_DEFINE(f, time, transmissionTime) // Macro pour Msgpack
};


#endif 