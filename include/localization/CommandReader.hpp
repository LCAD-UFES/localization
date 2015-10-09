#ifndef COMMAND_READER_H
#define COMMAND_READER_H


#include "geometry_msgs/TwistStamped.h"

class CommandReader {

    public:
        // the command
        virtual geometry_msgs::TwistStamped getCMD() =0;
        virtual void setCMD(geometry_msgs::TwistStamped) =0;
};

#endif