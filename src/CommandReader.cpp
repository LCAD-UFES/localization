#include "CommandReader.hpp"

CommandReader::CommandReader(std::string topic_name, unsigned int s) : topic(topic_name), queue_size(s), nh() {}