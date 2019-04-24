/**
 * @file parser.hpp
 * @author David Bang, Haowen Shi
 * @date 15 Apr 2019
 * @brief Mission file parser interface
 */

#ifndef __PARSER_HPP__
#define __PARSER_HPP__

#include "jr_common.h"

// TODO: more task related stuff here
// TODO: add hard-coded station map coordinate dict

/**
 * @brief Task is the class for each 
 */
class Task
{
private:
    int task_id;

public:
    /** @brief Class static counter for counter ID */
    static int task_counter;

    enum StationID station;

    Task(enum StationID station);

    /**
     * @brief Returns ID of the task
     */
    int getTaskID();
};

#endif /* __PARSER_HPP__ */