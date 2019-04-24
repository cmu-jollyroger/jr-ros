/**
 * @file parser.cpp
 * @author David Bang, Haowen Shi
 * @date 15 Apr 2019
 * @brief Mission file parser
 */

#include "parser.hpp"

int Task::task_counter = 0;

Task::Task(enum StationID station) {
    task_id = task_counter++;
}

int Task::getTaskID() {
    return task_id;
}