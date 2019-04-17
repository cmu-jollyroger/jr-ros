/**
 * @file parser.hpp
 * @author David Bang, Haowen Shi
 * @date 15 Apr 2019
 * @brief Mission file parser interface
 */

enum StationID {
    STATION_A,
    STATION_B,
    STATION_C,
    STATION_D,
    STATION_E,
    STATION_F,
    STATION_G,
    STATION_H
};

// TODO: more task related stuff here
// TODO: add hard-coded station map coordinate dict

class Task
{
private:
    int task_id;

public:
    static int task_counter;

    

    enum StationID station;

    Task(enum StationID station);

    /**
     * @brief Returns ID of the task
     */
    int getTaskID();
}