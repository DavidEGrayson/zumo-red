#pragma once

class RobotState
{
public:
    virtual void setup() { }
    virtual void loop() = 0;
};
