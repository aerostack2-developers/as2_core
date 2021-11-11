#include "platform_state_machine.hpp"
#include <iostream>

int main (int argc, char *argv[])
{
    std::cout << "Platform FSM test " << std::endl;
    as2::PlatformStateMachine state_machine;
    state_machine.processEvent(as2::PlatformStateMachine::Event::ARM);
    state_machine.processEvent(as2::PlatformStateMachine::Event::ARM);
    state_machine.processEvent(as2::PlatformStateMachine::Event::TAKE_OFF);
    state_machine.processEvent(as2::PlatformStateMachine::Event::LAND);
    state_machine.processEvent(as2::PlatformStateMachine::Event::EMERGENCY);
    state_machine.processEvent(as2::PlatformStateMachine::Event::TOOK_OFF);
    
    // state_machine.start();
    // state_machine.stop();
    return 0;
}