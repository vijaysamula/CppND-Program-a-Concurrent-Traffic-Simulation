#include <iostream>
#include <random>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */

 
template <typename T>
T MessageQueue<T>::receive()
{
    std::unique_lock<std::mutex> uLock(_mutex);
    _cond.wait(uLock,[this] {return !_queue.empty();});

    T msg = std::move(_queue.front());
    _queue.pop_front();

    return msg;

}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    
    std::lock_guard<std::mutex> lGuard(_mutex);
    _queue.clear();
    _queue.emplace_back(std::move(msg));
    _cond.notify_one();

}


/* Implementation of class "TrafficLight" */


TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

TrafficLight::~TrafficLight()
{
   
}

void TrafficLight::waitForGreen()
{
    while(true) {
        if(_messageQueue.receive() == TrafficLightPhase::green) break;
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases,this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    long timeSinceLastUpdate= 0;
    std::random_device rndm;
    static std::mt19937 mt(rndm());
    std::uniform_real_distribution<double> uDist(4000.0, 6000.0);
    auto cycleDuration = std::chrono::duration<double>(uDist(mt));
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;

    lastUpdate = std::chrono::system_clock::now();
    while(true){
        // sleep at every iteration to reduce CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // compute time difference to stop watch
        timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
        if (timeSinceLastUpdate >= cycleDuration.count())
        {   
            
            
            if (_currentPhase == TrafficLightPhase::red)
                _currentPhase = TrafficLightPhase::green;
                
            else 
                _currentPhase = TrafficLightPhase::red;
            
            _messageQueue.send(std::move(_currentPhase));
           
            lastUpdate = std::chrono::system_clock::now();
            cycleDuration = std::chrono::duration<double>(uDist(mt));
        }
    }
}

