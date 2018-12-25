/*
 * AirController.cpp
 *
 *  Created on: 21/07/2014
 *      Author: paco
 *
 *  Copyright 2014 Francisco Martín
 *
 *  This file is part of ATCSim.
 *
 *  ATCSim is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ATCSim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with ATCSim.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AirController.h"
#include "Airport.h"
#include "Flight.h"
#include "Position.h"
#include "CircuitRoute.h"
#include <list>
#include <fstream>

namespace atcsim{

float tcasAlertDist = 2.0*COLLISION_DISTANCE;

AirController::AirController() {
	// TODO Auto-generated constructor stub
    loadNavPoints();
	loadNavCircuits();
}

AirController::~AirController() {
	// TODO Auto-generated destructor stub
}


void setFinalApproach(Flight *f){
    // Final approach
    Position pos0(3500.0, 0.0, 200.0);
    Position pos1(1500.0, 0.0, 50.0);
    Position pos2(200.0, 0.0, 25.0);
    Position pos3(-750.0, 0.0, 25.0);

    Route r0, r1, r2, r3;

    r0.pos = pos0;
    r0.speed = 80.0;
    r1.pos = pos1;
    r1.speed = 75.0;
    r2.pos = pos2;
    r2.speed = 70.0;
    r3.pos = pos3;
    r3.speed = 50.0;
    //-----------------

    f->getRoute()->push_back(r0);
    f->getRoute()->push_back(r1);
    f->getRoute()->push_back(r2);
    f->getRoute()->push_back(r3);
}

void setRoute(Flight *f, std::string routeId){
    std::vector<Position> route = getRouteCircuit(routeId);

    std::vector<Position>::iterator it;
    ushort legCount = 0;
    for(it = route.begin(); it != route.end(); it++){
        Route r;
        r.pos = (*it);

        if( (*it).get_name() == "FINAL" )
            r.speed = 85;
        else if( (*it).get_name().find("1") != std::string::npos )
            r.speed = 100;
        else if( (*it).get_name().find("2") != std::string::npos )
            r.speed = 120;
        else if( (*it).get_name().find("3") != std::string::npos )
            r.speed = 120;
        else if( (*it).get_name().find("4") != std::string::npos )
            r.speed = 140;
        else if( (*it).get_name().find("5") != std::string::npos )
            r.speed = 150;
        else
            r.speed = 265;

        //r.speed = 200 - legCount*40;

        f->getRoute()->push_back(r);
        legCount++;
    }
}


void setEmptyRoute(Flight *f){
    std::string routeName;

    if( f->getPosition().get_x() < 9000 )
        routeName += "North";
    else if( f->getPosition().get_x() > 12000 )
        routeName += "South";

    if( f->getPosition().get_y() < 0 )
        routeName += "West";
    else
        routeName += "East";

    setRoute(f, routeName);

    setFinalApproach(f);
}



void checkTCAS(Flight *f)
{
    std::list<Flight*> flights = Airport::getInstance()->getFlights();
    std::list<Flight*>::iterator otherFlight;

    float deltaDist, deltaBearing, deltaAlt;
    for(otherFlight = flights.begin(); otherFlight!=flights.end(); ++otherFlight)
    {
        // Check that is not the flight itself
        if(f->getId() != (*otherFlight)->getId())
        {

            // If this flight has not alert active, active it
            if(tcasAlerts.count(f->getId()) == 0)
            {
                deltaDist = f->getPosition().distance((*otherFlight)->getPosition());
                if(deltaDist < tcasAlertDist)
                {
                    std::cout << "TCAS: conflict between " << f->getId() << " and "
                        << (*otherFlight)->getId() << " (" << deltaDist << ")" << std::endl;

                    deltaBearing = f->getBearing() - (*otherFlight)->getBearing();

                    if(abs(deltaBearing) < 25){         // reduce speed

                    }else if(abs(deltaBearing) < 60){   // reduce speed and deviation

                    }else{      // deviation both flights and change altitude

                    }



                    // Set this flights as alerted but not resolved
                    tcasAlerts.insert({f->getId(), 0});
                    tcasAlerts.insert({(*otherFlight)->getId(), 0});
                }//if
            }
            else
            {

            }//if

        }//if
    }// for
}// checkTCAS



void
AirController::doWork()
{
    std::list<Flight*> flights = Airport::getInstance()->getFlights();
    std::list<Flight*>::iterator it;



    // Test
    Route test0, test1, test2;
    test0.pos = getRoutePoint("MORAL");
    test0.speed = 200;
    test1.pos = getRoutePoint("TOBEK");
    test1.speed = 150;
    test2.pos = getRoutePoint("ASBIN");
    test2.speed = 120;
    //-----------------

    for(it = flights.begin(); it!=flights.end(); ++it)
    {

        // If this flight has not route
        if((*it)->getRoute()->empty())
        {
            setEmptyRoute((*it));
    	}

        // Check collision conflicts
        checkTCAS((*it));
    }
}








}  // namespace atcsim
