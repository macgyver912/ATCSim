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
#include <exception>

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

    if( f->getPosition().get_x() < 1000 && f->getSpeed() < LANDING_SPEED + 20)
        routeName += "GoAround";
    else if( f->getPosition().get_x() < 9000 )
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


float CheckBearing(float bearing){
    bearing = (180 - bearing);
    if(bearing < 0)
        bearing += 360;
    else if(bearing > 360)
        bearing -= 360;
    return bearing;
}


void checkTCAS(Flight *f, Flight *otherFlight)
{

    float deltaDist, deltaBearing, deltaAlt, dist2arptThis, dist2arptOther;
    float offsetXthis, offsetYthis, offsetXother, offsetYother;
    // Check that is not the flight itself
    if(f->getId() != otherFlight->getId())
    {
        // If this flight has not alert active, active it
        if(tcasAlerts.count(f->getId()) == 0)
        {
            dist2arptThis = f->getPosition().distance(Position(0, 0, 0));
            dist2arptOther = otherFlight->getPosition().distance(Position(0, 0, 0));
            deltaDist = f->getPosition().distance(otherFlight->getPosition());

            // If distance below minimum and aren't at final approach
            if(deltaDist < tcasAlertDist && dist2arptThis > 4500 && dist2arptOther > 4500)
            {
                std::cout << "TCAS: conflict between " << f->getId() << " and "
                    << otherFlight->getId() << " (" << deltaDist << ")" << std::endl;

                deltaBearing = CheckBearing(toDegrees(f->getBearing())) -
                    CheckBearing(toDegrees(otherFlight->getBearing()));

                // If bearing is 'South'
                if(CheckBearing(toDegrees(f->getBearing())) > 90
                    && CheckBearing(toDegrees(f->getBearing())) < 270 )
                {
                    offsetXthis = 1500.0;
                    // right turn to south-eastbound flights
                    offsetYthis = -1500.0;
                    // left turn to south-westbound flights
                    if(f->getPosition().get_y() < 0)
                        offsetYthis *= -1.0;
                }
                else
                {
                    offsetXthis = -1500.0;
                    // left turn to north-eastbound flights
                    offsetYthis = -1500.0;
                    // right turn to north-westbound flights
                    if(f->getPosition().get_y() < 0)
                        offsetYthis *= -1.0;
                }

                // If bearing is 'South'
                if(CheckBearing(toDegrees(otherFlight->getBearing())) > 90
                    && CheckBearing(toDegrees(otherFlight->getBearing())) < 270 )
                {
                    offsetXother = 1500.0;
                    // right turn to south-eastbound flights
                    offsetYother = -1500.0;
                    // left turn to south-westbound flights
                    if(otherFlight->getPosition().get_y() < 0)
                        offsetYother *= -1.0;
                }
                else
                {
                    offsetXother = -1500.0;
                    // left turn to north-eastbound flights
                    offsetYother = -1500.0;
                    // right turn to north-westbound flights
                    if(otherFlight->getPosition().get_y() < 0)
                        offsetYother *= -1.0;
                }

                if(abs(deltaBearing) < 25)
                {
                    // TODO: improve this logic because it depends of bearing
                    if(dist2arptThis > dist2arptOther){
                        // reduce speed
                        f->getRoute()->front().speed -= 20;
                    }else if(otherFlight->getRoute()->size() > 0){
                        // reduce speed
                        otherFlight->getRoute()->front().speed -= 20;
                    }

                }
                else if(abs(deltaBearing) < 60)
                {
                    // TODO: improve this logic because it depends of bearing
                    if(dist2arptThis > dist2arptOther){
                        // deviation to right, descend and reduce speed
                        Position auxPos(
                            f->getPosition().get_x() + offsetXthis,
                            f->getPosition().get_y() + offsetYthis,
                            f->getPosition().get_z() - 300
                        );
                        Route auxRoute;
                        auxRoute.pos = auxPos;
                        auxRoute.speed = f->getSpeed() - 30;
                        f->getRoute()->push_front(auxRoute);

                    }else if(otherFlight->getRoute()->size() > 0){
                        // deviation to right, descend and reduce speed
                        Position auxPos(
                            otherFlight->getPosition().get_x() + offsetXother,
                            otherFlight->getPosition().get_y() + offsetYother,
                            otherFlight->getPosition().get_z() - 300
                        );
                        Route auxRoute;
                        auxRoute.pos = auxPos;
                        auxRoute.speed = otherFlight->getSpeed() - 30;
                        otherFlight->getRoute()->push_front(auxRoute);
                    }

                }
                else
                {
                    // TODO: improve this logic because it depends of bearing
                    if(f->getPosition().get_z() > otherFlight->getPosition().get_z()){
                        // deviation to right
                        Position auxPos(
                            f->getPosition().get_x() + offsetXthis,
                            f->getPosition().get_y() + offsetYthis,
                            f->getPosition().get_z() - 300
                        );
                        Route auxRoute;
                        auxRoute.pos = auxPos;
                        auxRoute.speed = f->getSpeed() - 30;
                        f->getRoute()->push_front(auxRoute);

                    }else if(otherFlight->getRoute()->size() > 0){
                        // deviation to right
                        Position auxPos(
                            otherFlight->getPosition().get_x() + offsetXother,
                            otherFlight->getPosition().get_y() + offsetYother,
                            otherFlight->getPosition().get_z() - 300
                        );
                        Route auxRoute;
                        auxRoute.pos = auxPos;
                        auxRoute.speed = otherFlight->getSpeed() - 30;
                        otherFlight->getRoute()->push_front(auxRoute);
                    }
                }

                // Set this flights as alerted but not resolved
                tcasAlerts.insert({f->getId(), otherFlight->getId()});
                tcasAlerts.insert({otherFlight->getId(), f->getId()});
            }//if
        }
        else
        {
            // If TCAS alert has been triggered
            std::unordered_map<std::string, std::string>::const_iterator got = tcasAlerts.find (f->getId());
            if ( got != tcasAlerts.end() ){
                deltaDist = f->getPosition().distance(otherFlight->getPosition());
                // If these flights are the alerted flights and are free of conflict
                if( got->second == otherFlight->getId() && deltaDist > tcasAlertDist){
                    tcasAlerts.erase(f->getId());
                    tcasAlerts.erase(otherFlight->getId());
                }
            }//if got

        }//if-else tcasAlerts.count

    }//if f->getId()

}// checkTCAS



void
AirController::doWork()
{
    std::list<Flight*> flights = Airport::getInstance()->getFlights();
    std::list<Flight*>::iterator it, it2;

    for(it = flights.begin(); it!=flights.end(); ++it)
    {
        // This code could throw exception if flight crashes while accessing it
        try{
            // If this flight has not route
            if((*it)->getRoute()->empty())
            {
                setEmptyRoute((*it));
        	}
            // Check collision conflicts
            for(it2 = flights.begin(); it2!=flights.end(); ++it2){
                if(it2 != flights.end())
                    checkTCAS((*it), (*it2));
                else
                    std::cerr << "it2 is NULL" << '\n';
            }
        }catch(std::exception& e){
            std::cout << "Standard exception: " << e.what() << std::endl;
        }
    }
}








}  // namespace atcsim
