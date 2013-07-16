/*
 * jsonTest.cpp
 *
 *  Created on: Jul 6, 2013
 *      Author: root
 */
#include <cstdlib>
#include <iostream>
#include <string>

#include <json/json.h>
#include <json/value.h>

using namespace std;


int main(){


	// fill planes array
	Json::Value planes(Json::arrayValue);
	Json::Value plane(Json::objectValue);
	for (int i = 0; i < 3; i++){
		plane["a"] = i;
		plane["b"] = i;
		plane["c"] = i;
		plane["d"] = i;
		planes.append(plane);
	}

	// fill obstacle array
	Json::Value obstacles(Json::arrayValue);
	for (int i = 0; i < 3; i++){
		Json::Value obstacle(Json::objectValue);
		obstacle["x"] = i;
		obstacle["y"] = i;
		obstacle["z"] = i;
		obstacle["radius"] = i;
		obstacles.append(obstacle);
	}


	Json::Value root(Json::objectValue);

	root["timestamp"] = 10; //arbitrary for now
	root["planes"] = planes;
	root["ground"] = plane;
	root["foundObstacles"] = true;
	root["obstacles"] = obstacles;

	Json::StyledWriter writer;

	//while(true){
    cout << writer.write(root);
	//}
}

