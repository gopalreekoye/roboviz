/*
 * @(#) Scenario.cpp   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#include <iostream>
#include "config/RobogenConfig.h"
#include "config/TerrainConfig.h"
#include "model/objects/BoxObstacle.h"
#include "model/objects/Resource.h"
#include "scenario/Scenario.h"
#include "scenario/Terrain.h"
#include "Robot.h"
#include "Environment.h"

namespace robogen {

Scenario::Scenario(boost::shared_ptr<RobogenConfig> robogenConfig) :
		robogenConfig_(robogenConfig), startPositionId_(0) {

}

Scenario::~Scenario() {

}

bool Scenario::init(dWorldID odeWorld, dSpaceID odeSpace,
		std::vector<boost::shared_ptr<Robot>> robots) {

	environment_ = boost::shared_ptr<Environment>(new
			Environment(odeWorld, odeSpace, robogenConfig_));

	if(!environment_->init()) {
		return false;
	}
	osg::Vec3 gatheringZonePosition = robogenConfig_->getGatheringZonePosition();
	osg::Vec3 gatheringZoneSize = robogenConfig_->getGatheringZoneSize();
	osg::Vec3 rotAxis = osg::Vec3(0,0,0);
	//gathering zone setup
	boost::shared_ptr<BoxObstacle> gatheringZone(
		new BoxObstacle(odeWorld, odeSpace,gatheringZonePosition,
						gatheringZoneSize, 1.f,rotAxis,
						0.f));
	environment_->setGatheringZone(gatheringZone);

	robots_ = robots;
	unsigned int swarmSize= robogenConfig_->getSwarmSize();
	// Setup robot position
	std::vector<double> minX(swarmSize, 0.0);
	std::vector<double> maxX(swarmSize, 0.0);
	std::vector<double> minY(swarmSize, 0.0);
	std::vector<double> maxY(swarmSize, 0.0);
	std::vector<double> minZ(swarmSize, 0.0);
	std::vector<double> maxZ(swarmSize, 0.0);
	
	std::vector<osg::Vec2> startingPosition(swarmSize);


	// Starting position and orientation
	for(int q=0;q<swarmSize;q++){
		startingPosition[q] = robogenConfig_->getStartingPos()->getStartPosition(q)->getPosition();
		float startingAzimuth = robogenConfig_->getStartingPos()->getStartPosition(q)->getAzimuth();
		osg::Quat roboRot;
		roboRot.makeRotate(osg::inDegrees(startingAzimuth), osg::Vec3(0,0,1));
		robots[q]->rotateRobot(roboRot);
		robots[q]->getBB(minX[q], maxX[q], minY[q], maxY[q], minZ[q], maxZ[q]);
		robots[q]->translateRobot(
				osg::Vec3(startingPosition[q].x(),
						startingPosition[q].y(),
						robogenConfig_->getTerrainConfig()->getHeight()
							+ inMm(2) - minZ[q]));
		robots[q]->getBB(minX[q], maxX[q], minY[q], maxY[q], minZ[q], maxZ[q]);

		std::cout
				<< "The robot is enclosed in the AABB(minX, maxX, minY, maxY, minZ, maxZ) ("
				<< minX[q] << ", " << maxX[q] << ", " << minY[q] << ", " << maxY[q] << ", "
				<< minZ[q] << ", " << maxZ[q] << ")" << std::endl;
		std::cout << "Obstacles in this range will not be generated" << std::endl << std::endl;
	}
	// Setup obstacles
	boost::shared_ptr<ObstaclesConfig> obstacles =
			robogenConfig_->getObstaclesConfig();

	// Instance the boxes above the maximum terrain height
	const std::vector<osg::Vec3>& c = obstacles->getCoordinates();
	const std::vector<osg::Vec3>& s = obstacles->getSizes();
	const std::vector<float>& d = obstacles->getDensities();
	const std::vector<osg::Vec3>& rotationAxis = obstacles->getRotationAxes();
	const std::vector<float>& rotationAngles = obstacles->getRotationAngles();

	obstaclesRemoved_ = false;

	double overlapMaxZ= *std::min_element(minZ.begin(), minZ.end());

	for (unsigned int i = 0; i < c.size(); ++i) {
		boost::shared_ptr<BoxObstacle> obstacle(
									new BoxObstacle(odeWorld, odeSpace, c[i],
											s[i], d[i], rotationAxis[i],
											rotationAngles[i]));
		double oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ;
		obstacle->getAABB(oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ);

		/*
		float oMinX = c[i].x() - s[i].x() / 2;
		float oMaxX = c[i].x() + s[i].x() / 2;
		float oMinY = c[i].y() - s[i].y() / 2;
		float oMaxY = c[i].y() + s[i].y() / 2;
		float oMinZ = c[i].z() - s[i].z() / 2;
		float oMaxZ = c[i].z() + s[i].z() / 2;
		 */

		// Do not insert the obstacle if it is in the robot range
		//check in each robot range
		//if counter is greater than 0, 
		
		unsigned int counter=0;

		for(int h=0;h<swarmSize;h++){
			if ((oMinX <= minX[h] && oMaxX >= maxX[h]) || (oMinX >= minX[h] && oMinX <= maxX[h])
					|| (oMaxX >= minX[h] && oMaxX <= maxX[h])) {
				counter++;
			}

			
			if ((oMinY <= minY[h] && oMaxY >= maxY[h]) || (oMinY >= minY[h] && oMinY <= maxY[h])
					|| (oMaxY >= minY[h] && oMaxY <= maxY[h])) {
				counter++;
			}

			
			if ((oMinZ <= minZ[h] && oMaxZ >= maxZ[h]) || (oMinZ >= minZ[h] && oMinZ <= maxZ[h])
					|| (oMaxZ >= minZ[h] && oMaxZ <= maxZ[h])) {
				counter++;
			}
		}
		// Do not insert obstacles in the robot range
		if (counter==0) {
			environment_->addObstacle(obstacle);
		} else {
			if (robogenConfig_->getObstacleOverlapPolicy() ==
					RobogenConfig::ELEVATE_ROBOT) {

				if (oMaxZ > overlapMaxZ)
					overlapMaxZ = oMaxZ;
				environment_->addObstacle(obstacle);

			} else {
				obstacle->remove();
				obstaclesRemoved_ = true;
			}
		}

	}

	if (robogenConfig_->getObstacleOverlapPolicy() ==
			RobogenConfig::ELEVATE_ROBOT) {
		for(int g=0;g<swarmSize;g++){
			robots[g]->translateRobot(
					osg::Vec3(startingPosition[g].x(), startingPosition[g].y(),
							overlapMaxZ + inMm(2) - minZ[g]));
		}
	}

	// Setup resources
	boost::shared_ptr<ResourcesConfig> resources =
			robogenConfig_->getResourcesConfig();

	// Instance the boxes above the maximum terrain height
	const std::vector<osg::Vec3>& cr = resources->getCoordinates();
	const std::vector<osg::Vec3>& sr = resources->getSizes();
	const std::vector<float>& dr = resources->getDensities();
	const std::vector<osg::Vec3>& rotationAxisr = resources->getRotationAxes();
	const std::vector<float>& rotationAnglesr = resources->getRotationAngles();


	for (unsigned int i = 0; i < cr.size(); ++i) {
		boost::shared_ptr<BoxObstacle> resource(
			new BoxObstacle(odeWorld, odeSpace, cr[i],
					sr[i], dr[i], rotationAxisr[i],
					rotationAnglesr[i]));
		environment_->addResource(resource);

	}

	//Setup light sources
	boost::shared_ptr<LightSourcesConfig> lightSourcesConfig =
			robogenConfig_->getLightSourcesConfig();

	// todo do we need to do overlap check with light sources??

	std::vector<boost::shared_ptr<LightSource> > lightSources;
	this->getEnvironment()->setLightSources(lightSources);

	std::vector<osg::Vec3> lightSourcesCoordinates =
			lightSourcesConfig->getCoordinates();
	std::vector<float> lightSourcesIntensities =
				lightSourcesConfig->getIntensities();
	for (unsigned int i = 0; i < lightSourcesCoordinates.size(); ++i) {
		lightSources.push_back(boost::shared_ptr<LightSource>(
					new LightSource(odeSpace, lightSourcesCoordinates[i],
							lightSourcesIntensities[i])));

	}
	environment_->setLightSources(lightSources);


	// optimize the physics!  replace all fixed joints with composite bodies
	for(int e=0;e<swarmSize;e++){
		robots[e]->optimizePhysics();
	}
	return true;
}

boost::shared_ptr<StartPosition> Scenario::getCurrentStartPosition() {
	return robogenConfig_->getStartingPos()->getStartPosition(
			startPositionId_);
}

void Scenario::prune(){
	environment_.reset();
	unsigned int s=robogenConfig_->getSwarmSize();
	for(int k=0;k<s;k++){
		robots_[k].reset();
	}
}

boost::shared_ptr<Robot> Scenario::getRobot(int id) {
	return robots_[id];
}
boost::shared_ptr<Robot> Scenario::getRobot() {
	return robots_[0];
}

boost::shared_ptr<RobogenConfig> Scenario::getRobogenConfig() {
	return robogenConfig_;
}

std::vector<boost::shared_ptr<Robot>> Scenario::getRobots(){
	return robots_;
}

void Scenario::setStartingPosition(int id) {
	startPositionId_ = id;
}

boost::shared_ptr<Environment> Scenario::getEnvironment() {
	return environment_;
}



}
