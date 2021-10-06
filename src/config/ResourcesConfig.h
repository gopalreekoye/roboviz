#ifndef ROBOGEN_RESOURCES_CONFIG_H_
#define ROBOGEN_RESOURCES_CONFIG_H_

#include <osg/Vec3>
#include <vector>
#include "robogen.pb.h"

namespace robogen {

/**
 * Resources configuration parameters
 */
class ResourcesConfig: ObstaclesConfig {

public:

	/**
	 * Initializes obstacles configuration
	 */
	ResourcesConfig() {}

	ResourcesConfig(const std::vector<osg::Vec3>& coordinates,
			const std::vector<osg::Vec3>& sizes,
			const std::vector<float> &densities,
			const std::vector<osg::Vec3>& rotationAxes,
			const std::vector<float> &rotationAngles, unsigned int numberOfRobots) :
			coordinates_(coordinates), sizes_(sizes), densities_(densities),
			rotationAxes_(rotationAxes), rotationAngles_(rotationAngles), numberOfRobots_(numberOfRobots) {

	}

	/**
	 * Destructor
	 */
	virtual ~ResourcesConfig() {

	}

	/**
	 * @return the coordinates of the obstacles
	 */
	const std::vector<osg::Vec3>& getCoordinates() const {
		return coordinates_;
	}

	/**
	 * @return the size of the obstacles
	 */
	const std::vector<osg::Vec3>& getSizes() const {
		return sizes_;
	}

	/**
	 * @return the obstacle densities
	 */
	const std::vector<float>& getDensities() const{
		return densities_;
	}

	/**
	 * @return the obstacle densities
	 */
	const std::vector<osg::Vec3>& getRotationAxes() const{
		return rotationAxes_;
	}

	/**
	 * @return the obstacle densities
	 */
	const std::vector<float>& getRotationAngles() const{
		return rotationAngles_;
	}

	/**
	 * @return number of robots required to push resource
	 */
	const unsigned int getNumberOfRobots() const{
		return numberOfRobots_;
	}

	/**
	 * Serialize obstacles into a SimulatorConf message
	 */
	void serialize(robogenMessage::SimulatorConf &message){
		for (unsigned int i=0; i<coordinates_.size(); ++i){
			robogenMessage::Obstacle *curr = message.add_obstacles();
			curr->set_density(densities_[i]);
			curr->set_x(coordinates_[i].x());
			curr->set_y(coordinates_[i].y());
			curr->set_z(coordinates_[i].z());
			curr->set_xsize(sizes_[i].x());
			curr->set_ysize(sizes_[i].y());
			curr->set_zsize(sizes_[i].z());
			curr->set_xrotation(rotationAxes_[i].x());
			curr->set_yrotation(rotationAxes_[i].y());
			curr->set_zrotation(rotationAxes_[i].z());
			curr->set_rotationangle(rotationAngles_[i]);
		}
	}

private:

	/**
	 * Obstacles coordinates
	 */
	std::vector<osg::Vec3> coordinates_;

	/**
	 * Obstacle sizes
	 */
	std::vector<osg::Vec3> sizes_;

	/**
	 * Obstacle densities. If 0, obstacle is fixed
	 */
	std::vector<float> densities_;

	/**
	 * Obstacle rotationAxes
	 */
	std::vector<osg::Vec3> rotationAxes_;

	/**
	 * Obstacle rotationAngles
	 */
	std::vector<float> rotationAngles_;

	/**
	 * number of robots required to push resource
	 * 
	 */
	unsigned int numberOfRobots_;
};

}

#endif /* ROBOGEN_OBSTACLES_CONFIG_H_ */
