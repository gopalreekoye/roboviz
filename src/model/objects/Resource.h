#ifndef ROBOGEN_RESOURCE_H_
#define ROBOGEN_RESOURCE_H_

#include "model/PositionObservable.h"

namespace robogen {

class Resource : public PositionObservable {
	public:

	/**
	 * Initializes a box obstacle
	 */
	Resource(dWorldID odeWorld, dSpaceID odeSpace, const osg::Vec3& pos,
			const osg::Vec3& size, float density,
			const osg::Vec3& rotationAxis, float rotationAngle);

	/**
	 * Remove from world
	 */
	virtual void remove();

	/**
	 * Destructor
	 */
	virtual ~Resource();

	/**
	 * Inherited from PositionObservable
	 */
	virtual const osg::Vec3 getPosition();
	virtual const osg::Quat getAttitude();

	/**
	 * @return the box size
	 */
	const osg::Vec3 getSize();

	/**
	 * @return the box size
	 */
	void getAABB(double& minX, double& maxX, double& minY,
			double& maxY, double& minZ, double& maxZ);

    private:

        /**
         * The box
         */
        dBodyID box_;

        /**
         * The box
         */
        dGeomID boxGeom_;

        /**
         * The box size
         */
        osg::Vec3 size_;

};


}

#endif /* ROBOGEN_RESOURCE_H_ */
