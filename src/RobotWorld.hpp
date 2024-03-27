#ifndef ROBOTWORLD_HPP_
#define ROBOTWORLD_HPP_

#include "Config.hpp"

#include "ModelObject.hpp"
#include "Widgets.hpp"

#include <vector>

namespace Model
{
	class Robot;
	typedef std::shared_ptr<Robot> RobotPtr;

	class WayPoint;
	typedef std::shared_ptr<WayPoint> WayPointPtr;

	class Goal;
	typedef std::shared_ptr<Goal> GoalPtr;

	class Wall;
	typedef std::shared_ptr<Wall> WallPtr;

	class RobotWorld;
	typedef std::shared_ptr<RobotWorld> RobotWorldPtr;

	/**
	 *
	 */
	class RobotWorld : 	public ModelObject
	{
		public:
			static RobotWorld& getRobotWorld();
			RobotPtr newRobot(	const std::string& aName = "New Robot",
								const wxPoint& aPosition = wxPoint( -1, -1),
								bool aNotifyObservers = true);
			WayPointPtr newWayPoint(const std::string& aName = "New WayPoint",
									const wxPoint& aPosition = wxPoint( -1, -1),
									bool aNotifyObservers = true);
			GoalPtr newGoal(const std::string& aName = "New Goal",
							const wxPoint& aPosition = wxPoint( -1, -1),
							bool aNotifyObservers = true
							);
			WallPtr newWall(const wxPoint& aPoint1,
							const wxPoint& aPoint2,
							bool aNotifyObservers = true);
			void deleteRobot( 	RobotPtr aRobot,
								bool aNotifyObservers = true);
			void deleteWayPoint( 	WayPointPtr aWayPoint,
									bool aNotifyObservers = true);
			void deleteGoal( 	GoalPtr aGoal,
								bool aNotifyObservers = true);
			void deleteWall( 	WallPtr aWall,
								bool aNotifyObservers = true);
			RobotPtr getRobot( const std::string& aName) const;
			RobotPtr getRobot( const Base::ObjectId& anObjectId) const;
			WayPointPtr getWayPoint( const std::string& aName) const;
			WayPointPtr getWayPoint( const Base::ObjectId& anObjectId) const;
			GoalPtr getGoal( const std::string& aName) const;
			GoalPtr getGoal( const Base::ObjectId& anObjectId) const;
			WallPtr getWall( const Base::ObjectId& anObjectId) const;
			const std::vector< RobotPtr >& getRobots() const;
			const std::vector< WayPointPtr >& getWayPoints() const;
			const std::vector< GoalPtr >& getGoals() const;
			const std::vector< WallPtr >& getWalls() const;
			void populate(const int &worldNumber);
			void unpopulate( bool aNotifyObservers = true);
			std::string asCode() const;

			//Worlds: 
			void createDefaultWorld(bool notifyObserver = true);
			void createStudentWorld1(bool notifyObserver = true); //Situation 1.0
			void createStudentWorld2(bool notifyObserver = true); //Situation 1.1

            void createStudentWorld3(bool notifyObserver = true); //Situation 2.0

            void createStudentWorld4(bool notifyObserver = true); //Situation 2.1

			void createStudentWorld5(bool notifyObserver = true); //Situation 3.0

			void createStudentWorld6(bool notifyObserver = true); //Situation 3.1


            void createWorldBorder(bool notifyObserver = true);

            /**
			 *
			 * @param aKeepObjects Keep the objects with these ObjectIdsin the world
			 * @param aNotifyObservers
			 */
			void unpopulate( const std::vector<Base::ObjectId >& aKeepObjects,
							 bool aNotifyObservers = true);
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override;
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override;
			//@}

			void addRobot(RobotPtr aRobot);

		protected:
			RobotWorld() = default;
			virtual ~RobotWorld();

		private:
			/**
			 * The vectors are mutable to allow for lazy instantiation
			 */
			mutable std::vector< RobotPtr > robots;
			mutable std::vector< WayPointPtr > wayPoints;
			mutable std::vector< GoalPtr > goals;
			mutable std::vector< WallPtr > walls;
	};
} // namespace Model
#endif // ROBOTWORLD_HPP_
