#include "Robot.hpp"

#include "Client.hpp"
#include "CommunicationService.hpp"
#include "Goal.hpp"
#include "Logger.hpp"
#include "MainApplication.hpp"
#include "MathUtils.hpp"
#include "Message.hpp"
#include "MessageTypes.hpp"
#include "RobotWorld.hpp"
#include "Server.hpp"
#include "Shape2DUtils.hpp"
#include "Wall.hpp"
#include "WayPoint.hpp"
#include "MainFrameWindow.hpp"
#include "serverConfig.hpp"

#include <chrono>
#include <ctime>
#include <sstream>
#include <thread>

namespace Model
{
	/**
	 *
	 */
	Robot::Robot() : Robot("", wxDefaultPosition)
	{
	}
	/**
	 *
	 */
	Robot::Robot( const std::string& aName) : Robot(aName, wxDefaultPosition)
	{
	}
	/**
	 *
	 */
	Robot::Robot(	const std::string& aName,
					const wxPoint& aPosition) :
								name( aName),
								size( wxDefaultSize),
								position( aPosition),
								front( 0, 0),
								speed( 0.0),
								acting(false),
								driving(false),
								communicating(false)
	{
		// We use the real position for starters, not an estimated position.
		startPosition = position;
	}
	/**
	 *
	 */
	Robot::~Robot()
	{
		if(driving)
		{
			Robot::stopDriving();
		}
		if(acting)
		{
			Robot::stopActing();
		}
		if(communicating)
		{
			stopCommunicating();
		}
	}
	/**
	 *
	 */
	void Robot::setName( const std::string& aName,
						 bool aNotifyObservers /*= true*/)
	{
		name = aName;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	wxSize Robot::getSize() const
	{
		return size;
	}
	/**
	 *
	 */
	void Robot::setSize(	const wxSize& aSize,
							bool aNotifyObservers /*= true*/)
	{
		size = aSize;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::setPosition(	const wxPoint& aPosition,
								bool aNotifyObservers /*= true*/)
	{
		position = aPosition;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	BoundedVector Robot::getFront() const
	{
		return front;
	}
	/**
	 *
	 */
	void Robot::setFront(	const BoundedVector& aVector,
							bool aNotifyObservers /*= true*/)
	{
		front = aVector;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	float Robot::getSpeed() const
	{
		return speed;
	}
	/**
	 *
	 */
	void Robot::setSpeed( float aNewSpeed,
						  bool aNotifyObservers /*= true*/)
	{
		speed = aNewSpeed;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::startActing(){
		this->acting = true;

		// Create a new thread that calls startDriving() in robotThread list
		robotThread = std::thread([this]{startDriving();});
}

	/**
	 *
	 */
	void Robot::stopActing()
	{
		acting = false;
		driving = false;
		robotThread.join();
	}
	/**
	 *
	 */
	void Robot::startDriving()
	{
		driving = true;

		goal = RobotWorld::getRobotWorld().getGoal( "Goal");
		calculateRoute(goal);

		drive();
	}
	/**
	 *
	 */
	void Robot::stopDriving()
	{
		driving = false;
	}
	/**
	 *
	 */
	void Robot::startCommunicating()
	{
		if(!communicating)
		{
			communicating = true;

			std::string localPort = "12345";
			if (Application::MainApplication::isArgGiven( "-local_port"))
			{
				localPort = Application::MainApplication::getArg( "-local_port").value;
			}

			if(Messaging::CommunicationService::getCommunicationService().isStopped())
			{
				TRACE_DEVELOP( "Restarting the Communication service");
				Messaging::CommunicationService::getCommunicationService().restart();
			}

			server = std::make_shared<Messaging::Server>(	static_cast<unsigned short>(std::stoi(localPort)),
															toPtr<Robot>());
			Messaging::CommunicationService::getCommunicationService().registerServer( server);
		}
	}
	/**
	 *
	 */
	void Robot::stopCommunicating()
	{
		if(communicating)
		{
			communicating = false;

			std::string localPort = "12345";
			if (Application::MainApplication::isArgGiven( "-local_port"))
			{
				localPort = Application::MainApplication::getArg( "-local_port").value;
			}

			Messaging::Client c1ient( 	"localhost",
										static_cast<unsigned short>(std::stoi(localPort)),
										toPtr<Robot>());
			Messaging::Message message( Messaging::StopCommunicatingRequest, "stop");
			c1ient.dispatchMessage( message);
		}
	}
	/**
	 *
	 */
	wxRegion Robot::getRegion() const
	{
		wxPoint translatedPoints[] = { getFrontRight(), getFrontLeft(), getBackLeft(), getBackRight() };
		return wxRegion( 4, translatedPoints); // @suppress("Avoid magic numbers")
	}
	/**
	 *
	 */
	bool Robot::intersects( const wxRegion& aRegion) const{
		wxRegion region = expandedRegion();
		region.Intersect( aRegion);
		return !region.IsEmpty();
	}
	/**
	 *
	 */
	wxPoint Robot::getFrontLeft() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		wxPoint originalFrontLeft( x, y);
		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		wxPoint frontLeft( static_cast<int>((originalFrontLeft.x - position.x) * std::cos( angle) - (originalFrontLeft.y - position.y) * std::sin( angle) + position.x),
						 static_cast<int>((originalFrontLeft.y - position.y) * std::cos( angle) + (originalFrontLeft.x - position.x) * std::sin( angle) + position.y));

		return frontLeft;
	}
	/**
	 *
	 */
	wxPoint Robot::getFrontRight() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		wxPoint originalFrontRight( x + size.x, y);
		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		wxPoint frontRight( static_cast<int>((originalFrontRight.x - position.x) * std::cos( angle) - (originalFrontRight.y - position.y) * std::sin( angle) + position.x),
						  static_cast<int>((originalFrontRight.y - position.y) * std::cos( angle) + (originalFrontRight.x - position.x) * std::sin( angle) + position.y));

		return frontRight;
	}
	/**
	 *
	 */
	wxPoint Robot::getBackLeft() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		wxPoint originalBackLeft( x, y + size.y);

		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		wxPoint backLeft( static_cast<int>((originalBackLeft.x - position.x) * std::cos( angle) - (originalBackLeft.y - position.y) * std::sin( angle) + position.x),
						static_cast<int>((originalBackLeft.y - position.y) * std::cos( angle) + (originalBackLeft.x - position.x) * std::sin( angle) + position.y));

		return backLeft;
	}
	/**
	 *
	 */
	wxPoint Robot::getBackRight() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		wxPoint originalBackRight( x + size.x, y + size.y);

		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		wxPoint backRight( static_cast<int>((originalBackRight.x - position.x) * std::cos( angle) - (originalBackRight.y - position.y) * std::sin( angle) + position.x),
						 static_cast<int>((originalBackRight.y - position.y) * std::cos( angle) + (originalBackRight.x - position.x) * std::sin( angle) + position.y));

		return backRight;
	}
	/**
	 *
	 */
	void Robot::handleNotification()
	{
		//	std::unique_lock<std::recursive_mutex> lock(robotMutex);

		static int update = 0;
		if ((++update % 200) == 0) // @suppress("Avoid magic numbers")
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::handleRequest( Messaging::Message& aMessage)
	{
		FUNCTRACE_TEXT_DEVELOP(aMessage.asString());

		switch(aMessage.getMessageType())
		{
			case Messaging::StopCommunicatingRequest:
			{
				aMessage.setMessageType(Messaging::StopCommunicatingResponse);
				aMessage.setBody("StopCommunicatingResponse");
				// Handle the request. In the limited context of this works. I am not sure
				// whether this works OK in a real application because the handling is time sensitive,
				// i.e. 2 async timers are involved:
				// see CommunicationService::stopServer and Server::stopHandlingRequests
				Messaging::CommunicationService::getCommunicationService().stopServer(12345,true); // @suppress("Avoid magic numbers")

				break;
			}
			case Messaging::EchoRequest:
			{
				aMessage.setMessageType(Messaging::EchoResponse);
				aMessage.setBody( "Messaging::EchoResponse: " + aMessage.asString());
				break;
			}
			case Messaging::SyncWorldRequest:{
				aMessage.setMessageType(Messaging::SyncWorldResponse);
				aMessage.setBody( "Messaging::SyncWorldResponse: " + aMessage.asString());
				break;
			}
			case Messaging::RobotPositionRequest:{
				aMessage.setMessageType(Messaging::RobotPositionResponse);
				std::ostringstream os;
				os << position.x << " " << position.y << " " << getFront().asString();
				aMessage.setBody(os.str());
				TRACE_DEVELOP("Oke here is robot location");

				//Request robot from other side
				// if(!WorldSynced){
				// 	TRACE_DEVELOP("Getting other Robot");
				// 	Application::MainFrameWindow::requestRobotLocation();
				// }
				break;
			}

			default:
			{
				TRACE_DEVELOP(__PRETTY_FUNCTION__ + std::string(": Default: Request not implemented!"));
				break;
			}
		}
	}
	/**
	 *
	 */
	void Robot::handleResponse( const Messaging::Message& aMessage)
	{
		FUNCTRACE_TEXT_DEVELOP(aMessage.asString());

		switch(aMessage.getMessageType())
		{
			case Messaging::StopCommunicatingResponse:
			{
				//Messaging::CommunicationService::getCommunicationService().stop();
				break;
			}
			case Messaging::EchoResponse:
			{
				break;
			}
			case Messaging::RobotPositionResponse:
			{
				updateRobotVector(aMessage.getBody());
				TRACE_DEVELOP("Robot position received: " + aMessage.getBody());
				break;
			}
			case Messaging::SyncWorldResponse:
			{
				TRACE_DEVELOP("Yeah boy! Syncing!");
				SyncWorld();
				break;
			}
			default:
			{
				TRACE_DEVELOP(__PRETTY_FUNCTION__ + std::string( ": default not implemented, ") + aMessage.asString());
				break;
			}
		}
	}
	/**
	 *
	 */
	std::string Robot::asString() const
	{
		std::ostringstream os;

		os << "Robot " << name << " at (" << position.x << "," << position.y << ")";

		return os.str();
	}
	/**
	 *
	 */
	std::string Robot::asDebugString() const
	{
		std::ostringstream os;

		os << "Robot:\n";
		os << "Robot " << name << " at (" << position.x << "," << position.y << ")\n";

		return os.str();
	}
	/**
	 *
	 */
	void Robot::SyncWorld(){
		if(!WorldSynced){
			//ToDo: Request Goal and walls
			TRACE_DEVELOP("Imma add the walls and goals OwO");
		}
		//Send robot position request
		Application::MainFrameWindow::requestRobotLocation();
	}


	void Robot::updateRobotVector(std::string messageBody){
		std::stringstream is(messageBody);
		unsigned short x, y, cx, cy;
		is >> x >> y >> cx >> cy;

		TRACE_DEVELOP("Robot position received: X " + std::to_string(x));
    	TRACE_DEVELOP("Robot position received: Y " + std::to_string(y));
	
		
		//Update Robot position
		if(!WorldSynced){
			Model::RobotWorld::getRobotWorld().newRobot("Bober", wxPoint(x, y));
			WorldSynced = true;
		}else{
			TRACE_DEVELOP("Ja stomme kneus dit is al true");
			auto robotToo =Model::RobotWorld::getRobotWorld().getRobot("Bober");
			robotToo->setPosition(wxPoint(x, y));
		}
	}

//-----------------------------------------------------
	void Robot::drive(){
		try{
			// The runtime value always wins!!
			speed = static_cast<float>(Application::MainApplication::getSettings().getSpeed());

			// Compare a float/double with another float/double: use epsilon...
			if (std::fabs(speed - 0.0) <= std::numeric_limits<float>::epsilon())
			{
				setSpeed(10.0, false); // @suppress("Avoid magic numbers")
			}

			if(WorldSynced){
				Application::MainFrameWindow::requestRobotLocation();
			}

			// We use the real position for starters, not an estimated position.
			startPosition = position;

			unsigned pathPoint = 0;
			while (position.x > 0 && position.x < 500 && position.y > 0 && position.y < 500 && pathPoint < path.size()) // @suppress("Avoid magic numbers")
			{
				// Do the update
				const PathAlgorithm::Vertex& vertex = path[pathPoint+=static_cast<unsigned int>(speed)];
				front = BoundedVector( vertex.asPoint(), position);
				position.x = vertex.x;
				position.y = vertex.y;

				// Stop on arrival or collision
				if (arrived(goal)) {
                Application::Logger::log(
                        __PRETTY_FUNCTION__ + std::string(": arrived"));
                driving = false;
            }
            if (wallCollision()) {
                Application::Logger::log(
                        __PRETTY_FUNCTION__ + std::string(": wall collision"));
                driving = false;
            }
            if (robotCollision()) {
                Application::Logger::log(
                        __PRETTY_FUNCTION__ + std::string(": robot collision"));
                evade();
            }
            if (tempPointActive) {
                if (arrived(tempPointPtr)) {
                    tempPointActive = false;
                    Application::Logger::log("tempPoint deleted");
                    if (tempPointPtr){
                    RobotWorld::getRobotWorld().deleteWayPoint(tempPointPtr);
                    }
                    Application::Logger::log(
                            __PRETTY_FUNCTION__
                                    + std::string(": evade point reached"));

                    startDriving();

                }
            }

				notifyObservers();

				// If there is no sleep_for here the robot will immediately be on its destination....
				std::this_thread::sleep_for( std::chrono::milliseconds( 100)); // @suppress("Avoid magic numbers")

				// this should be the last thing in the loop
				if(driving == false)
				{
					break;
				}
			} // while
		}
		catch (std::exception& e)
		{
			Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": ") + e.what());
			std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
		}
		catch (...)
		{
			Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": unknown exception"));
			std::cerr << __PRETTY_FUNCTION__ << ": unknown exception" << std::endl;
		}
	}
	/**
	 *
	 */
	void Robot::calculateRoute(WayPointPtr aGoal){
		path.clear();
		if (aGoal)
		{
			// Turn off logging if not debugging AStar
			Application::Logger::setDisable();

			front = BoundedVector( aGoal->getPosition(), position);
			//handleNotificationsFor( astar);
			path = astar.search( position, aGoal->getPosition(), size);
			//stopHandlingNotificationsFor( astar);

			Application::Logger::setDisable( false);
		}else{
			Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": no goal set"));
		}
	}
	/**
	 *
	 */
	bool Robot::arrived(GoalPtr aGoal) {
    if (aGoal && intersectsFinish(aGoal->getRegion())) {
        return true;
    }
    return false;
	}


	bool Robot::intersectsFinish(const wxRegion &aRegion) const {
    	wxRegion region = getRegion();
    	region.Intersect(aRegion);
    	return !region.IsEmpty();
	}

	/**
	 *
	 */
	bool Robot::wallCollision()
    {
        wxPoint frontLeft = getFrontLeft();
        wxPoint frontRight = getFrontRight();
        wxPoint backLeft = getBackLeft();
        wxPoint backRight = getBackRight();

        const std::vector< WallPtr >& walls = RobotWorld::getRobotWorld().getWalls();
        for (WallPtr wall : walls)
        {
            if (Utils::Shape2DUtils::intersect( frontLeft, frontRight, wall->getPoint1(), wall->getPoint2())     ||
                Utils::Shape2DUtils::intersect( frontLeft, backLeft, wall->getPoint1(), wall->getPoint2())        ||
                Utils::Shape2DUtils::intersect( frontRight, backRight, wall->getPoint1(), wall->getPoint2()))
            {
                Application::Logger::log("CollisionWithWall");
                return true;
            }
        }

        return false;
    }

    bool Robot::robotCollision() {
    const std::vector<RobotPtr> &robots = RobotWorld::getRobotWorld().getRobots();
    for (RobotPtr robot : robots) {
        if (getObjectId() == robot->getObjectId()) {
            continue;
        }
        if (intersects(robot->hitRegion())) {
            Application::Logger::log("CollisionWithRobot");
            return true;
        }
    }
    return false;
}

	void Robot::evade() {
    double angle = angleCollision();
    if (angle < 45 || angle > 315){
        Application::Logger::log("turning left");
		turnAround();

    }else if (angle > 225 && angle <= 315){
        Application::Logger::log("continuing");
        driving = true;
    }else if (angle > 135 && angle <= 225){
        Application::Logger::log("no collision");
    }else if (angle > 45 && angle <= 135){
        Application::Logger::log("waiting");
        while (robotCollision()){
            driving = false;
        }
        driving = true;
    }
}

double Robot::angleCollision() {
    double angle = 0;
    const std::vector<RobotPtr> &robots =
            RobotWorld::getRobotWorld().getRobots();
    for (RobotPtr robot : robots) {
        if (getObjectId() == robot->getObjectId()) {
            continue;
        }
        if (intersects(robot->getRegion())) {
            angle = Utils::Shape2DUtils::getAngle(this->position, robot->position);
            angle = Utils::MathUtils::toDegrees(angle);
            double currentAngle = Utils::Shape2DUtils::getAngle(front);
            currentAngle = Utils::MathUtils::toDegrees(currentAngle);
            angle = angle - currentAngle;
            if (angle < 0){
                angle = 360 + angle;
            }
            Application::Logger::log(std::to_string(angle));
        }
    }

    return angle;
}

void Robot::turnAround() {
    if (robotCollision()) {
        if (!tempPointActive) {
            wxPoint evadePoint((getFrontRight().x + getBackRight().x) / 2,
                    (getFrontRight().y + getBackRight().y) / 2);
            RobotWorld::getRobotWorld().newWayPoint("Point", evadePoint);
            tempPointPtr = RobotWorld::getRobotWorld().getWayPoint("Point");
            Application::Logger::log("temppoint created");
            tempPointActive=true;
            //Application::Logger::log(    Utils::Shape2DUtils::asString(evadePoint) + " " + Utils::Shape2DUtils::asString(position));
        }
        Application::Logger::log("driving to evade");
        restartDriving();
    } else {
        Application::Logger::log("driving to goal");
        startDriving();
    }
}

wxRegion Robot::expandedRegion() const {
    int leftX = 0;
    int leftY = 0;
    int rightX = 0;
    int rightY = 0;
    if (getFrontRight().x >= getBackRight().x) {
        rightX = getFrontRight().x + (getFrontRight().x - getBackRight().x);
    } else {
        rightX = getFrontRight().x + (getBackRight().x - getFrontRight().x);
    }
    if (getFrontRight().y >= getBackRight().y) {
        rightY = getFrontRight().y + (getFrontRight().y - getBackRight().y);
    } else {
        rightY = getFrontRight().y + (getBackRight().y - getFrontRight().y);
    }

    if (getFrontLeft().x >= getBackLeft().x) {
        leftX = getFrontLeft().x + (getFrontLeft().x - getBackLeft().x);
    } else {
        leftX = getFrontLeft().x + (getBackLeft().x - getFrontLeft().x);
    }
    if (getFrontLeft().y >= getBackLeft().y) {
        leftY = getFrontLeft().y + (getFrontLeft().y - getBackLeft().y);
    } else {
        leftY = getFrontLeft().y + (getBackLeft().y - getFrontLeft().y);
    }

    wxPoint newFrontLeft(leftX, leftY);
    wxPoint newFrontRight(rightX, rightY);

    wxPoint translatedPoints[] = { newFrontRight, newFrontLeft, getBackLeft(),
            getBackRight() };

    Application::Logger::log(Utils::Shape2DUtils::asString(newFrontLeft));
    return wxRegion(4, translatedPoints);
}

bool Robot::arrived(WayPointPtr tempGoal) {
    if (tempGoal && intersects(tempGoal->getRegion())) {
        return true;
    }
    return false;
}

void Robot::restartDriving() {
    driving = true;
    calculateRoute(tempPointPtr);
    drive();
}

wxRegion Robot::hitRegion() const {
    // x and y are pointing to top left now
    int x = position.x - (size.x);
    int y = position.y - (size.y);

    wxPoint originalUpperLeft(x, y);
    wxPoint originalUpperRight(x + size.x * 2, y);
    wxPoint originalBottomLeft(x, y + size.y * 2);
    wxPoint originalBottomRight(x + size.x * 2, y + size.y * 2);

    wxPoint originalPoints[] = { originalUpperRight, originalUpperLeft,
            originalBottomLeft, originalBottomRight };

    return wxRegion(4, originalPoints);
}

} // namespace Model
