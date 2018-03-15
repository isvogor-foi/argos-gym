/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef FOOTBOT_AI_CONTROLLER_H
#define FOOTBOT_AI_CONTROLLER_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/space/space.h>


#include <QElapsedTimer>
#include <array>

#include <boost/bind.hpp>
#include <boost/asio.hpp>

#ifdef __APPLE__
#include <glu.h>
#else
#include <GL/glu.h>
#endif
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;
using boost::asio::ip::udp;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotAIController : public CCI_Controller {

public:
  class State{
    float m_global_x;
    float m_global_y;
    float m_v_left;
    float m_v_right;
    float m_bottom_color;
    // add neighbors, id's and distances

  public:
    State(){};

    inline void Update(float global_x, float global_y){
      m_global_x = global_x;
      m_global_y = global_y;
    }

    std::string GetPackage();

  };

  public:

   /* Class constructor. */
   CFootBotAIController();

   /* Class destructor. */
   virtual ~CFootBotAIController() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /**
    * Return initial velocity
    */
   float getInitialVelocity();

   /**
    * Get the str id
    */
   std::string getstrId();

   /**
    * Set the int id
    */
   void setFbId(int FbId);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   void setDataType(std::string dt);

   State m_state;
   CSpace& m_cSpace;


private:

/* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;

   CCI_PositioningSensor* m_pcPos;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

   /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   Real m_fDelta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;

   /**
    * Distance travelled by the footbot.
    */
   float m_distance;

   /**
    * Convert TReadings to a multi-D array
    * to pass to the environment.
    */
   std::array<float, 48> ConvertTReadings(CCI_FootBotProximitySensor::TReadings& proximities);

   /**
    * index of the associated footbot (!= id of the associated footbot)
    */
   int m_fb_id;

   /**
    * Type of data exchanged (frame, numerical data)
    */
   std::string data_type;

   /**
    * Pointer to the associated footbot
    */
   CFootBotEntity* m_footbot;

   /**
    * Initial position
    */
   CVector3 m_initial_position;

   /**
    * Set the initial position (only the info)
    */
   void setInitialPosition(CVector3 init_pos);


   /**
   * Setup sockets
   */

private:
   udp::socket* m_in_socket;
   udp::socket* m_out_socket;

   udp::endpoint m_sender_endpoint;
   udp::endpoint m_receiver_endpoint;

   boost::asio::io_context* m_io_context;

   short m_send_port = 5030;
   short m_rec_port = 6030;

   enum { max_length = 1024 };
   char m_data[max_length];
   int m_id;

public:
   void startSocket();
   void doSend(char m_data[max_length], std::size_t length);
   void doReceive();
};

#endif
