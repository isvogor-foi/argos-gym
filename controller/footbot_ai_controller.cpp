/* Include the controller definition */
#include "footbot_ai_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <string>
#include <argos3/core/utility/math/angles.h>
#include <QImage>
#include <string>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/datatypes/color.h>
#include <boost/algorithm/string.hpp>



/****************************************/
/****************************************/

CFootBotAIController::CFootBotAIController() :
   //m_in_socket(m_io_context, udp::endpoint(udp::v4(), m_port)),
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cSpace(argos::CSimulator::GetInstance().GetSpace()),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotAIController::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   m_pcPos = GetSensor  <CCI_PositioningSensor>("positioning");

   try{
     m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
   } catch(CARGoSException& ex) {}
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   std::string full_id = std::string(GetId().c_str(), 5);
   // if ID is the same, try changing the random number in .argos file
   //std::cout<<"L: "<<full_id<<std::endl;
   m_id = std::stoi(full_id.erase(0, 2));

   m_state = State();
   startSocket();
}

/****************************************/
/****************************************/

void CFootBotAIController::startSocket(){
  m_io_context = new boost::asio::io_context();

  m_send_port = m_send_port + m_id;
  m_rec_port = m_rec_port + m_id;

  std::cout<<"Port: " << m_send_port << ", " << m_rec_port << std::endl;

  m_in_socket = new udp::socket(*m_io_context, udp::endpoint(udp::v4(), m_send_port));
  m_out_socket = new udp::socket(*m_io_context);
  m_out_socket->open(boost::asio::ip::udp::v4());

  doReceive();
}
/****************************************/
/****************************************/

void CFootBotAIController::doSend(char data[max_length], std::size_t length){
  m_receiver_endpoint = boost::asio::ip::udp::endpoint(
    boost::asio::ip::address::from_string("127.0.0.1"),
    m_rec_port);

  m_out_socket->async_send_to(
    boost::asio::buffer(data, length), m_receiver_endpoint,
    [this](boost::system::error_code, std::size_t){});
}

/****************************************/
/****************************************/

void CFootBotAIController::doReceive(){
  m_in_socket->async_receive_from(
      boost::asio::buffer(m_data, max_length), m_sender_endpoint,
      [this](boost::system::error_code ec, std::size_t bytes_recvd)
      {
        if (!ec && bytes_recvd > 0)
        {
          std::vector<std::string> result;
          boost::split(result, m_data, boost::is_any_of(";"));
          SetWheelSpeedsFromVector(CVector2(std::stof(result[0]), std::stof(result[1])));
          //std::cout<<"Incoming: " << result[0] << std::endl;
          //m_pcWheels->SetLinearVelocity(std::stof(result[0]), std::stof(result[1]));

          doReceive();
        }
        else
        {
          doReceive();
        }
      });
}

/****************************************/
/****************************************/

void CFootBotAIController::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

   /* Turning state switching conditions */
   if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
      /* No Turn, heading angle very small */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
   }
   else if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
      /* Hard Turn, heading angle very large */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
   }
   else if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN &&
           Abs(cHeadingAngle) > m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
      /* Soft Turn, heading angle in between the two cases */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
   }

   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }

      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }

      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }

   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

CFootBotAIController::SWheelTurningParams::SWheelTurningParams() :
   TurningMechanism(NO_TURN),
   HardTurnOnAngleThreshold(ToRadians(CDegrees(90.0))),
   SoftTurnOnAngleThreshold(ToRadians(CDegrees(70.0))),
   NoTurnAngleThreshold(ToRadians(CDegrees(10.0))),
   MaxSpeed(10.0)
{
}

void CFootBotAIController::setInitialPosition(CVector3 init_pos)
{
  m_initial_position = init_pos;
}

/****************************************/
/****************************************/

float CFootBotAIController::getInitialVelocity()
{
  return m_fWheelVelocity;
}

/****************************************/
/****************************************/

std::string CFootBotAIController::getstrId()
{
  return m_strId;
}

/****************************************/
/****************************************/

void CFootBotAIController::setFbId(int FbId)
{
  m_fb_id = FbId;
}

/****************************************/
/****************************************/

void CFootBotAIController::setDataType(std::string dt)
{
  data_type = dt;
}

/****************************************/
/****************************************/

std::array<float, 48> CFootBotAIController::ConvertTReadings(CCI_FootBotProximitySensor::TReadings& proximities)
{
  std::array<float, 48> proxima;
  for(int i = 0; i<24; i++)
  {
    proxima[2*i] = proximities[i].Value;
    proxima[2*i+1] = proximities[i].Angle.GetValue();
  }
  return proxima;
}

std::string CFootBotAIController::State::GetPackage(){
  return std::to_string(m_global_x) + ";" + std::to_string(m_global_y);
}
/*************************************/
/*************************************/

void CFootBotAIController::ControlStep() {
  //std::cerr <<m_id<<", "<< "entering control step" << std::endl;

  const CCI_PositioningSensor::SReading& sPosRead = m_pcPos->GetReading();
  CColor clr = m_cSpace.GetFloorEntity().GetColorAtPoint(sPosRead.Position.GetX(), sPosRead.Position.GetY());
  m_state.Update(sPosRead.Position.GetX(), sPosRead.Position.GetY());

  m_io_context->poll();
  std::string msg = m_state.GetPackage() + ";" + std::to_string(clr.ToGrayScale());
  char pack[msg.size() + 1];
  strcpy(pack, msg.c_str());

  doSend(pack, sizeof(pack));

  // get the action to execute
  //float wheel_speed = m_env->getActions(m_fb_id);
  //std::cerr << "Action [" << m_fb_id << "]: " << wheel_speed << std::endl;
  // execute the action (throttle)
  //m_pcWheels->SetLinearVelocity(wheel_speed, wheel_speed);

  // set the new state for this footbot
  //if(data_type != "frame")
  //{
  //  CCI_FootBotProximitySensor::TReadings proximities = m_pcProximity->GetReadings();
  //  std::array<float, 48> proxim_readings = this->ConvertTReadings(proximities);
  //  m_distance += (wheel_speed/10);
    //CVector3 dist = m_footbot->GetEmbodiedEntity().GetOriginAnchor().Position - m_initial_position;
    //m_distance = dist.Length();
  //}
}

/****************************************/
/****************************************/

/*
* Setup socket stuff
*/


/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotAIController, "crossroad_footbot_controller")
