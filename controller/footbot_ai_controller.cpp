/* Include the controller definition */
#include "footbot_ai_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <string>
#include <argos3/core/utility/math/angles.h>
#include <QImage>
#include <string>


/****************************************/
/****************************************/

CFootBotAIController::CFootBotAIController() :
   //m_in_socket(m_io_context, udp::endpoint(udp::v4(), m_port)),
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
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
          std::cout<<"Incoming: " << m_data << std::endl;
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

/*************************************/
/*************************************/

void CFootBotAIController::ControlStep() {
  std::cerr <<m_id<<", "<< "entering control step" << std::endl;

  m_io_context->poll();
  
  char message[] = "From c++";
  doSend(message, sizeof(message));

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
