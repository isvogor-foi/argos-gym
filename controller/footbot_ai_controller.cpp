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

/****************************************/
/****************************************/

CFootBotAIController::CFootBotAIController() :
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
}

/****************************************/
/****************************************/

void CFootBotAIController::startSocket(){

}

/****************************************/
/****************************************/

void CFootBotAIController::doSend(std::size_t length){

}

/****************************************/
/****************************************/

void CFootBotAIController::doReceive(){

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
  std::cerr << "entering control step" << std::endl;

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
