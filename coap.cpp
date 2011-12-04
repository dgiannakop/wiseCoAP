/*
 * Simple Wiselib Example
 */
#include "external_interface/external_interface.h"
#include "algorithms/routing/tree/tree_routing.h"
// SENSORS
#include "util/delegates/delegate.hpp"
#include "util/pstl/map_static_vector.h"
#include "util/pstl/static_string.h"
//#include "util/wisebed_node_api/sensors/sensor_controller.h"
//#include "util/wisebed_node_api/sensors/managed_sensor.h"
// END OF SENSORS
#include "algorithms/coap/coap.h"

#ifdef ISENSE
//#include "external_interface/isense/isense_light_sensor.h"
//#include "external_interface/isense/isense_temp_sensor.h"
#include "isense/modules/environment_module/environment_module.h"
#include "isense/modules/environment_module/temp_sensor.h"
#include "isense/modules/environment_module/light_sensor.h"
#endif

typedef wiselib::OSMODEL Os;
typedef Os::TxRadio Radio;
typedef Radio::block_data_t block_data_t;
typedef wiselib::Coap<Os, Radio, Os::Timer, Os::Debug, Os::Rand> coap_t;

// --------------------------------------------------------------------------
#ifdef ISENSE
typedef isense::EnvironmentModule environment_module_t;
typedef isense::TempSensor temp_sensor_t;
#endif
// --------------------------------------------------------------------------
class CoapApplication:
#ifdef ISENSE
   public isense::Uint32DataHandler,
   public isense::Int8DataHandler
#endif
{
   public:
      void init( Os::AppMainParameter& value )
      {
         radio_ = &wiselib::FacetProvider<Os, Os::Radio>::get_facet( value );
         timer_ = &wiselib::FacetProvider<Os, Os::Timer>::get_facet( value );
         debug_ = &wiselib::FacetProvider<Os, Os::Debug>::get_facet( value );
         rand_ = &wiselib::FacetProvider<Os, Os::Rand>::get_facet( value );

#ifdef SHAWN
         mid_ = ( uint16_t )rand_->operator()( 65536 / 2 );
         debug_->debug( "CoAP Application booting! %d", mid_ );
         add_resources();
         coap_.init( *radio_, *timer_, *debug_, mid_, resources );
         radio_->reg_recv_callback<CoapApplication, &CoapApplication::receive_radio_message>( this );
         if ( radio_->id() == 0 )
            timer_->set_timer<CoapApplication, &CoapApplication::simple_send>( 2000, this, 0 );
#endif
#ifdef ISENSE
         rand_->srand( radio_->id() );
         mid_ = ( uint16_t )rand_->operator()( 65536 / 2 );
         debug_->debug( "CoAP Application booting! %d", mid_ );
         em_ = new isense::EnvironmentModule( value );
         if ( em_ != NULL )
         {
            em_->enable( true );
            if ( em_->light_sensor()->enable() )
            {
               light_sensor_ = true;
               em_->light_sensor()->set_data_handler( this );
               //os().add_task_in(Time(10, 0), this, (void*) TASK_SET_LIGHT_THRESHOLD);
            }
            if ( em_->temp_sensor()->enable() )
            {
               temp_sensor_ = true;
               em_->temp_sensor()->set_data_handler( this );
            }
         }
         add_resources();
         coap_.init( *radio_, *timer_, *debug_, mid_, resources );
         radio_->reg_recv_callback<CoapApplication, &CoapApplication::receive_radio_message>( this );
         if ( radio_->id() != 0xca3 )
            timer_->set_timer<CoapApplication, &CoapApplication::init_thresholds>( 11000, this, 0 );
         if ( radio_->id() == 0xca3 )
            timer_->set_timer<CoapApplication, &CoapApplication::simple_send>( 10000, this, 0 );
#endif
      }
      void init_thresholds( void * )
      {
         if ( em_->temp_sensor()->enabled() )
         {
            em_->temp_sensor()->set_threshold( em_->temp_sensor()->temperature() + 1, em_->temp_sensor()->temperature() - 1 );
         }
         if ( em_->light_sensor()->enabled() )
         {
            if ( em_->light_sensor()->luminance() >= 20 )
               em_->light_sensor()->enable_threshold_interrupt(true, 5);
         }
      }
      // --------------------------------------------------------------------
      void simple_send( void* )
      {
         char path[] = "s/t/st\0";
         char payload[] = "0";
         block_data_t buf[100];
         uint8_t buf_len;

#ifdef SHAWN
         //uint16_t host = 1;
#endif
#ifdef ISENSE
         uint16_t host = 0x9979;
#endif
         uint8_t token[8];
         token[0] = 0xc1;
         token[1] = 0x45;
         token[2] = 0xaf;
         packet.init();
         packet.set_type( CON );
         packet.set_code( PUT );
         packet.set_mid( mid_++ );

         packet.set_uri_host( host );
         packet.set_uri_path_len( sizeof( path ) - 1 );
         packet.set_uri_path( path );
         packet.set_observe( 0 );
         packet.set_token_len( 3 );
         packet.set_token( token );

         packet.set_option( URI_HOST );
         packet.set_option( URI_PATH );
         packet.set_option( OBSERVE );
         packet.set_option( TOKEN );

         packet.set_payload((uint8_t *)payload);
         packet.set_payload_len(sizeof(payload));
         buf_len = packet.packet_to_buffer( buf );

         radio_->send( Os::Radio::BROADCAST_ADDRESS, buf_len, buf );
         //timer_->set_timer<CoapApplication, &CoapApplication::simple_send>( 30000, this, 0 );
      }
      // --------------------------------------------------------------------
      void receive_radio_message( Os::Radio::node_id_t from, Os::Radio::size_t len, Os::Radio::block_data_t *buf )
      {
         if ( buf[0] == WISELIB_MID_COAP )
         {
            //debug_->debug( "\n" );
            debug_->debug( "Node %x received msg from %x msg type: CoAP length: %d", radio_->id(), from, len );
            coap_.receiver( &len, buf, &from );
         }
      }

      void add_resources()
      {
         uint8_t i;
         // reg_resource(name, fast_response, observable_time - zero for non observable, expected_len, content_type )
         for( i = 0; i < CONF_MAX_RESOURCES; i++ )
         {
            resources[i].init();
         }
         i = 0;
         resources[i].set_method( GET );
         resources[i].reg_callback<CoapApplication, &CoapApplication::resource_discovery>( this );
         resources[i].reg_resource( ".well-known/core", false, 0, 1, APPLICATION_LINK_FORMAT );
         i++;
#ifdef ISENSE
         resources[i].set_method( GET );
         resources[i].reg_callback<CoapApplication, &CoapApplication::get_temp>( this );
         resources[i].reg_resource( "s/t", true, 1, 60, TEXT_PLAIN );
         temp_id = i;
         i++;
         resources[i].set_method( GET );
         resources[i].set_method( PUT );
         resources[i].reg_callback<CoapApplication, &CoapApplication::temp_status>( this );
         resources[i].reg_resource( "s/t/st", true, 0, 5, TEXT_PLAIN );
         i++;

         resources[i].set_method( GET );
         resources[i].reg_callback<CoapApplication, &CoapApplication::get_light>( this );
         resources[i].reg_resource( "s/l", true, 10*60, 1, TEXT_PLAIN );
         light_id = i;
         i++;
         resources[i].set_method( GET );
         resources[i].set_method( PUT );
         resources[i].reg_callback<CoapApplication, &CoapApplication::light_status>( this );
         resources[i].reg_resource( "s/l/st", true, 0, 5, TEXT_PLAIN );
         i++;
#endif
      }
      char* resource_discovery(uint8_t method)
      {
         memset( data_, 0, sizeof( data_ ) );
         coap_.coap_resource_discovery( data_ );
         //debug_->debug("CPP: %x - %s", data_, data_);
         return data_;
      }
#ifdef ISENSE
      void handle_int8_data( int8 value )
      {
         //notify temperature change and change threshold
         //debug_->debug( "NEW TEMP: %d", value );
         coap_.coap_notify_from_interrupt( temp_id );
         if (value > 0)
            em_->temp_sensor()->set_threshold( value + 1, value - 1 );
      }
      void handle_uint32_data( uint32 value )
      {
         //debug_->debug( "NEW LIGHT: %d", value );
         if ( value < 20 )
         {
            em_->light_sensor()->enable_threshold_interrupt(false, 5);
            //em_->light_sensor()->enable_threshold_interrupt(true, 5);
         }
         coap_.coap_notify_from_interrupt( light_id );
      }
#endif
      char* get_temp(uint8_t method)
      {
         int8_t temp = em_->temp_sensor()->temperature();
         debug_->debug( "temperature = %i °C", temp );
         sprintf( data_, "%d\0", temp );
         return data_;
      }
      char* temp_status( uint8_t method)
      {
         int8_t ret = -1;
         uint8_t * data = resources[temp_id+1].put_data();
         if ( method == GET )
         {
            ret = em_->temp_sensor()->enabled();

         }
         else if ( method == PUT )
         {
            if ( *data == 0x31 )
            {
               ret = em_->temp_sensor()->enable();
            }
            else if (*data == 0x30 )
            {
               ret = em_->temp_sensor()->disable();
            }

         }
         if ( ret == true )
         {
            sprintf( data_, "true" );
         }
         else if ( ret == false )
         {
            sprintf( data_, "false" );
         }
         else
         {
            sprintf( data_, "error" );
         }
         return data_;
      }

      char* get_light(uint8_t method)
      {
         uint32_t lux = em_->light_sensor()->luminance();
         debug_->debug( "luminance = %d lux", lux );
         sprintf( data_, "%d\0", lux );
         return data_;
      }

      char* light_status( uint8_t method)
      {
         int8_t ret = -1;
         uint8_t * data = resources[light_id+1].put_data();
         if ( method == GET )
         {
            ret = em_->light_sensor()->enabled();

         }
         else if ( method == PUT )
         {
            if ( *data == 0x31 )
            {
               ret = em_->light_sensor()->enable();
            }
            else if (*data == 0x30 )
            {
               ret = em_->light_sensor()->disable();
            }
         }
         if ( ret == true )
         {
            sprintf( data_, "true" );
         }
         else if ( ret == false )
         {
            sprintf( data_, "false" );
         }
         else
         {
            sprintf( data_, "error" );
         }
         return data_;
      }
   private:
      Os::Radio::self_pointer_t radio_;
      Os::Timer::self_pointer_t timer_;
      Os::Debug::self_pointer_t debug_;
      Os::Rand::self_pointer_t rand_;
      coap_t coap_;
      coap_packet_t packet;
      resource_t resources[CONF_MAX_RESOURCES];
      uint16_t mid_;

      char data_[128];
#ifdef ISENSE
      environment_module_t* em_;
      bool temp_sensor_, light_sensor_;
      uint8_t temp_id, light_id;
#endif
};
// --------------------------------------------------------------------------
wiselib::WiselibApplication<Os, CoapApplication> coap_app;
// --------------------------------------------------------------------------
void application_main( Os::AppMainParameter& value )
{
   coap_app.init( value );
}

