#include "external_interface/external_interface.h"
#include "util/delegates/delegate.hpp"
#include "util/pstl/map_static_vector.h"
#include "util/pstl/static_string.h"
#include "algorithms/routing/tree/tree_routing.h"
// SENSORS
#undef CORE_COLLECTOR
#undef WEATHER_COLLECTOR
#undef ENVIRONMENTAL_COLLECTOR
#undef SECURITY_COLLECTOR
#undef SOLAP_COLLECTOR

//Uncomment to enable the isense module
#define CORE_COLLECTOR
//#define ENVIRONMENTAL_COLLECTOR
//#define SECURITY_COLLECTOR
//#define SOLAR_COLLECTOR
//#define WEATHER_COLLECTOR
//#define ND_COLLECTOR

//#define I_AM_ALIVE

#ifdef CORE_COLLECTOR
#include <isense/modules/core_module/core_module.h>
#endif
#ifdef ENVIRONMENTAL_COLLECTOR
#include <isense/modules/environment_module/environment_module.h>
#include <isense/modules/environment_module/temp_sensor.h>
#include <isense/modules/environment_module/light_sensor.h>
#endif
#ifdef SOLAR_COLLECTOR
#include <isense/modules/solar_module/solar_module.h>
#endif
#ifdef SECURITY_COLLECTOR
#include <isense/modules/security_module/pir_sensor.h>
#endif
#ifdef WEATHER_COLLECTOR
#include <isense/modules/cc_weather_module/ms55xx.h>
#endif

#include "algorithms/coap/coap.h"

#define DEBUG_COAP

#define TEMP_RESOURCE "temp"
#define LIGHT_RESOURCE "light"
#define PIR_RESOURCE "pir"


typedef wiselib::OSMODEL Os;
typedef Os::TxRadio Radio;
typedef Radio::node_id_t node_id_t;
typedef Radio::block_data_t block_data_t;

typedef wiselib::iSenseExtendedTime<Os> ExtendedTime;
typedef wiselib::iSenseClockModel<Os, ExtendedTime> Clock;

typedef wiselib::Coap<Os, Radio, Os::Timer, Os::Debug, Os::Clock, Os::Rand, wiselib::StaticString> coap_t;


class iSenseCoapCollectorApp:
#ifdef SECURITY_COLLECTOR
   public isense::SensorHandler,
#endif
#ifdef SOLAR_COLLECTOR
   public isense::SleepHandler,
#endif
   public isense::Uint32DataHandler,
   public isense::Int8DataHandler
{
   public:

      void init( Os::AppMainParameter& value ) {
         ospointer = &value;
         radio_ = &wiselib::FacetProvider<Os, Os::Radio>::get_facet( value );
         timer_ = &wiselib::FacetProvider<Os, Os::Timer>::get_facet( value );
         debug_ = &wiselib::FacetProvider<Os, Os::Debug>::get_facet( value );
         rand_ = &wiselib::FacetProvider<Os, Os::Rand>::get_facet( value );
         clock_ = &wiselib::FacetProvider<Os, Os::Clock>::get_facet( value );

         radio_->set_channel( 12 );

#ifdef CORE_COLLECTOR
         cm_ = new isense::CoreModule( value );
#endif
#ifdef WEATHER_COLLECTOR
         init_weather_module( value );
#endif
#ifdef SOLAR_COLLECTOR
         init_solar_module( value );
#endif
#ifdef ENVIRONMENTAL_COLLECTOR
         init_environmental_module( value );
#endif
#ifdef SECURITY_COLLECTOR
         init_security_module( value );
#endif


#ifdef CORE_COLLECTOR
         //send_reading( 0xffff, "led", 0 );
         cm_->led_off();
#endif
         // coap init
         rand_->srand( radio_->id() );
         mid_ = ( uint16_t ) rand_->operator()( 65536 / 2 );
         debug_->debug( "iSense CoAP Collector App" );
         add_resources();
         coap_.init( *radio_, *timer_, *debug_, *clock_, mid_ );
         radio_->reg_recv_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::receive_radio_message > ( this );

#ifdef I_AM_ALIVE
         timer_->set_timer<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::broadcast>(60000, this, 0);
         alive_broadcast_ = true;
#endif
      }
#ifdef ENVIRONMENTAL_COLLECTOR
      void init_thresholds( void * ) {
         if ( em_->temp_sensor()->enabled() ) {
            em_->temp_sensor()->set_threshold( em_->temp_sensor()->temperature() + 1, em_->temp_sensor()->temperature() - 1 );
         }
         if ( em_->light_sensor()->enabled() ) {
            if ( em_->light_sensor()->luminance() >= 20 )
               em_->light_sensor()->enable_threshold_interrupt( true, 5 );
         }
      }
      // --------------------------------------------------------------------
#endif
      void receive_radio_message( Os::Radio::node_id_t from, Os::Radio::size_t len, Os::Radio::block_data_t *buf ) {
         if ( buf[0] == WISELIB_MID_COAP ) {
            debug_->debug( "Node %x received msg from %x msg type: CoAP length: %d", radio_->id(), from, len );
#ifdef DEBUG_COAP
            debug_hex( buf, len );
#endif
            coap_.receiver( &len, buf, &from );
         }
         else if ( buf[0] == 0x7f && buf[1] == 0x69 && buf[2] == 112 && buf[3] == WISELIB_MID_COAP ) {
            debug_->debug( "Node %x received msg from %x msg type: CoAP length: %d", radio_->id(), from, len );
#ifdef DEBUG_COAP
            debug_hex( buf, len );
#endif
            coap_.receiver( &len, &buf[3], &from );
         }
      }

      void add_resources()
      {
#ifdef ENVIRONMENTAL_COLLECTOR
         resource_t new_resource( TEMP_RESOURCE, GET, true, 120, TEXT_PLAIN );
         new_resource.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::get_temp>( this );
         coap_.add_resource( new_resource );

         resource_t new_resource2( LIGHT_RESOURCE, GET, true, 60, TEXT_PLAIN );
         new_resource2.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::get_light>( this );
         coap_.add_resource( new_resource2 );
#endif

#ifdef WEATHER_MODULE
         resource_t new_resource3( "temp", GET, true, 120, TEXT_PLAIN );
         new_resource3.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::get_weather_temp>( this );
         coap_.add_resource( new_resource3 );

         resource_t new_resource4( "bpressure", GET, true, 60, TEXT_PLAIN );
         new_resource4.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::get_weather_bar>( this );
         coap_.add_resource( new_resource4 );
#endif

#ifdef SOLAR_MODULE
         resource_t new_resource5( "capacity", GET, true, 120, TEXT_PLAIN );
         new_resource5.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::solar_charge>( this );
         coap_.add_resource( new_resource5 );

         resource_t new_resource6( "voltage", GET, true, 60, TEXT_PLAIN );
         new_resource6.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::solar_voltage>( this );
         coap_.add_resource( new_resource6 );

         resource_t new_resource7( "current", GET, true, 120, TEXT_PLAIN );
         new_resource7.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::solar_current>( this );
         coap_.add_resource( new_resource7 );

         resource_t new_resource8( "duty_cycle", GET, true, 60, TEXT_PLAIN );
         new_resource8.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::solar_duty_cycle>( this );
         coap_.add_resource( new_resource8 );
#endif

#ifdef SECURITY_COLLECTOR
         resource_t new_resource9( PIR_RESOURCE, GET, true, 10, TEXT_PLAIN );
         new_resource9.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::security_pir>( this );
         coap_.add_resource( new_resource9 );
#endif
         resource_t hello_resource( "hello_world", GET, true, 0, TEXT_PLAIN );
         hello_resource.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::hello>( this );
         coap_.add_resource( hello_resource );

#ifdef I_AM_ALIVE
         resource_t alive_resource( "alive", GET | POST, true, 0, TEXT_PLAIN );
         alive_resource.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::alive_broadcast>( this );
         coap_.add_resource( alive_resource );
#endif
/*
         resource_t large_resource( "large", GET, true, 0, TEXT_PLAIN );
         large_resource.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::large>( this );
         coap_.add_resource( large_resource );
*/
      }

      void handle_int8_data( int8 value ) {
         //notify temperature change and change threshold
         //debug_->debug( "NEW TEMP: %d", value );
         ///coap_.coap_notify_from_interrupt( temp_id );
         //if ( value > 0 )
         //em_->temp_sensor()->set_threshold( value + 1, value - 1 );
      }

      void handle_uint32_data( uint32 value ) {
         //debug_->debug( "NEW LIGHT: %d", value );
         if ( value < 20 ) {
            //em_->light_sensor()->enable_threshold_interrupt( false, 5 );
            //em_->light_sensor()->enable_threshold_interrupt(true, 5);
         }
         ///coap_.coap_notify_from_interrupt( light_id );
      }

#ifdef ENVIRONMENTAL_COLLECTOR

      /**
       * Initializes the Environmental Sensor Module
       * @param value pointer to os
       */
      void init_environmental_module( Os::AppMainParameter& value ) {
         em_ = new isense::EnvironmentModule( value );
         if ( em_ != NULL ) {
            em_->enable( true );
            if ( em_->light_sensor()->enable() ) {
               em_->light_sensor()->set_data_handler( this );
               //os().add_task_in(Time(10, 0), this, (void*) TASK_SET_LIGHT_THRESHOLD);
               debug_->debug( "em light" );
            }
            if ( em_->temp_sensor()->enable() ) {
               em_->temp_sensor()->set_data_handler( this );
               debug_->debug( "em temp" );
            }
         }
      }
#endif

#ifdef SECURITY_COLLECTOR

      /**
       * Initializes the Security Sensor Module
       * @param value pointer to os
       */
      void init_security_module( Os::AppMainParameter & value ) {
         pir_timestamp_ = clock_->time();
         pir_ = new isense::PirSensor( value );
         pir_->set_sensor_handler( this );
         pir_->set_pir_sensor_int_interval( 2000 );
         if ( pir_->enable() ) {
            pir_sensor_ = true;
            debug_->debug( "id::%x em pir", radio_->id() );
         }

         //        accelerometer_ = new isense::LisAccelerometer(value);
         //        if (accelerometer_ != NULL) {
         //            accelerometer_->set_mode(MODE_THRESHOLD);
         //            accelerometer_->set_threshold(25);
         //            accelerometer_->set_handler(this);
         //            accelerometer_->enable();
         //        }
      }
#endif

#ifdef WEATHER_COLLECTOR

      /**
       * Initializes the Weather Sensor Module
       * @param value pointer to os
       */
      void init_weather_module( Os::AppMainParameter& value ) {
         ms_ = new isense::Ms55xx( value );
      }
#endif

#ifdef SOLAR_COLLECTOR
      void init_solar_module( Os::AppMainParameter& value ) {
         debug_->debug( "init_solar_module" );
         // create SolarModule instance
         sm_ = new isense::SolarModule( value );

         // if allocation of SolarModule was successful
         if ( sm_ != NULL ) {
            debug_->debug( "not null" );
            // read out the battery state
            isense::BatteryState bs = sm_->battery_state();
            // estimate battery charge from the battery voltage
            uint32 charge = sm_->estimate_charge( bs.voltage );
            // set the estimated battery charge
            sm_->set_battery_charge( charge );
            debug_->debug( "initialized" );

         }

      }
#endif

#ifdef ENVIRONMENTAL_COLLECTOR
      coap_status_t get_temp( uint8_t method, uint8_t* input_data, size_t input_data_len, uint8_t* output_data, uint16_t* output_data_len ) {
         if( method == COAP_GET )  {
            //int temp = 0;
            int8_t temp = em_->temp_sensor()->temperature();
            debug_->debug( "temperature = %i °C", temp );
            *output_data_len = sprintf( ( char* )output_data, "%d\0" , temp );
            return CONTENT;
         }
         return INTERNAL_SERVER_ERROR;
      }
      coap_status_t get_light( uint8_t method, uint8_t* input_data, size_t input_data_len, uint8_t* output_data, uint16_t* output_data_len ) {
         if( method == COAP_GET ) {
            uint32_t lux = em_->light_sensor()->luminance();
            debug_->debug( "luminance = %d lux", lux );
            *output_data_len = sprintf( ( char* )output_data, "%d\0", lux );
            return CONTENT;
         }
         return INTERNAL_SERVER_ERROR;
      }
#endif

#ifdef WEATHER_COLLECTOR
      coap_status_t get_weather_temp( uint8_t method, uint8_t* input_data, size_t input_data_len, uint8_t* output_data, uint16_t* output_data_len ) {
         if( method == COAP_GET ) {
            ms_ = new isense::Ms55xx( *ospointer );
            ms_->reset();
            int16 temp = ms_->get_temperature();
            *output_data_len = sprintf( ( char* )output_data, "%d\0" , temp / 10 );
            return CONTENT;
         }
         return INTERNAL_SERVER_ERROR;
      }
      coap_status_t get_weather_bar( uint8_t method, uint8_t* input_data, size_t input_data_len, uint8_t* output_data, uint16_t* output_data_len ) {
         if( method == COAP_GET ) {
            ms_ = new isense::Ms55xx( *ospointer );
            ms_->reset();
            int16 bpressure = ms_->read_pressure();
            *output_data_len = sprintf( ( char* )output_data, "%d\0" , bpressure / 10 );
            return CONTENT;
         }
         return INTERNAL_SERVER_ERROR;
      }
#endif

#ifdef SOLAR_COLLECTOR
      coap_status_t solar_charge( uint8_t method, uint8_t* input_data, size_t input_data_len, uint8_t* output_data, uint16_t* output_data_len ) {
         if( method == COAP_GET ) {
            isense::BatteryState bs = sm_->control();
            duty_cycle( bs );
            *output_data_len = sprintf( ( char* )output_data, "%d\0" , bs.capacity );
            return CONTENT;
         }
         return INTERNAL_SERVER_ERROR;
      }
      coap_status_t solar_voltage( uint8_t method, uint8_t* input_data, size_t input_data_len, uint8_t* output_data, uint16_t* output_data_len ) {
         if( method == COAP_GET ) {
            isense::BatteryState bs = sm_->control();
            duty_cycle( bs );
            *output_data_len = sprintf( ( char* )output_data, "%d\0" , bs.voltage );
            return CONTENT;
         }
         return INTERNAL_SERVER_ERROR;
      }
      coap_status_t solar_current( uint8_t method, uint8_t* input_data, size_t input_data_len, uint8_t* output_data, uint16_t* output_data_len ) {
         if( method == COAP_GET ) {
            isense::BatteryState bs = sm_->control();
            duty_cycle( bs );
            *output_data_len = sprintf( ( char* )output_data, "%d\0" , bs.current );
            return CONTENT;
         }
         return INTERNAL_SERVER_ERROR;
      }
      coap_status_t solar_duty_cycle( uint8_t method, uint8_t* input_data, size_t input_data_len, uint8_t* output_data, uint16_t* output_data_len ) {
         if( method == COAP_GET ) {
            isense::BatteryState bs = sm_->control();
            duty_cycle( bs );
            *output_data_len = sprintf( ( char* )output_data, "%d\0" , duty_cycle_ );
            return CONTENT;
         }
         return INTERNAL_SERVER_ERROR;
      }
#endif

#ifdef SECURITY_COLLECTOR
      coap_status_t security_pir( uint8_t method, uint8_t* input_data, size_t input_data_len, uint8_t* output_data, uint16_t* output_data_len ) {
         if( method == COAP_GET ) {
            uint8_t ret_val;
            Clock::time_t diff = clock_->time() - pir_timestamp_;
            debug_->debug( "%d", clock_->seconds( diff ) );
            if ( clock_->seconds( diff ) < 10 )
            {
               ret_val = 1;
            }
            else
            {
               ret_val = 0;
            }
            *output_data_len = sprintf( ( char* )output_data, "%d\0" , ret_val );
            return CONTENT;
         }
         return INTERNAL_SERVER_ERROR;
      }
#endif
      coap_status_t hello( uint8_t method, uint8_t* input_data, size_t input_data_len, uint8_t* output_data, uint16_t* output_data_len ) {
         if( method == COAP_GET ) {
            *output_data_len = sprintf( ( char* )output_data, "hello from %x device!\0", radio_->id() );
            return CONTENT;
         }
         return INTERNAL_SERVER_ERROR;
      }

#ifdef I_AM_ALIVE
      coap_status_t alive_broadcast( uint8_t method, uint8_t* input_data, size_t input_data_len, uint8_t* output_data, uint16_t* output_data_len ) {
         if( method == COAP_GET ) {
            if ( alive_broadcast_ == true )
               *output_data_len = sprintf( ( char* )output_data, "I am alive message is sent every 60s (to disable POST:0)" );
            else
               *output_data_len = sprintf( ( char* )output_data, "I am alive message is disabled (to enable POST:1)" );
            return CONTENT;
         }
         else if ( method == COAP_POST ) {
            if ( (*input_data == 0x30) && (alive_broadcast_ == true) )
            {
               alive_broadcast_ = 0;
               *output_data_len = sprintf( ( char* )output_data, "I am alive message disabled" );
               return CHANGED;
            }
            else if ( (*input_data == 0x31) && (alive_broadcast_ == false))
            {
               alive_broadcast_ = 1;
               *output_data_len = sprintf( ( char* )output_data, "I am alive message enabled" );
               return CHANGED;
            }
            return NOT_IMPLEMENTED;
         }
         return INTERNAL_SERVER_ERROR;
      }
#endif

      coap_status_t large( uint8_t method, uint8_t* input_data, size_t input_data_len, uint8_t* output_data, uint16_t* output_data_len ) {
         if( method == COAP_GET ) {
            *output_data_len = sprintf( ( char* )output_data, "This is a large resource just to test the blockwise response. The text that follows is from LOTR.\n\nTheoden: Where is the horse and the rider? Where is the horn that was blowing? They have passed like rain on the mountain, like wind in the meadow. The days have gone down in the West behind the hills into shadow. How did it come to this?\n\n" );
            *output_data_len += sprintf( (char* )(output_data + *output_data_len), "Aragorn: Hold your ground, hold your ground! Sons of Gondor, of Rohan, my brothers! I see in your eyes the same fear that would take the heart of me. A day may come when the courage of men fails, when we forsake our friends and break all bonds of fellowship, but it is not this day. An hour of woes and shattered shields, when the age of men comes crashing down! But it is not this day! This day we fight! By all that you hold dear on this good Earth, I bid you *stand, Men of the West!*\0");
            debug_->debug("Large len %d", *output_data_len);
            return CONTENT;
         }
         return INTERNAL_SERVER_ERROR;
      }

      bool stand_by( void ) {
         return true;
      }

      //----------------------------------------------------------------------------

      bool hibernate( void ) {
         return false;
      }

      //----------------------------------------------------------------------------

      void wake_up( bool memory_held ) {
      }

#ifdef I_AM_ALIVE
      void broadcast( void* )
      {
         if (alive_broadcast_ == true)
         {
            debug_->debug("** I AM ALIVE **");
            block_data_t buf[CONF_MAX_MSG_LEN];
            buf[0] = 0x7f;
            buf[1] = 0x69;
            buf[2] = 112;
            buf[3] = WISELIB_MID_COAP;
            buf[4] = 1;
            radio_->send(Os::Radio::BROADCAST_ADDRESS, 5 , buf);
         }
         timer_->set_timer<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::broadcast>(60000, this, 0);
      }
#endif

#ifdef DEBUG_COAP

      void debug_hex( const uint8_t * payload, size_t length ) {
         char buffer[2048];
         int bytes_written = 0;
         bytes_written += sprintf( buffer + bytes_written, "DATA:!" );
         for ( size_t i = 0; i < length; i++ ) {
            bytes_written += sprintf( buffer + bytes_written, "%x!", payload[i] );
         }
         bytes_written += sprintf( buffer + bytes_written, "" );
         buffer[bytes_written] = '\0';
         debug_->debug( "%s", buffer );
      }
#endif

#ifdef SOLAR_COLLECTOR
      void duty_cycle( isense::BatteryState bs )
      {
         if ( bs.charge < 50000 ) {
            // battery nearly empty -->
            // set ultra-low duty cycle
            // live ~20 days
            duty_cycle_ = 1; // 0.1%
         } else if ( bs.capacity < 1000000 ) //1 Ah or less
         {
            //live approx. 9 days out of 1Ah
            // and then another 20 days at 0.1% duty cycle
            duty_cycle_ = 100;
         } else if ( bs.capacity < 3000000 ) //3Ah or less
         {
            // live approx. 6 days out of 1Ah
            // and then another 9 days at 10% duty cycle
            // and then another 20 days at 0.1% duty cycle
            duty_cycle_ = 300; // 30%
         } else if ( bs.capacity < 5000000/*2Ah*/ ) {
            // live approx. 4 days out of 2Ah
            // and then another 6 days at 30%
            // and then another 9 days at 10% duty cycle
            // and then another 20 days at 0.1% duty cycle
            duty_cycle_ = 500; // 50%
         } else {
            // live approx. 1.5 days out of 1.4Ah
            // and then another 4 days at 40%
            // and then another 6 days at 30%
            // and then another 9 days at 10% duty cycle
            // and then another 20 days at 0.1% duty cycle
            duty_cycle_ = 880; // 88%
         }
      }
#endif

#ifdef SECURITY_COLLECTOR
      /**
           * Handles a new Pir Event
           * Reports the Reading to the Gateway
           */
      virtual void handle_sensor() {
         debug_->debug( "pir event" );
         pir_timestamp_ = clock_->time();
         coap_.coap_notify_from_interrupt( PIR_RESOURCE );
      }
#endif
   private:
      Os::Radio::self_pointer_t radio_;
      Os::Timer::self_pointer_t timer_;
      Os::Debug::self_pointer_t debug_;
      Os::Clock::self_pointer_t clock_;
      Os::Rand::self_pointer_t rand_;
      coap_t coap_;
      uint16_t mid_;
      bool pir_sensor_;
      Os::AppMainParameter* ospointer;
#ifdef I_AM_ALIVE
      bool alive_broadcast_;
#endif
#ifdef ND_COLLECTOR
      nb_t nb_;
#endif
#ifdef ENVIRONMENTAL_COLLECTOR
      isense::EnvironmentModule* em_;
#endif
#ifdef SECURITY_COLLECTOR
      isense::PirSensor* pir_;
      //    isense::LisAccelerometer* accelerometer_;
      Clock::time_t pir_timestamp_;
#endif
#ifdef WEATHER_COLLECTOR
      isense::Ms55xx* ms_;
#endif
#ifdef SOLAR_COLLECTOR
      isense::SolarModule* sm_;
      uint16_t duty_cycle_;
#endif
#ifdef CORE_COLLECTOR
      isense::CoreModule* cm_;
#endif

      //environment_module_t* em_;
      //bool temp_sensor_, light_sensor_;
      //uint8_t temp_id, light_id;
};
// --------------------------------------------------------------------------
wiselib::WiselibApplication<Os, iSenseCoapCollectorApp> coap_app;
// --------------------------------------------------------------------------

void application_main( Os::AppMainParameter& value ) {
   coap_app.init( value );
}
