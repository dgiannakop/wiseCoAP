#include "external_interface/external_interface.h"
#include "util/delegates/delegate.hpp"
#include "util/pstl/map_static_vector.h"
#include "util/pstl/static_string.h"
//#include "algorithms/routing/tree/tree_routing.h"
#include "algorithms/routing/flooding/flooding_algorithm.h"
// SENSORS
#undef CORE_COLLECTOR
#undef WEATHER_COLLECTOR
#undef ENVIRONMENTAL_COLLECTOR
#undef SECURITY_COLLECTOR
#undef SOLAP_COLLECTOR
#undef USE_FLOODING

//Uncomment to enable the isense module
#define CORE_COLLECTOR
#define ENVIRONMENTAL_COLLECTOR
//#define SECURITY_COLLECTOR
//#define SOLAR_COLLECTOR
//#define WEATHER_COLLECTOR


//#define USE_FLOODING

//Uncomment to enable resources
//#define HELLO_RESOURCE
#define I_AM_ALIVE
//#define LARGE_RESOURCE
//#define URI_QUERY_TEST

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

//#define DEBUG_COAP
#ifdef DEBUG_COAP
#define DBG_C(X) X
#else
#define DBG_C(X)
#endif

#define TEMP_RESOURCE "temp"
#define LIGHT_RESOURCE "light"
#define PIR_RESOURCE "pir"
#define TASK_SLEEP 1
#define TASK_WAKE 2

typedef wiselib::iSenseExtendedTime<Os> ExtendedTime;
typedef wiselib::iSenseClockModel<Os, ExtendedTime> Clock;

#ifdef USE_FLOODING
typedef wiselib::Coap<Os, flooding_alogrithm_t, Os::Timer, Os::Debug, Os::Clock, Os::Rand, wiselib::StaticString> coap_t;
#else
typedef wiselib::Coap<Os, Os::Radio, Os::Timer, Os::Debug, Os::Clock, Os::Rand, wiselib::StaticString> coap_t;
#endif

class iSenseCoapCollectorApp :
#ifdef SECURITY_COLLECTOR
public isense::SensorHandler,
#endif
#ifdef SOLAR_COLLECTOR
public isense::SleepHandler,
#endif
public isense::Uint32DataHandler,
public isense::Int8DataHandler {
public:

    void init(Os::AppMainParameter& value) {
        ospointer = &value;
        radio_ = &wiselib::FacetProvider<Os, Os::Radio>::get_facet(value);
        timer_ = &wiselib::FacetProvider<Os, Os::Timer>::get_facet(value);
        debug_ = &wiselib::FacetProvider<Os, Os::Debug>::get_facet(value);
        rand_ = &wiselib::FacetProvider<Os, Os::Rand>::get_facet(value);
        clock_ = &wiselib::FacetProvider<Os, Os::Clock>::get_facet(value);
        uart_ = &wiselib::FacetProvider<Os, Os::Uart>::get_facet(value);

        radio_->set_channel(12);
#ifdef CORE_COLLECTOR
        cm_ = new isense::CoreModule(value);
        led_status_ = 0;
        cm_->led_off();
#endif
#ifdef WEATHER_COLLECTOR
        init_weather_module(value);
#endif
#ifdef SOLAR_COLLECTOR
        init_solar_module(value);
#endif
#ifdef ENVIRONMENTAL_COLLECTOR
        init_environmental_module(value);
#endif
#ifdef SECURITY_COLLECTOR
        init_security_module(value);
#endif

#ifdef SOLAR_COLLECTOR
        //debug_->debug("set timer");
        ((isense::Os *) ospointer)->allow_sleep(false);

        ((isense::Os *) ospointer)->allow_doze(false);

        timer_->set_timer<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::read_solar_sensors > (5000, this, (void*) TASK_WAKE);
#endif
#ifdef USE_FLOODING 
        //flooding algorithm init
        flooding_.init(*radio_, *debug_);
        flooding_.enable_radio();
        flooding_.reg_recv_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::receive_flooding_message > (this);
#endif
        radio_->enable_radio();
        // coap init
        rand_->srand(radio_->id());
        mid_ = (uint16_t) rand_->operator()(65536 / 2);
        debug_->debug("iSense CoAP Collector App %x", radio_->id());
        add_resources();
#ifdef USE_FLOODING 
        coap_.init(&flooding_, *timer_, *debug_, *clock_, mid_, *uart_);
#else
        coap_.init(*radio_, *timer_, *debug_, *clock_, mid_, *uart_);
#endif
        radio_->reg_recv_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::receive_radio_message > (this);

        uart_->reg_read_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::handle_uart_msg > (this);
        uart_->enable_serial_comm();

#ifdef I_AM_ALIVE
        timer_->set_timer<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::broadcast > (5000, this, 0);
        alive_broadcast_ = true;
#endif
    }

#ifdef USE_FLOODING

    void receive_flooding_message(Os::Radio::node_id_t from, Os::Radio::size_t len, Os::Radio::block_data_t *buf) {
        //debug_->debug( "received flooding at %x from %x with length: %d", radio_->id(), from, len );
        if (buf[0] == WISELIB_MID_COAP) {
            DBG_C(debug_->debug("FLOODING A CoAP message from iSense: %x with length: %d", from, len));
            coap_.receiver(&len, buf, from);
        }
        if (buf[0] == WISELIB_MID_COAP_RESP) {
            //debug_->debug( "FLOODING B CoAP message from iSense: %x with length: %d", from, len );
            //coap_.receiver( &len, buf, from );
            coap_.debug_hex(buf, len);
        }
    }
#endif

    void receive_radio_message(Os::Radio::node_id_t from, Os::Radio::size_t len, Os::Radio::block_data_t *buf) {
        if (buf[0] == WISELIB_MID_COAP_RESP) {
            //coap_.receiver( &len, buf, from );
            coap_.debug_hex(buf, len);
        } else if (buf[0] == WISELIB_MID_COAP) {
            debug_->debug("NORMAL CoAP message from iSense: %x with length: %d", from, len);
            //coap_.debug_hex( buf, len );
            coap_.receiver( &len, buf, from );
        } else if (buf[0] == 0x7f && buf[1] == 0x69 && buf[2] == 112 && buf[3] == WISELIB_MID_COAP) {
            //debug_->debug( "NORMAL CoAP message from xbee: %x with length: %d", from, len );
            coap_.debug_hex(&buf[3], len - 3);
            //coap_.receiver( &len, &buf[3], from );
        } else if (buf[0] == 0x7f && buf[1] == 0x69 && buf[2] == 112 && len == 10) {
            //debug_->debug( "here from xbee: %x", from );
            uint8_t * addr = (uint8_t*) & from;
            buf[1] = *addr;
            buf[2] = *(addr + 1);
            coap_.debug_hex(buf, len);
            //coap_.receiver( &len, &buf[3], from );
        }
    }

    void handle_uart_msg(Os::Uart::size_t len, Os::Uart::block_data_t *buf) {
        DBG_C(debug_->debug("UART CoAP message + with length: %d %x %x ", len, buf[2], WISELIB_MID_COAP));

        if (buf[2] == WISELIB_MID_COAP) {
            uint16_t *dest = (uint16_t*) buf;
            if (*dest == radio_->id()) {
                len -= 2;
                coap_.receiver(&len, &buf[2], radio_->id());
            } else {
                //debug_->debug( "UART CoAP message with length: %d dest: %x , %x==%x", len, *dest ,buf[2], WISELIB_MID_COAP);
#ifdef USE_FLOODING
                flooding_.send(*dest, len - 2, &buf[2]);
#endif
		radio_->send(*dest, len - 2, &buf[2]);

                // arduino part
                Os::Radio::block_data_t buf_arduino[Os::Radio::MAX_MESSAGE_LENGTH];
                buf_arduino[0] = 0x7f;
                buf_arduino[1] = 0x69;
                buf_arduino[2] = 112;
                //buf_arduino[3] = WISELIB_MID_COAP;
                memcpy(&buf_arduino[3], &buf[2], len - 2);
                coap_.debug_hex(buf_arduino, len + 1);
                radio_->send(*dest, len + 1, buf_arduino);
            }
        }
    }

    void add_resources() {
        /*
                 uint16_t i = 0;
                 char namez[3];
                 for(i=0; i<26; i++)
                 {
                    debug_->debug("%d",i);
                    namez[0] = 0x41 + i;
                    namez[1] = '\0';
                    resource_t resource( namez, "test", GET, true, 0, TEXT_PLAIN);
                    coap_.add_resource( resource);
                 }
                 for(i=0; i<CONF_MAX_RESOURCES-26; i++)
                 {
                    debug_->debug("%d",i);
                    namez[0] = 0x41;
                    namez[1] = 0x41 + i;
                    namez[2] = '\0';
                    resource_t resource( namez, "test", GET, true, 0, TEXT_PLAIN);
                    coap_.add_resource( resource);
                 }
         */
#ifdef CORE_COLLECTOR
        resource_t core_resource("led", GET | POST, true, 60, TEXT_PLAIN);
        core_resource.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::led > (this);
        coap_.add_resource(&core_resource);
#endif
#ifdef ENVIRONMENTAL_COLLECTOR
        resource_t new_resource(TEMP_RESOURCE, GET, true, 60, TEXT_PLAIN);
        new_resource.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::get_temp > (this);
        coap_.add_resource(&new_resource);

        resource_t new_resource2(LIGHT_RESOURCE, GET, true, 60, TEXT_PLAIN);
        new_resource2.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::get_light > (this);
        coap_.add_resource(&new_resource2);
#endif

#ifdef WEATHER_COLLECTOR
        resource_t new_resource3("temp", GET, true, 60, TEXT_PLAIN);
        new_resource3.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::get_weather_temp > (this);
        coap_.add_resource(&new_resource3);

        resource_t new_resource4("bpressure", GET, true, 60, TEXT_PLAIN);
        new_resource4.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::get_weather_bar > (this);
        coap_.add_resource(&new_resource4);
#endif

#ifdef SOLAR_COLLECTOR
        resource_t new_resource5("capacity", GET, true, 60, TEXT_PLAIN);
        new_resource5.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::solar_charge > (this);
        coap_.add_resource(&new_resource5);

        resource_t new_resource6("voltage", GET, true, 60, TEXT_PLAIN);
        new_resource6.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::solar_voltage > (this);
        coap_.add_resource(&new_resource6);

        resource_t new_resource7("current", GET, true, 60, TEXT_PLAIN);
        new_resource7.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::solar_current > (this);
        coap_.add_resource(&new_resource7);

        resource_t new_resource8("duty_cycle", GET, true, 60, TEXT_PLAIN);
        new_resource8.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::solar_duty_cycle > (this);
        coap_.add_resource(&new_resource8);

#endif

#ifdef SECURITY_COLLECTOR
        if (pir_ != NULL) {
            resource_t new_resource9(PIR_RESOURCE, GET, true, 30, TEXT_PLAIN);
            new_resource9.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::security_pir > (this);
            coap_.add_resource(&new_resource9);
        }
#endif

#ifdef HELLO_RESOURCE
        resource_t hello_resource("hello_world", GET, true, 5, TEXT_PLAIN);
        hello_resource.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::hello > (this);
        coap_.add_resource(&hello_resource);
#endif

#ifdef LARGE_RESOURCE
        resource_t large_resource("large", GET, true, 0, TEXT_PLAIN);
        large_resource.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::large > (this);
        coap_.add_resource(&large_resource);
#endif
#ifdef URI_QUERY_TEST
        resource_t query_resource("uri_query", GET, true, 0, TEXT_PLAIN);
        query_resource.reg_callback<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::uri_query > (this);
        coap_.add_resource(&query_resource);
#endif
    }

    void handle_int8_data(int8 value) {
        //notify temperature change and change threshold
        //debug_->debug( "NEW TEMP: %d", value );
        ///coap_.coap_notify_from_interrupt( temp_id );
        //if ( value > 0 )
        //em_->temp_sensor()->set_threshold( value + 1, value - 1 );
    }

    void handle_uint32_data(uint32 value) {
        //debug_->debug( "NEW LIGHT: %d", value );
        if (value < 20) {
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
    void init_environmental_module(Os::AppMainParameter& value) {
        em_ = new isense::EnvironmentModule(value);
        if (em_ != NULL) {
            em_->enable(true);
            if (em_->light_sensor()->enable()) {
                //em_->light_sensor()->set_data_handler( this );
                //os().add_task_in(Time(10, 0), this, (void*) TASK_SET_LIGHT_THRESHOLD);
                DBG_C(debug_->debug("em light"));
            }
            if (em_->temp_sensor()->enable()) {
                //em_->temp_sensor()->set_data_handler( this );
                DBG_C(debug_->debug("em temp"));
            }
        }
    }
#endif

#ifdef SECURITY_COLLECTOR

    /**
     * Initializes the Security Sensor Module
     * @param value pointer to os
     */
    void init_security_module(Os::AppMainParameter & value) {
        pir_timestamp_ = clock_->time();
        pir_ = new isense::PirSensor(value);
        pir_->set_sensor_handler(this);
        pir_->set_pir_sensor_int_interval(2000);
        if (pir_->enable()) {
            pir_sensor_ = true;
            //debug_->debug( "id::%x em pir", radio_->id() );
        }
    }
#endif

#ifdef WEATHER_COLLECTOR

    /**
     * Initializes the Weather Sensor Module
     * @param value pointer to os
     */
    void init_weather_module(Os::AppMainParameter& value) {
        ms_ = new isense::Ms55xx(value);
    }
#endif

#ifdef SOLAR_COLLECTOR

    void init_solar_module(Os::AppMainParameter& value) {
        debug_->debug("init_solar_module");
        // create SolarModule instance
        sm_ = new isense::SolarModule(value);

        // if allocation of SolarModule was successful
        if (sm_ != NULL) {
            //debug_->debug( "not null" );
            // read out the battery state
            isense::BatteryState bs = sm_->battery_state();
            // estimate battery charge from the battery voltage
            uint32 charge = sm_->estimate_charge(bs.voltage);
            // set the estimated battery charge
            sm_->set_battery_charge(charge);
            //debug_->debug( "initialized" );

        }

    }
#endif

#ifdef CORE_COLLECTOR

    coap_status_t led(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            *(args->output_data_len) = sprintf((char*) args->output_data, "%d", led_status_);
            return CONTENT;
        } else if (args->method == COAP_POST) {
            if (*(args->input_data) == 0x30) {
                led_status_ = 0;
		cm_->led_off();
                *(args->output_data_len) = sprintf((char*) args->output_data, "%d", led_status_);
                return CHANGED;
            } else if (*(args->input_data) == 0x31) {
                led_status_ = 1;
                *(args->output_data_len) = sprintf((char*) args->output_data, "%d", led_status_);
		cm_->led_on();
                return CHANGED;
            }
            return NOT_IMPLEMENTED;
        }
        return INTERNAL_SERVER_ERROR;
    }
#endif
#ifdef ENVIRONMENTAL_COLLECTOR

    coap_status_t get_temp(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            //int temp = 0;
            int8_t temp = em_->temp_sensor()->temperature();
            DBG_C(debug_->debug("temperature = %i °C", temp));
            *(args->output_data_len) = sprintf((char*) args->output_data, "%d\0", temp);
            return CONTENT;
        }
        return INTERNAL_SERVER_ERROR;
    }

    coap_status_t get_light(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            uint32_t lux = em_->light_sensor()->luminance();
            DBG_C(debug_->debug("luminance = %d lux", lux));
            *(args->output_data_len) = sprintf((char*) args->output_data, "%d\0", lux);
            return CONTENT;
        }
        return INTERNAL_SERVER_ERROR;
    }
#endif

#ifdef WEATHER_COLLECTOR

    coap_status_t get_weather_temp(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            ms_ = new isense::Ms55xx(*ospointer);
            ms_->reset();
            int16 temp = ms_->get_temperature();
            *(args->output_data_len) = sprintf((char*) args->output_data, "%d\0", temp / 10);
            return CONTENT;
        }
        return INTERNAL_SERVER_ERROR;
    }

    coap_status_t get_weather_bar(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            ms_ = new isense::Ms55xx(*ospointer);
            ms_->reset();
            int16 bpressure = ms_->read_pressure();
            *(args->output_data_len) = sprintf((char*) args->output_data, "%d\0", bpressure / 10);
            return CONTENT;
        }
        return INTERNAL_SERVER_ERROR;
    }
#endif

#ifdef SOLAR_COLLECTOR

    coap_status_t solar_charge(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            isense::BatteryState bs = sm_->control();
            //duty_cycle( bs );
            *args->output_data_len = sprintf((char*) args->output_data, "%d\0", bs.capacity);
            return CONTENT;
        }
        return INTERNAL_SERVER_ERROR;
    }

    coap_status_t solar_voltage(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            isense::BatteryState bs = sm_->control();
            //duty_cycle( bs );
            *args->output_data_len = sprintf((char*) args->output_data, "%d\0", bs.voltage);
            return CONTENT;
        }
        return INTERNAL_SERVER_ERROR;
    }

    coap_status_t solar_current(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            isense::BatteryState bs = sm_->control();
            //duty_cycle( bs );
            *args->output_data_len = sprintf((char*) args->output_data, "%d\0", bs.current);
            return CONTENT;
        }
        return INTERNAL_SERVER_ERROR;
    }

    coap_status_t solar_duty_cycle(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            //isense::BatteryState bs = sm_->control();
            //duty_cycle( bs );
            *args->output_data_len = sprintf((char*) args->output_data, "%d\0", duty_cycle_);
            return CONTENT;
        }
        return INTERNAL_SERVER_ERROR;
    }
#endif

#ifdef SECURITY_COLLECTOR

    coap_status_t security_pir(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            uint8_t ret_val;
            Clock::time_t diff = clock_->time() - pir_timestamp_;
            //debug_->debug( "%d", clock_->seconds( diff ) );
            if (clock_->seconds(diff) < 10) {
                ret_val = 1;
            } else {
                ret_val = 0;
            }
            *(args->output_data_len) = sprintf((char*) args->output_data, "%d\0", ret_val);
            return CONTENT;
        }
        return INTERNAL_SERVER_ERROR;
    }
#endif
#ifdef HELLO_RESOURCE

    coap_status_t hello(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            *(args->output_data_len) = sprintf((char*) args->output_data, "hello from %x device!\0", radio_->id());
            return CONTENT;
        }
        return INTERNAL_SERVER_ERROR;
    }
#endif
#ifdef I_AM_ALIVE_RESOURCE

    coap_status_t alive_broadcast(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            if (alive_broadcast_ == true)
                *(args->output_data_len) = sprintf((char*) args->output_data, "disable POST:0");
            else
                *(args->output_data_len) = sprintf((char*) args->output_data, "enable POST:1");
            return CONTENT;
        } else if (args->method == COAP_POST) {
            if ((*(args->input_data) == 0x30) && (alive_broadcast_ == true)) {
                alive_broadcast_ = 0;
                *(args->output_data_len) = sprintf((char*) args->output_data, "disabled");
                return CHANGED;
            } else if ((*args->input_data == 0x31) && (alive_broadcast_ == false)) {
                alive_broadcast_ = 1;
                *(args->output_data_len) = sprintf((char*) args->output_data, "enabled");
                return CHANGED;
            }
            return NOT_IMPLEMENTED;
        }
        return INTERNAL_SERVER_ERROR;
    }
#endif
#ifdef LARGE_RESOURCE

    coap_status_t large(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            *(args->output_data_len) = sprintf((char*) args->output_data, "This is a large resource just to test the blockwise response. The text that follows is from LOTR.\n\nTheoden: Where is the horse and the rider? Where is the horn that was blowing? They have passed like rain on the mountain, like wind in the meadow. The days have gone down in the West behind the hills into shadow. How did it come to this?\n\n");
            *(args->output_data_len) += sprintf((char*) (args->output_data + *(args->output_data_len)), "Aragorn: Hold your ground, hold your ground! Sons of Gondor, of Rohan, my brothers! I see in your eyes the same fear that would take the heart of me. A day may come when the courage of men fails, when we forsake our friends and break all bonds of fellowship, but it is not this day. An hour of woes and shattered shields, when the age of men comes crashing down! But it is not this day! This day we fight! By all that you hold dear on this good Earth, I bid you *stand, Men of the West!*\0");
            return CONTENT;
        }
        return INTERNAL_SERVER_ERROR;
    }
#endif
#ifdef URI_QUERY_TEST

    coap_status_t uri_query(callback_arg_t* args) {
        if (args->method == COAP_GET) {
            if (strcmp(args->uri_queries->value_of("act"), "0") == 0)
                *(args->output_data_len) = sprintf((char*) args->output_data, "URI QUERY is working");
            else
                *(args->output_data_len) = sprintf((char*) args->output_data, "Wrong URI QUERY, act=0 is the right.");
            return CONTENT;
        }
        return INTERNAL_SERVER_ERROR;
    }
#endif

    bool stand_by(void) {
        return true;
    }

    bool hibernate(void) {
        return false;
    }

    void wake_up(bool memory_held) {
    }

#ifdef I_AM_ALIVE

    void broadcast(void*) {
        if (alive_broadcast_ == true) {
            //DBG_C(debug_->debug( "** I AM ALIVE **" ));
            block_data_t buf[11];
            //buf[0] = 0x7f;
            //buf[1] = 0x69;
            //buf[2] = 112;
            buf[0] = WISELIB_MID_COAP_RESP;
            buf[1] = (radio_->id() & 0xFF00) >> 8;
            buf[2] = radio_->id() & 0x00FF;
            strcpy((char*) &buf[3], "hereiam");
            //_->send( Os::Radio::BROADCAST_ADDRESS, 13 , buf );
            //coap_.debug_hex(buf, 10);
            radio_->send(Os::Radio::BROADCAST_ADDRESS, 10, buf);
            uart_->write(10, buf);

        }
        timer_->set_timer<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::broadcast > (60000, this, 0);
    }
#endif

#ifdef SOLAR_COLLECTOR

    void read_solar_sensors(void* userdata) {
        DBG_C(debug_->debug("read_solar_sensors"));
        if ((uint32) userdata == TASK_WAKE) {
            // register as a task to wake up again in one minute
            // the below call equals calling
            // add_task_in(Time(60,0), this, (void*)TASK_WAKE);
            timer_->set_timer<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::read_solar_sensors > (60000, this, (void*) TASK_WAKE);

            //initiate control cycle, and read out battery state
            isense::BatteryState bs = sm_->control();

            // prevent sleeping to keep node awake
            ((isense::Os *) ospointer)->allow_sleep(false);
            //            ospointer->allow_sleep(false);

            // adopt duty cycle to the remaining battery charge
            if (bs.charge < 50000) {
                // battery nearly empty -->
                // set ultra-low duty cycle
                // live ~20 days
                duty_cycle_ = 1; // 0.1%
            } else if (bs.capacity < 1000000) //1 Ah or less
            {
                //live approx. 9 days out of 1Ah
                // and then another 20 days at 0.1% duty cycle
                duty_cycle_ = 100;
            } else if (bs.capacity < 3000000) //3Ah or less
            {
                // live approx. 6 days out of 1Ah
                // and then another 9 days at 10% duty cycle
                // and then another 20 days at 0.1% duty cycle
                duty_cycle_ = 300; // 30%
            } else if (bs.capacity < 5000000/*2Ah*/) {
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
            // add task to allow sleeping again
            timer_->set_timer<iSenseCoapCollectorApp, &iSenseCoapCollectorApp::read_solar_sensors > (duty_cycle_ * 60000, this, (void*) TASK_SLEEP);

            // output battery state and duty cycle
                        debug_->debug("voltage=%dmV, charge=%iuAh ",bs.voltage ,bs.capacity );
//                        os().debug("voltage=%dmV, charge=%iuAh -> duty cycle=%d, currnt=%i",                    ,bs.voltage ,bs.capacity , );
        } else if ((uint32) userdata == TASK_SLEEP) {
            // allow sleeping again
            ((isense::Os *) ospointer)->allow_sleep(true);
            //            os().allow_sleep(true);
        }
    }
#endif

#ifdef SECURITY_COLLECTOR

    /**
     * Handles a new Pir Event
     * Reports the Reading to the Gateway
     */
    virtual void handle_sensor() {
        DBG_C(debug_->debug("pir event"));
        pir_timestamp_ = clock_->time();
        coap_.coap_notify_from_interrupt(PIR_RESOURCE);
    }
#endif
private:
    Os::Radio::self_pointer_t radio_;
    Os::Timer::self_pointer_t timer_;
    Os::Debug::self_pointer_t debug_;
    Os::Clock::self_pointer_t clock_;
    Os::Rand::self_pointer_t rand_;
    Os::Uart::self_pointer_t uart_;
#ifdef USE_FLOODING 
    flooding_algorithm_t flooding_;
#endif
    coap_t coap_;
    uint16_t mid_;
    bool pir_sensor_;
    Os::AppMainParameter* ospointer;
#ifdef I_AM_ALIVE
    bool alive_broadcast_;
#endif
#ifdef CORE_COLLECTOR
    isense::CoreModule* cm_;
    uint8_t led_status_;
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
};
// --------------------------------------------------------------------------
wiselib::WiselibApplication<Os, iSenseCoapCollectorApp> coap_app;
// --------------------------------------------------------------------------

void application_main(Os::AppMainParameter& value) {
    coap_app.init(value);
}
