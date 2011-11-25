/*
 * Simple Wiselib Example
 */
#include "external_interface/external_interface.h"
#include "algorithms/routing/tree/tree_routing.h"
// SENSORS
#include "util/delegates/delegate.hpp"
#include "util/pstl/map_static_vector.h"
#include "util/pstl/static_string.h"
#include "util/wisebed_node_api/sensors/sensor_controller.h"
#include "util/wisebed_node_api/sensors/managed_sensor.h"
// END OF SENSORS
#include "algorithms/coap/coap.h"

#ifdef ISENSE
    #include "external_interface/isense/isense_light_sensor.h"
    #include "external_interface/isense/isense_temp_sensor.h"
#endif

typedef wiselib::OSMODEL Os;
typedef Os::TxRadio Radio;
typedef Radio::block_data_t block_data_t;
typedef wiselib::Coap<Os,Radio,Os::Timer,Os::Debug, Os::Rand> coap_t;

// --------------------------------------------------------------------------
typedef wiselib::MapStaticVector<Os, uint8_t, wiselib::pair<wiselib::StaticString, delegate0<char*> >, 10> sensor_map_t;
typedef wiselib::SensorController<Os, sensor_map_t, wiselib::StaticString> sensor_controller_t;
#ifdef ISENSE
   typedef wiselib::iSenseLightSensor<Os, 30> LightSensor;
   typedef wiselib::iSenseTempSensor<Os> TempSensor;
   typedef wiselib::ManagedSensor<Os, LightSensor, wiselib::StaticString> ManagedLightSensor;
   typedef wiselib::ManagedSensor<Os, TempSensor, wiselib::StaticString> ManagedTempSensor;
#endif
// --------------------------------------------------------------------------
//typedef SingularCallback< SensorController, char *, unsigned char> Acallback;

class CoapApplication
{
public:
    void init( Os::AppMainParameter& value )
    {
        radio_ = &wiselib::FacetProvider<Os, Os::Radio>::get_facet( value );
        timer_ = &wiselib::FacetProvider<Os, Os::Timer>::get_facet( value );
        debug_ = &wiselib::FacetProvider<Os, Os::Debug>::get_facet( value );
        rand_ = &wiselib::FacetProvider<Os, Os::Rand>::get_facet(value);

        #ifdef ISENSE
            rand_->srand(radio_->id());
        #endif
        mid_ = (uint16_t)rand_->operator()(65536/2);

        debug_->debug( "CoAP Application booting! %d\n",mid_);

         #ifdef SHAWN

         #endif
         #ifdef ISENSE
            sensor_controller_.init( *debug_ );
            managed_light_sensor_.init_with_facetprovider( value, "light" );
            managed_temp_sensor_.init_with_facetprovider( value, "temperature" );
            sensor_controller_.add_sensor( 1, managed_light_sensor_.name(), managed_light_sensor_.sensor_delegate() );
            sensor_controller_.add_sensor( 2, managed_temp_sensor_.name(), managed_temp_sensor_.sensor_delegate() );
         #endif

        add_test_resources();

        coap_.init(*radio_, *timer_, *debug_, mid_, resources);

        radio_->reg_recv_callback<CoapApplication,&CoapApplication::receive_radio_message>( this );
        //coap_.send_simple();
        #ifdef SHAWN
            if (radio_->id() == 0)
                timer_->set_timer<CoapApplication,&CoapApplication::simple_send>( 2000, this, 0 );
        #endif
        #ifdef ISENSE
            if (radio_->id() == 0xca3)
                timer_->set_timer<CoapApplication,&CoapApplication::simple_send>( 10000, this, 0 );
        #endif
    }
    // --------------------------------------------------------------------
    void simple_send( void* )
    {
        char path[] = "temperature\0";
        block_data_t buf[100];
        uint8_t buf_len;

        #ifdef SHAWN
            uint16_t host = 1;
        #endif
        #ifdef ISENSE
            uint16_t host = 0x9979;
        #endif
        uint8_t token[8];
        token[0] = 0xc1;
        token[1] = 0x45;
        token[2] = 0xaf;
        packet.init();
        packet.set_type(CON);
        packet.set_code(GET);
        packet.set_mid(mid_++);

        packet.set_uri_host(host);
        packet.set_uri_path_len(sizeof(path) - 1);
        packet.set_uri_path(path);
        packet.set_observe(0);
        packet.set_token_len(3);
        packet.set_token(token);

        packet.set_option(URI_HOST);
        packet.set_option(URI_PATH);
        packet.set_option(OBSERVE);
        packet.set_option(TOKEN);

        buf_len = packet.packet_to_buffer(buf);

        radio_->send( Os::Radio::BROADCAST_ADDRESS, buf_len, buf );
        timer_->set_timer<CoapApplication,&CoapApplication::simple_send>( 25000, this, 0 );
    }
    // --------------------------------------------------------------------
    void receive_radio_message( Os::Radio::node_id_t from, Os::Radio::size_t len, Os::Radio::block_data_t *buf )
    {
        if (buf[0] == WISELIB_MID_COAP)
        {
            debug_->debug("\n");
            debug_->debug( "Node %x received msg from %x msg type: CoAP length: %d", radio_->id(), from, len);
            coap_.receiver(&len, buf, &from);
        }
    }

    void add_test_resources()
    {
        uint8_t i = 0;
        #ifdef SHAWN
            resources[i].reg_callback<CoapApplication, &CoapApplication::my_callback>(this);
            resources[i].reg_resource("temperature", 0, 4, TEXT_PLAIN);
            i++;
        #endif
        #ifdef ISENSE
            resources[i].reg_callback<sensor_controller_t, &sensor_controller_t::value>(&sensor_controller_);
            resources[i].reg_resource(managed_light_sensor_.name(), 1, 4, TEXT_PLAIN);
            i++;
            resources[i].reg_callback<sensor_controller_t, &sensor_controller_t::value>(&sensor_controller_);
            resources[i].reg_resource(managed_temp_sensor_.name(), 2, 1, TEXT_PLAIN);
            i++;
        #endif
        while ( i < CONF_MAX_RESOURCES)
        {
            resources[i++].init();
        }

    }
    char * my_callback(uint8_t i)
    {
        sprintf(test_, "%s", "test");
        return test_;
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

    char test_[50];
#ifdef ISENSE
      ManagedLightSensor managed_light_sensor_;
      ManagedTempSensor managed_temp_sensor_;
      sensor_controller_t sensor_controller_;
#endif
};
// --------------------------------------------------------------------------
wiselib::WiselibApplication<Os, CoapApplication> coap_app;
// --------------------------------------------------------------------------
void application_main( Os::AppMainParameter& value )
{
    coap_app.init( value );
}
