#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <uxr/client/client.h>
#include <ucdr/microcdr.h>
#include "HelloWorld.h"
#include "agv.h"

#define STREAM_HISTORY 8
#define BUFFER_SIZE UXR_CONFIG_UDP_TRANSPORT_MTU *STREAM_HISTORY
// BUFFER_SIZE 4096

#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR

void on_topic(
    uxrSession *session,
    uxrObjectId object_id,
    uint16_t request_id,
    uxrStreamId stream_id,
    struct ucdrBuffer *ub,
    uint16_t length,
    void *args)
{
    (void)session;
    (void)object_id;
    (void)request_id;
    (void)stream_id;
    (void)length;
    (void)args;

    Distance topic;
    Distance_deserialize_topic(ub, &topic);

    printf("Received angle:%ld,distance:%f \n", topic.angle, topic.distance);
}

static void xrce_client_task(void *pvParameters)
{

    uint32_t max_topics = 10;
    // Transport
    char *ip = "10.0.0.104";
    char *port = "8888";
    uxrUDPTransport transport;
    if (!uxr_init_udp_transport(&transport, UXR_IPv4, ip, port))
    {
        printf("Error at create transport.\n");
        return;
    }

    // Session
    uxrSession session;
    uxr_init_session(&session, &transport.comm, 0xAAAABBBB);
    uxr_set_topic_callback(&session, on_topic, NULL);

    if (!uxr_create_session(&session))
    {
        printf("Error at create session.\n");
        return;
    }
    //////////////////////////////////////////BIN RELIABILITY//////////////////////////////////////////////////////

    // Streams
    uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
    uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, BUFFER_SIZE,STREAM_HISTORY);

    uint8_t input_reliable_stream_buffer[BUFFER_SIZE];
    uxrStreamId reliable_in = uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE,STREAM_HISTORY);

    // Create entities
    uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    uint16_t participant_req = uxr_buffer_create_participant_bin(&session, reliable_out, participant_id, 0, NULL, UXR_REPLACE);

    uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    uint16_t topic_req_distance = uxr_buffer_create_topic_bin_key(&session, reliable_out, topic_id, participant_id, "TagValue_Distance","Distance", UXR_REPLACE, true);

    uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    uint16_t publisher_req = uxr_buffer_create_publisher_bin_partition(&session, reliable_out, publisher_id, participant_id, UXR_REPLACE, "AGV1");
    // UXR_DURABILITY_TRANSIENT_LOCAL //by ld
    uxrObjectId datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    uxrQoS_t qos = {.reliability = UXR_RELIABILITY_RELIABLE, .durability = UXR_DURABILITY_VOLATILE, .history = UXR_HISTORY_KEEP_LAST, .depth = 0};

    uint16_t datawriter_req = uxr_buffer_create_datawriter_bin(&session, reliable_out, datawriter_id, publisher_id, topic_id, qos, UXR_REPLACE);

    uxrObjectId subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
    uint16_t subscriber_req = uxr_buffer_create_subscriber_bin_partition(&session, reliable_out, subscriber_id, participant_id, UXR_REPLACE, "AGV1");

    uxrObjectId datareader_id = uxr_object_id(0x01, UXR_DATAREADER_ID);
    uint16_t datareader_req = uxr_buffer_create_datareader_bin(&session, reliable_out, datareader_id, subscriber_id, topic_id, qos, UXR_REPLACE);

    // Send create entities message and wait its status
    uint16_t requests[] = {participant_req, topic_req_distance, publisher_req, datawriter_req, subscriber_req, datareader_req};
    uint8_t status[sizeof(requests) / 2];
    if (!uxr_run_session_until_all_status(&session, 1000, requests, status, sizeof(status)))
    {
        printf("Error at create entities.\n");
        return;
    }

    // Request topics
    uxrDeliveryControl delivery_control = {0};
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    uxr_buffer_request_data(&session, reliable_out, datareader_id, reliable_in, &delivery_control);

    // Write topics
    bool connected = true;
    uint32_t count = 0;
    while (connected && count < max_topics)
    {
        ucdrBuffer ub;
        // distance
        Distance topic = {"distance", "AGV1", 1, {1, 2}, 20, 30.2};
        uint32_t topic_size = Distance_size_of_topic(&topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, datawriter_id, &ub, topic_size);
        Distance_serialize_topic(&ub, &topic);
        connected = uxr_run_session_time(&session, 1000);
    }
    //////////////////////////////////////////BIN RELIABILITY//////////////////////////////////////////////////////

    //////////////////////////////////////////BIN BEST EFFORT//////////////////////////////////////////////////////
    // // Streams
    // uint8_t output_best_effort_stream_buffer[UXR_CONFIG_UDP_TRANSPORT_MTU];
    // uxrStreamId best_effort_out = uxr_create_output_best_effort_stream(&session, output_best_effort_stream_buffer, UXR_CONFIG_UDP_TRANSPORT_MTU);
    // uxrStreamId best_effort_in = uxr_create_input_best_effort_stream(&session);


    // // Create entities
    // uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    // uint16_t participant_req = uxr_buffer_create_participant_bin(&session, best_effort_out, participant_id, 0, NULL, UXR_REPLACE);

    // uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    // uint16_t topic_req_distance = uxr_buffer_create_topic_bin_key(&session, best_effort_out, topic_id, participant_id, "TagValue_Distance",
    //                                                               "Distance", UXR_REPLACE, true);

    // uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    // uint16_t publisher_req = uxr_buffer_create_publisher_bin_partition(&session, best_effort_out, publisher_id, participant_id, UXR_REPLACE, "AGV1");
    // // UXR_DURABILITY_TRANSIENT_LOCAL //by ld
    // uxrObjectId datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    // // uxrQoS_t qos = {
    // //     .reliability = UXR_RELIABILITY_RELIABLE, .durability = UXR_DURABILITY_VOLATILE, .history = UXR_HISTORY_KEEP_LAST, .depth = 0};
    // uxrQoS_t qos = {
    //     .reliability = UXR_RELIABILITY_BEST_EFFORT, .durability = UXR_DURABILITY_VOLATILE, .history = UXR_HISTORY_KEEP_LAST, .depth = 0};
    // uint16_t datawriter_req = uxr_buffer_create_datawriter_bin(&session, best_effort_out, datawriter_id, publisher_id, topic_id, qos, UXR_REPLACE);

    // uxrObjectId subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
    // uint16_t subscriber_req = uxr_buffer_create_subscriber_bin_partition(&session, best_effort_out, subscriber_id, participant_id, UXR_REPLACE, "AGV1");

    // uxrObjectId datareader_id = uxr_object_id(0x01, UXR_DATAREADER_ID);
    // uint16_t datareader_req = uxr_buffer_create_datareader_bin(&session, best_effort_out, datareader_id, subscriber_id, topic_id, qos, UXR_REPLACE);

    // // Send create entities message and wait its status
    // uint16_t requests[] = {participant_req, topic_req_distance, publisher_req, datawriter_req, subscriber_req, datareader_req};
    // uint8_t status[sizeof(requests) / 2];
    // if (!uxr_run_session_until_all_status(&session, 1000, requests, status, sizeof(status)))
    // {
    //     printf("Error at create entities.\n");
    //     return;
    // }

    // // Request topics
    // uxrDeliveryControl delivery_control = {0};
    // delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    // uxr_buffer_request_data(&session, best_effort_out, datareader_id, best_effort_in, &delivery_control);

    // // Write topics
    // bool connected = true;
    // uint32_t count = 0;
    // while (connected && count < max_topics)
    // {
    //     ucdrBuffer ub;
    //     // distance
    //     Distance topic = {"distance", "AGV1", 1, {1, 2}, 20, 30.2};
    //     uint32_t topic_size = Distance_size_of_topic(&topic, 0);
    //     uxr_prepare_output_stream(&session, best_effort_out, datawriter_id, &ub, topic_size);
    //     Distance_serialize_topic(&ub, &topic);
    //     connected = uxr_run_session_time(&session, 1000);
    // }
    //////////////////////////////////////////BIN//////////////////////////////////////////////////////

    //////////////////////////////////////////XML//////////////////////////////////////////////////////
    // Streams
    // uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
    // uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, BUFFER_SIZE,
    //                                                              STREAM_HISTORY);

    // uint8_t input_reliable_stream_buffer[BUFFER_SIZE];
    // uxrStreamId reliable_in = uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE,
    //                                                            STREAM_HISTORY);

    // uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    // const char *participant_xml = "<dds>"
    //                               "<participant>"
    //                               "<rtps>"
    //                               "<name>default_xrce_participant</name>"
    //                               "</rtps>"
    //                               "</participant>"
    //                               "</dds>";
    // uint16_t participant_req = uxr_buffer_create_participant_xml(&session, reliable_out, participant_id, 0, participant_xml, UXR_REPLACE);

    // uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    // const char *topic_xml = "<dds>"
    //                         "<topic>"
    //                         "<name>HelloWorldTopic</name>"
    //                         "<dataType>HelloWorld</dataType>"
    //                         "</topic>"
    //                         "</dds>";
    // uint16_t topic_req = uxr_buffer_create_topic_xml(&session, reliable_out, topic_id, participant_id, topic_xml, UXR_REPLACE);

    // uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    // const char *publisher_xml = "";
    // uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session, reliable_out, publisher_id, participant_id, publisher_xml, UXR_REPLACE);

    // uxrObjectId datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    // const char *datawriter_xml = "<dds>"
    //                              "<data_writer>"
    //                              "<topic>"
    //                              "<kind>NO_KEY</kind>"
    //                              "<name>HelloWorldTopic</name>"
    //                              "<dataType>HelloWorld</dataType>"
    //                              "</topic>"
    //                              "</data_writer>"
    //                              "</dds>";
    // uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(&session, reliable_out, datawriter_id, publisher_id, datawriter_xml, UXR_REPLACE);

    // // Send create entities message and wait its status
    // uint8_t status[4];
    // uint16_t requests[4] = {participant_req, topic_req, publisher_req, datawriter_req};
    // if (!uxr_run_session_until_all_status(&session, 1000, requests, status, 4))
    // {
    //     printf("Error at create entities: participant: %i topic: %i publisher: %i darawriter: %i\n", status[0], status[1], status[2], status[3]);
    //     return;
    // }

    // // Write topics
    // bool connected = true;
    // uint32_t count = 0;
    // while (connected && count < max_topics)
    // {
    //     HelloWorld topic = {++count, "Hello DDS world!"};

    //     ucdrBuffer ub;
    //     uint32_t topic_size = HelloWorld_size_of_topic(&topic, 0);
    //     uxr_prepare_output_stream(&session, reliable_out, datawriter_id, &ub, topic_size);
    //     HelloWorld_serialize_topic(&ub, &topic);

    //     printf("Send topic: %s, id: %lu\n", topic.message, topic.index);
    //     connected = uxr_run_session_time(&session, 1000);
    // }
    //////////////////////////////////////////XML//////////////////////////////////////////////////////

    // Delete resources
    uxr_delete_session(&session);
    uxr_close_udp_transport(&transport);
    //////////////////////////////////////////CDR//////////////////////////////////////////////////////

    // Data buffer
    //  #define BUFFER_LENGTH 256

    // uint8_t buffer[BUFFER_LENGTH];

    // // Structs for handle the buffer.
    // ucdrBuffer writer;
    // ucdrBuffer reader;

    // // Initialize the MicroBuffers for working with an user-managed buffer.
    // ucdr_init_buffer(&writer, buffer, BUFFER_LENGTH);
    // ucdr_init_buffer(&reader, buffer, BUFFER_LENGTH);

    // // Serialize data
    // char input[16] = "Hello MicroCDR!"; //16 characters
    // ucdr_serialize_array_char(&writer, input, 16);

    // // Deserialize data
    // char output[16];
    // ucdr_deserialize_array_char(&reader, output, 16);

    // printf("Input: %s\n", input);
    // printf("Output: %s\n", output);
    //////////////////////////////////////////CDR//////////////////////////////////////////////////////

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(xrce_client_task, "udp_client", 4096 * 4, NULL, 5, NULL);
}
