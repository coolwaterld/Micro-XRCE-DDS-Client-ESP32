# set(CONFIG_PROFILE_COMMON_CREATE_ENTITIES CONFIG_PROFILE_CREATE_ENTITIES_REF OR CONFIG_PROFILE_CREATE_ENTITIES_XML)

set(COMPONENT_ADD_INCLUDEDIRS xrce-dds-client/include include)

#file(GLOB_RECURSE COMPONENT_SRCS xrce-dds-client/src/c/*.c)

# set(TRANSPORT_TYPES "UDP" "TCP" "SERIAL")

# # Transport sources.
# foreach(TRANS_TYPE ${TRANSPORT_TYPES})
#     if(CONFIG_PROFILE_${TRANS_TYPE}_TRANSPORT)
#         string(TOLOWER ${TRANS_TYPE} TT)
#         set(${TRANS_TYPE}_SRCS
#             xrce-dds-client/src/c/profile/transport/${TT}/${TT}_transport.c
#             xrce-dds-client/src/c/profile/transport/${TT}/${TT}_transport_linux.c
#             )
#     else()
#         set(${TRANS_TYPE}_SRCS )
#     endif()
# endforeach()

# # Transport discovery source.
# if(CONFIG_PROFILE_DISCOVERY AND CONFIG_PROFILE_UDP_TRANSPORT)
#     set(UDP_DISCOVERY_SRCS xrce-dds-client/src/c/profile/discovery/transport/udp_transport_datagram_linux.c)
# endif()

# Other sources
set(COMPONENT_SRCS
    xrce-dds-client/src/c/core/session/stream/input_best_effort_stream.c
    xrce-dds-client/src/c/core/session/stream/input_reliable_stream.c
    xrce-dds-client/src/c/core/session/stream/output_best_effort_stream.c
    xrce-dds-client/src/c/core/session/stream/output_reliable_stream.c
    xrce-dds-client/src/c/core/session/stream/stream_storage.c
    xrce-dds-client/src/c/core/session/stream/stream_id.c
    xrce-dds-client/src/c/core/session/stream/seq_num.c
    xrce-dds-client/src/c/core/session/session.c
    xrce-dds-client/src/c/core/session/session_info.c
    xrce-dds-client/src/c/core/session/submessage.c
    xrce-dds-client/src/c/core/session/object_id.c
    xrce-dds-client/src/c/core/serialization/xrce_types.c
    xrce-dds-client/src/c/core/serialization/xrce_header.c
    xrce-dds-client/src/c/core/serialization/xrce_subheader.c
    xrce-dds-client/src/c/util/time.c
    xrce-dds-client/src/c/profile/transport/ip/udp/udp_transport_posix.c
    xrce-dds-client/src/c/profile/transport/ip/udp/udp_transport.c
    xrce-dds-client/src/c/profile/discovery/transport/udp_transport_datagram_posix.c
    src/core/log/log.c
    xrce-dds-client/src/c/core/session/common_create_entities.c
    xrce-dds-client/src/c/core/session/create_entities_ref.c
    xrce-dds-client/src/c/core/session/create_entities_xml.c
    xrce-dds-client/src/c/core/session/create_entities_bin.c
    xrce-dds-client/src/c/core/session/read_access.c
    xrce-dds-client/src/c/core/session/write_access.c
    xrce-dds-client/src/c/profile/discovery/discovery.c   
    )

set(COMPONENT_PRIV_REQUIRES lwip microcdr newlib)

register_component()

