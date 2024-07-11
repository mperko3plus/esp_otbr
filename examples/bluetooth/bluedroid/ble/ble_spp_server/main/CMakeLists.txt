set(srcs "ble_spp_server.c"
    "board.c")
    
idf_component_register(SRCS "ble_spp_server_demo.c" "board.c"
                    INCLUDE_DIRS ".")
