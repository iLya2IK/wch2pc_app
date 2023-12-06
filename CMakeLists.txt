set(COMPONENT_ADD_INCLUDEDIRS .)

set(COMPONENT_SRCS "wch2pcapp.c")

set(COMPONENT_REQUIRES wc_ble_config wch2pc_esp32)
set(COMPONENT_PRIV_REQUIRES nvs_flash bt json)

register_component()
