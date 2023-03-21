idf_component_register(SRCS "PROJET_merged_i2c_ble.c" "./components/lsm6dso/lsm6dso_reg.c" "./components/lis2mdl/lis2mdl_reg.c" "./components/i2c/i2c_func.c" "./components/ble/ble_func.c"
                    INCLUDE_DIRS "." "components/lsm6dso" "components/lis2mdl" "components/i2c" "components/ble")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
