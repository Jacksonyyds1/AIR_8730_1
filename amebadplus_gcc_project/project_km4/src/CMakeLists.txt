##########################################################################################
## * This part defines public part of the component
## * Public part will be used as global build configures for all component

set(public_includes)                #public include directories, NOTE: relative path is OK
set(public_definitions)             #public definitions
set(public_libraries)               #public libraries(files), NOTE: linked with whole-archive options


#----------------------------------------#
# Component public part, user config begin

# You may use if-else condition to set or update predefined variable above

# Component public part, user config end
#----------------------------------------#

#WARNING: Fixed section, DO NOT change!
ameba_global_include(${public_includes})
ameba_global_define(${public_definitions})
ameba_global_library(${public_libraries}) #default: whole-archived

##########################################################################################
## * This part defines private part of the component
## * Private part is used to build target of current component
## * NOTE: The build API guarantees the global build configures(mentioned above)
## *       applied to the target automatically. So if any configure was already added
## *       to public above, it's unnecessary to add again below.

#NOTE: User defined section, add your private build configures here
# You may use if-else condition to set these predefined variable
# They are only for ameba_add_internal_library/ameba_add_external_app_library/ameba_add_external_soc_library
set(private_sources)                 #private source files, NOTE: relative path is OK
set(private_includes)                #private include directories, NOTE: relative path is OK
set(private_definitions)             #private definitions
set(private_compile_options)         #private compile_options

#------------------------------#
# Component private part, user config begin
ameba_list_append(private_includes
    ../absolute_encoder
    ../axis_hal
)

ameba_list_append(private_sources
    ../absolute_encoder/absolute_encoder.cpp
    ../absolute_encoder/encoder_map.cpp
    ../absolute_encoder/smart_search_manager.cpp
    ../absolute_encoder/position_tracker.cpp
    ../absolute_encoder/input_preprocessor.cpp
    ../absolute_encoder/runtime_calibrator.cpp
    ../axis_hal/axis_controller.c
    ../axis_hal/axis_coordinate_transform.c
    ../axis_hal/axis_motion_control.c
    ../axis_hal/encoder.c
         main.c
         app_example.c
         multi_button.c
         TimeEvent.c
         step_motor.c
         Lcd.c
         Fan_motor.c
         task_manager.c
         lvg_lcd_adapter.c
         peripheral_wrapper.c
         )

# Component private part, user config end
#------------------------------#

#WARNING: Select right API based on your component's release/not-release/standalone

###NOTE: For open-source component, always build from source
ameba_add_internal_library(project_src
    p_SOURCES
        ${private_sources}
    p_INCLUDES
        ${private_includes}
    p_DEFINITIONS
        ${private_definitions}
    p_COMPILE_OPTIONS
        ${private_compile_options}
)

##########################################################################################