cmake_minimum_required(VERSION 3.22)


# 设置机器人ID
# 包含机器人配置（定义ROBOT_NAME）
include(${CMAKE_CURRENT_SOURCE_DIR}/PYRo/cmake/config/pyro_robot_id_config.cmake)



# ==============================================================================
# 1. 编译器与工具链配置 (已由 gcc-arm-none-eabi.cmake 处理)
# ==============================================================================
include(cmake/gcc-arm-none-eabi.cmake)

# ==============================================================================
# 2. 项目定义
# ==============================================================================
set(CMAKE_PROJECT_NAME PYRo)
project(${CMAKE_PROJECT_NAME} C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()

# ==============================================================================
# 3. 额外性能优化 (保留工具链未包含的指令)
# ==============================================================================
add_compile_options(
        "-mthumb"
        "-Wdouble-promotion"
        "-funsafe-math-optimizations"
)

# ==============================================================================
# 4. CMSIS-DSP & dsppp 库引入
# ==============================================================================
set(DSP_REPO "${CMAKE_CURRENT_SOURCE_DIR}/Ported_third_lib/CMSIS-DSP")

add_compile_definitions(
        ARM_MATH_CM7
        ARM_MATH_LOOPUNROLL
)

set(CMSISDSP_BUILD_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)
add_subdirectory(${DSP_REPO} build_cmsis_dsp)

target_include_directories(CMSISDSP PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include"
)

add_library(dsppp_lib INTERFACE)
target_include_directories(dsppp_lib INTERFACE
        ${DSP_REPO}/dsppp/Include
        ${DSP_REPO}/Include
)

# ==============================================================================
# 5. 源文件配置 (按分类排序)
# ==============================================================================
add_executable(${CMAKE_PROJECT_NAME}
        # --- Core ---
        PYRo/Core/ETL/map.cpp
        PYRo/Core/Lock/pyro_mutex.cpp
        PYRo/Core/Lock/pyro_rw_lock.cpp
        PYRo/Core/Task/pyro_task.cpp
        PYRo/Core/Memory/pyro_core_dma_heap.c
        PYRo/Core/Memory/pyro_core_mem.cpp
        PYRo/Core/Task/pyro_task.cpp

        # --- Peripheral ---
        PYRo/Peripheral/CAN/pyro_can_drv.cpp
        PYRo/Peripheral/DWT/pyro_dwt_drv.cpp
        PYRo/Peripheral/UART/pyro_uart_drv.cpp

        # --- Algorithm ---
        PYRo/Algorithm/Kinematics/pyro_kin_hybrid.cpp
        PYRo/Algorithm/Kinematics/pyro_kin_mec.cpp
        PYRo/Algorithm/Kinematics/pyro_kin_rudder.cpp
        PYRo/Algorithm/OLS/pyro_algo_ols.cpp
        PYRo/Algorithm/PID/pyro_algo_pid.cpp
        PYRo/Algorithm/EKF/QuaternionEKF.cpp
        PYRo/Algorithm/KF/kalman_filter.cpp
        PYRo/Algorithm/Common/pyro_algo_common.cpp

        # --- Component ---
        PYRo/Component/CRC/PYRo_crc.cpp
        PYRo/Component/Motor/pyro_dji_motor_drv.cpp
        PYRo/Component/Motor/pyro_dm_motor_drv.cpp
        PYRo/Component/Motor/pyro_motor_base.cpp
        PYRo/Component/Powercontrol/pyro_power_control_drv.cpp
        PYRo/Component/Powermeter/pyro_powermeter.cpp
        PYRo/Component/RC/pyro_dr16_rc_drv.cpp
        PYRo/Component/RC/pyro_rc_base_drv.cpp
        PYRo/Component/RC/pyro_rc_hub.cpp
        PYRo/Component/RC/pyro_vt03_rc_drv.cpp
        PYRo/Component/Referee/fifo.c
        PYRo/Component/Referee/pyro_referee.cpp
        PYRo/Component/INS/pyro_ins.cpp
        PYRo/Component/Supercap/pyro_supercap_drv.cpp

        # --- Module ---
#        PYRo/Module/Chassis/Rudder/pyro_rud_chassis.cpp
#        PYRo/Module/Chassis/Rudder/fsm/pyro_rud_active_state.cpp
#        PYRo/Module/Chassis/Rudder/fsm/pyro_rud_passive_state.cpp
#        PYRo/Module/Chassis/Rudder/fsm/pyro_rud_moving_state.cpp
#        PYRo/Module/Chassis/Rudder/fsm/pyro_rud_braking_state.cpp
#        PYRo/Module/Chassis/Rudder/fsm/pyro_rud_turning_state.cpp
#        PYRo/Module/pyro_module_base.cpp
#        PYRo/Module/Yaw/pyro_yaw.cpp
#        PYRo/Module/Yaw/fsm/pyro_yaw_passive_state.cpp
#        PYRo/Module/Yaw/fsm/pyro_yaw_active_state.cpp

        # --- Application ---
        ## --- Demo ---
        PYRo/Application/Demo/demo_task.cpp
#        PYRo/Application/Demo/pyro_control_demo.cpp
#        PYRo/Application/Demo/pyro_controller_demo.cpp
        PYRo/Application/Demo/pyro_motor_demo.cpp
        PYRo/Application/Demo/pyro_rc_demo.cpp
        PYRo/Application/Demo/pyro_shoot_demo.cpp

        ## --- Mission ---
        PYRo/Application/Mission/pyro_init_thread.cpp
        PYRo/Application/Mission/pyro_mission_planer.cpp

        # --- Debug ---
        PYRo/Debug/Debug_task.cpp
        PYRo/Debug/JCOM/pyro_jcom.cpp
        PYRo/Debug/VOFA/pyro_vofa.cpp


        # --- Communication ---
        PYRo/Communication/pyro_com_cantx.cpp
        PYRo/Communication/pyro_com_canrx.cpp


        # --- Device ---
        PYRo/Device/BMI088/BMI088_driver.cpp
        PYRo/Device/BMI088/BMI088_middleware.cpp
#        PYRo/Module/Chassis/Rudder/fsm/pyro_rud_moving_state.cpp
#        PYRo/Module/Chassis/Rudder/fsm/pyro_rud_braking_state.cpp
#        PYRo/Module/Chassis/Rudder/fsm/pyro_rud_turning_state.cpp



)
if (ROBOT_ID EQUAL HERO_ID)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE

            PYRo/Module/Booster/Quad_pink/pyro_quad_booster.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_passive_state.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_active_state.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_homing_state.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_ready_state.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_stall_state.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_interim_state.cpp
            PYRo/Module/Booster/Quad_pink/fsm/pyro_quad_busy_state.cpp

            PYRo/Module/Gimbal/Hero/Direct/pyro_direct_gimbal.cpp
            PYRo/Module/Gimbal/Hero/Direct/fsm/pyro_direct_active_state.cpp
            PYRo/Module/Gimbal/Hero/Direct/fsm/pyro_direct_passive_state.cpp

            PYRo/Module/Chassis/Mecanum/pyro_mec_chassis.cpp
            PYRo/Module/Chassis/Mecanum/fsm/pyro_mec_passive_state.cpp
            PYRo/Module/Chassis/Mecanum/fsm/pyro_mec_active_state.cpp

            PYRo/Application/Mission/Hero/pyro_chassis_app.cpp
            PYRo/Application/Mission/Hero/pyro_gimbal_app.cpp
            PYRo/Application/Mission/Hero/pyro_booster_app.cpp
    )
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
            PYRo/Module/Booster/Quad_pink
            PYRo/Module/Gimbal/Hero/Direct
            PYRo/Module/Chassis/Mecanum
    )
elseif (ROBOT_ID EQUAL SUB_HERO_ID)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
            PYRo/Application/Mission/Hero_hybrid/pyro_chassis_app.cpp
            PYRo/Application/Mission/Hero_hybrid/pyro_booster_app.cpp

            PYRo/Module/Booster/Quad_tango/pyro_quad_booster.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_passive_state.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_active_state.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_homing_state.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_ready_state.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_stall_state.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_interim_state.cpp
            PYRo/Module/Booster/Quad_tango/fsm/pyro_quad_busy_state.cpp

            PYRo/Module/Chassis/Hybrid/pyro_hybrid_chassis.cpp
            PYRo/Module/Chassis/Hybrid/fsm/pyro_hybrid_active_state.cpp
            PYRo/Module/Chassis/Hybrid/fsm/pyro_hybrid_passive_state.cpp
            PYRo/Module/Chassis/Hybrid/fsm/pyro_hybrid_cruising_state.cpp
            PYRo/Module/Chassis/Hybrid/fsm/pyro_hybrid_climbing_state.cpp
    )
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
            PYRo/Module/Booster/Quad_tango
            PYRo/Module/Chassis/Hybrid
    )
elseif(ROBOT_ID EQUAL SENTRY_ID)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
            PYRo/Application/Mission/Sentry/pyro_sentry_chassis_app.cpp
            PYRo/Application/Mission/Sentry/pyro_sentry_gimbal_app.cpp
            PYRo/Application/Mission/Sentry/pyro_sentry_booster_app.cpp

            PYRo/Module/Chassis/Rudder/pyro_rud_chassis.cpp
            PYRo/Module/Chassis/Rudder/fsm/pyro_rud_active_state.cpp
            PYRo/Module/Chassis/Rudder/fsm/pyro_rud_passive_state.cpp
#            PYRo/Module/Chassis/Rudder/fsm/pyro_rud_moving_state.cpp
#            PYRo/Module/Chassis/Rudder/fsm/pyro_rud_braking_state.cpp
#            PYRo/Module/Chassis/Rudder/fsm/pyro_rud_turning_state.cpp
            
            PYRo/Module/Yaw/pyro_yaw.cpp
            PYRo/Module/Yaw/fsm/pyro_yaw_passive_state.cpp
            PYRo/Module/Yaw/fsm/pyro_yaw_active_state.cpp
            PYRo/Module/Yaw/fsm/pyro_yaw_manual_state.cpp
            PYRo/Module/Yaw/fsm/pyro_yaw_scanning_state.cpp

            PYRo/Module/Gimbal/Sentry/pyro_sentry_gimbal.cpp
            PYRo/Module/Gimbal/Sentry/fsm/pyro_sentry_gimbal_active_state.cpp
            PYRo/Module/Gimbal/Sentry/fsm/pyro_sentry_gimbal_passive_state.cpp
            PYRo/Module/Gimbal/Sentry/fsm/pyro_sentry_gimbal_scanning_state.cpp
            PYRo/Module/Gimbal/Sentry/fsm/pyro_sentry_gimbal_manual_state.cpp
            PYRo/Module/Gimbal/Sentry/fsm/pyro_sentry_gimbal_tracking_coarse.cpp
            PYRo/Module/Gimbal/Sentry/fsm/pyro_sentry_gimbal_tracking_fine.cpp
            PYRo/Module/Gimbal/Sentry/fsm/pyro_sentry_gimbal_tracking_state.cpp

    )
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
            PYRo/Module/Chassis/Rudder
            PYRo/Module/Yaw
            PYRo/Module/Gimbal/Sentry
    )
endif ()

add_subdirectory(cmake/stm32cubemx)

# ==============================================================================
# 6. 包含路径与链接 (按路径排序)
# ==============================================================================
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        # --- Core ---
        PYRo/Core/Config
        PYRo/Core/Def
        PYRo/Core/ETL
        PYRo/Core/FSM
        PYRo/Core/Lock
        PYRo/Core/Memory
        PYRo/Core/Task

        # --- Peripheral ---
        PYRo/Peripheral/CAN
        PYRo/Peripheral/DWT
        PYRo/Peripheral/UART

        # --- Device ---
        PYRo/Device/BMI088

        # --- Algorithm ---
        PYRo/Algorithm/Common
        PYRo/Algorithm/Kinematics
        PYRo/Algorithm/OLS
        PYRo/Algorithm/PID
        PYRo/Algorithm/EKF
        PYRo/Algorithm/KF

        # --- Component ---
        PYRo/Component/CRC
        PYRo/Component/IMU
        PYRo/Component/INS
        PYRo/Component/Motor
        PYRo/Component/Powercontrol
        PYRo/Component/Powermeter
        PYRo/Component/RC
        PYRo/Component/Referee
        PYRo/Component/Shoot
        PYRo/Component/Supercap

        # --- Module ---
        PYRo/Module
#        PYRo/Module/Chassis/Rudder
#        PYRo/Module/Yaw

        # --- Communication ---
        PYRo/Communication

        # --- Debug ---
        PYRo/Debug/VOFA
        PYRo/Debug/JCOM
)

target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        CMSISDSP
        dsppp_lib
)

add_dependencies(echo_robot_id ${CMAKE_PROJECT_NAME})

