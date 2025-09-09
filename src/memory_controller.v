//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: memory_controller
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////
module memory_controller(
    input clk, 
    input reset,
    
    input wire UART_RX,
    output wire UART_TX,
    
    input wire tx_en,
    input wire config_transfer_en,
    output wire soft_reset_DONE,
    
    //DATA_TX
    input wire signed [31:0] current_x,
    input wire signed [31:0] current_y, 
    input wire [15:0] yaw,
    input wire target_reached,
    input wire [7:0] input_sensors_in,
    input wire [11:0] total_angle,
    input wire [1:0] mode,
    input wire [2:0] motor_out,
    
    //CONFIG_RX
    output wire [31:0] gyro_delay_counter_constant,
    output wire [7:0] clock_1MHz_counter_constant,
    output wire [31:0] period,
    output wire [31:0] duty_cycle,
    output wire [7:0] critical_distance,
    output wire signed [31:0] initial_position_x,
    output wire signed [31:0] initial_position_y,
    output wire signed [31:0] target_position_x,
    output wire signed [31:0] target_position_y,
    output wire [15:0] STOP_THRESHOLD,
    output wire [15:0] slow_clk_divider,
    output wire [15:0] turn_constant,
    output wire [31:0] PULSE_PER_CM,
    output wire [7:0] DEBOUNCE_TIME,
    output wire [31:0] ms_counter,
    output wire [7:0] DEVICE_ADDR,
    output wire [7:0] PWR_MGMT_1,
    output wire [7:0] SMPLRT_DIV,
    output wire [7:0] INT_ENABLE,
    output wire [7:0] GYRO_ZOUT,
    output wire [7:0] PWR_MGMT_1_data,
    output wire [31:0] SMPLRT_DIV_data,
    output wire [7:0] INT_ENABLE_data,
    output wire [31:0] timer1,
    output wire [31:0] timer2,
    output wire [15:0] timer3,
    output wire [15:0] timer4,
    output wire [31:0] timer5,
    output wire [15:0] dt,
    output wire [13:0] uart_CLKS_PER_BIT,
    
    output wire [15:0] ext_yaw,
    output wire [7:0] ext_sensor_data,
    output wire [7:0] ext_mode_select,
    output wire [7:0] ROBOT_TURN_COUNT
    );
    

    data_tx data_tx (clk, reset, tx_en, current_x, current_y, target_position_x, target_position_y, 
                     yaw, target_reached, input_sensors_in, total_angle, mode,
                     motor_out, uart_CLKS_PER_BIT, UART_TX);
    
    config_rx config_rx (clk, reset, config_transfer_en, soft_reset_DONE, gyro_delay_counter_constant, clock_1MHz_counter_constant,
                        period, duty_cycle, critical_distance, initial_position_x, initial_position_y, 
                        target_position_x, target_position_y, STOP_THRESHOLD, slow_clk_divider, turn_constant,
                        PULSE_PER_CM, DEBOUNCE_TIME, ms_counter,
                        DEVICE_ADDR, PWR_MGMT_1, SMPLRT_DIV, INT_ENABLE, GYRO_ZOUT, PWR_MGMT_1_data, 
                        SMPLRT_DIV_data, INT_ENABLE_data, timer1, timer2, timer3, timer4, timer5,
                        dt, uart_CLKS_PER_BIT, ext_sensor_data, ext_mode_select, ext_yaw, UART_RX, ROBOT_TURN_COUNT);
    
endmodule
