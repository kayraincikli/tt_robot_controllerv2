//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: robot_top_module
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////
module robot_top_module(
    input clk,
    input reset,
    
    
    //Top Module Enable Pini
    input wire robot_enable,
    
    //Gyro I2C Master bağlantısı
    input wire SDA_master,
    output wire SCL_master,
    output wire SDA_oe_top,
    output wire SDA_reg_top,
    
    //UART Bağlantısı
    input wire UART_RX,
    output wire UART_TX,
    
    //Encoder Input Pins
    input wire A1, B1,
    
    //SR04 Sensor Pins
    output wire trig_tx,
    input wire echo_rx,  
    output wire [3:0] mux_sensor_select,  
    
    //Motor Outputs
    output wire motor_driver_STBY,
    
    output wire pwm_out_rightmotor,
    output wire pwm_out_leftmotor,
    
    output wire direction_rightmotor_1, 
    output wire direction_rightmotor_2, 
    output wire direction_leftmotor_1, 
    output wire direction_leftmotor_2,
    
    //Target Reached Flag
    output wire target_reached
    );
    
    parameter state_INITIALIZE  = 0;
    parameter state_INITIALIZE2 = 1;
    parameter state_ENABLE      = 2; 
    parameter state_IDLE        = 3;
     

    
    //#################################REGISTERS#################################
    //Internal Değişkenler
    reg [3:0] state;
    reg [31:0] gyro_delay_counter;

    //Robot Controller Değişkenleri
    reg robot_controller_en = 0;

    // Current Position Output
    wire signed [31:0] current_x;
    wire signed [31:0] current_y;

    //Gyro Driver Değişkenleri
    reg gyro_en;
    wire [15:0] yaw; 

    //Clock Divider Değişkenleri
    reg [31:0] clock_1MHz_counter = 0;
    reg clock_1MHz = 0;
    reg clock_1MHz_prev = 0;
    
    //soft_reset değişkenleri
    reg mem_tx_en;
    reg mem_config_transfer_en;
    wire mem_soft_reset_DONE;
    wire [7:0] input_sensors_in;
    wire [11:0] total_angle;
    wire [1:0] mode;
    wire [2:0] motor_out;
    
    wire [31:0] gyro_delay_counter_constant;
    wire [7:0] clock_1MHz_counter_constant;
    wire [31:0] period;
    wire [31:0] duty_cycle;
    wire [7:0] critical_distance;
    wire signed [31:0] initial_position_x;
    wire signed [31:0] initial_position_y;
    wire signed [31:0] target_position_x;
    wire signed [31:0] target_position_y;
    wire [15:0] STOP_THRESHOLD;
    wire [15:0] slow_clk_divider;
    wire [15:0] turn_constant;
    wire [31:0] PULSE_PER_CM;
    wire [7:0] DEBOUNCE_TIME;
    wire [31:0] ms_counter;
    wire [7:0] DEVICE_ADDR; 
    wire [7:0] PWR_MGMT_1;
    wire [7:0] SMPLRT_DIV;
    wire [7:0] INT_ENABLE;
    wire [7:0] GYRO_ZOUT; 
    wire [7:0] PWR_MGMT_1_data;
    wire [31:0] SMPLRT_DIV_data;
    wire [7:0] INT_ENABLE_data;
    wire [31:0] timer1;
    wire [31:0] timer2;
    wire [15:0] timer3;
    wire [15:0] timer4;
    wire [31:0] timer5;
    wire [15:0] dt;
    wire [13:0] uart_CLKS_PER_BIT;
    wire [7:0] ROBOT_TURN_COUNT;
    
    wire [15:0] ext_yaw;
    wire [15:0] robot_yaw_in;
    wire [7:0] ext_sensor_data;
    wire [7:0] ext_mode_select;
    
    assign robot_yaw_in = ext_mode_select[1] ? ext_yaw : yaw;
    //assign robot_yaw_in = yaw;
    
    memory_controller memory_controller (clk, reset, UART_RX, UART_TX, 
                                        //DATA_TX
                                        mem_tx_en, mem_config_transfer_en, mem_soft_reset_DONE,
                                        current_x, current_y, robot_yaw_in, target_reached, input_sensors_in, total_angle, mode, motor_out,
                                        //CONFIG_RX
                                        gyro_delay_counter_constant, clock_1MHz_counter_constant, period, duty_cycle,
                                        critical_distance, initial_position_x, initial_position_y, target_position_x, target_position_y,
                                        STOP_THRESHOLD, slow_clk_divider, turn_constant, PULSE_PER_CM, 
                                        DEBOUNCE_TIME, ms_counter, DEVICE_ADDR, PWR_MGMT_1, SMPLRT_DIV, INT_ENABLE,
                                        GYRO_ZOUT, PWR_MGMT_1_data, SMPLRT_DIV_data, INT_ENABLE_data, timer1, timer2, timer3, timer4,
                                        timer5, dt, uart_CLKS_PER_BIT, ext_yaw, ext_sensor_data , ext_mode_select, ROBOT_TURN_COUNT
                                        );
     
    robot_controller robot_controller (clk, reset, clock_1MHz, clock_1MHz_prev,
                            robot_controller_en, A1, B1, 
                            robot_yaw_in, trig_tx, echo_rx, critical_distance, mux_sensor_select,
                            period, duty_cycle, motor_driver_STBY,
                            pwm_out_rightmotor, pwm_out_leftmotor,
                            direction_rightmotor_1, direction_rightmotor_2,
                            direction_leftmotor_1, direction_leftmotor_2,
                            initial_position_x, initial_position_y,
                            target_position_x, target_position_y, 
                            current_x, current_y, target_reached, 
                            input_sensors_in, total_angle, mode,
                            motor_out, ROBOT_TURN_COUNT, STOP_THRESHOLD, 
                            slow_clk_divider, turn_constant,
                            PULSE_PER_CM, DEBOUNCE_TIME,
                            ms_counter, ext_sensor_data, ext_mode_select
                            );
    
    mpu6050_gyro_driver gyro_driver (clk, reset, gyro_en, yaw, 
                                    SDA_master, SCL_master, SDA_oe_top, SDA_reg_top,
                                    DEVICE_ADDR, PWR_MGMT_1, SMPLRT_DIV, INT_ENABLE, 
                                    GYRO_ZOUT, PWR_MGMT_1_data, SMPLRT_DIV_data,
                                    INT_ENABLE_data, timer1, timer2, timer3, 
                                    timer4, timer5, dt, motor_out);


    always@(posedge clk or posedge reset) begin
       if(reset) begin 
            clock_1MHz = 0;
            clock_1MHz_counter = 0;
        end
        else begin
            clock_1MHz_counter = clock_1MHz_counter + 1;
            clock_1MHz_prev = clock_1MHz;
            if (clock_1MHz_counter > clock_1MHz_counter_constant) begin 
                clock_1MHz = ~clock_1MHz;
                clock_1MHz_counter = 0;
            end
        end 
    end
    
    always@(posedge clk or posedge reset) begin
        if(reset) begin
            state = state_INITIALIZE;
            gyro_delay_counter = 0;
            mem_config_transfer_en = 0;
            mem_tx_en = 0;
            gyro_en = 0;
            gyro_delay_counter = 0;
        end
        else begin
            if (robot_enable) begin
                case (state)
                     state_INITIALIZE: begin
                        mem_config_transfer_en = 1;
                        if (mem_soft_reset_DONE) begin
                            state = state_INITIALIZE2;
                            mem_config_transfer_en = 0;
                        end else state = state_INITIALIZE;
                    end
                
                    state_INITIALIZE2: begin
                        gyro_en = 1;
                        gyro_delay_counter = gyro_delay_counter + 1;
                        if (gyro_delay_counter > gyro_delay_counter_constant) state = state_ENABLE; 
                    end
                    
                    
                    state_ENABLE : begin
                        robot_controller_en = 1;
                        mem_tx_en = 1;
                        gyro_en = 1;
                        if (target_reached) state = state_IDLE;
                    end
                    
                    state_IDLE : state = state_IDLE;
                    
                    default state = state_IDLE;
                endcase
            end
            else begin
                //gyro_en = 0;
                robot_controller_en = 0;
                //mem_tx_en = 0;
            end
        end
    end
endmodule
