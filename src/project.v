//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: robot_controller_top_module
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////

`default_nettype none

module tt_um_robot_controller_top_module (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);


    // List all unused inputs to prevent warnings
    wire _unused = &{ena, ui_in[4], ui_in[5], ui_in[6] ,1'b0};
        
    wire reset;
    wire robot_enable;
    
    wire SDA_master;
    wire SCL_master;
    wire SDA_oe_top;
    wire SDA_reg_top;
    
    wire UART_RX;
    wire UART_TX;
    
    wire A1, B1;
    
    wire trig_tx;
    wire echo_rx;
    wire [3:0] mux_sensor_select;  
    
    wire motor_driver_STBY;
    
    wire pwm_out_rightmotor;
    wire pwm_out_leftmotor;
    
    wire direction_rightmotor_1; 
    wire direction_rightmotor_2; 
    wire direction_leftmotor_1; 
    wire direction_leftmotor_2;
    
    wire target_reached;
    
    robot_top_module robot_top_module (.clk(clk), .reset(reset), .robot_enable(robot_enable), .SDA_master(SDA_master), .SCL_master(SCL_master), 
                                        .SDA_oe_top(SDA_oe_top), .SDA_reg_top(SDA_reg_top), .UART_RX(UART_RX), .UART_TX(UART_TX), .A1(A1), .B1(B1), .trig_tx(trig_tx), 
                                        .echo_rx(echo_rx), .mux_sensor_select(mux_sensor_select), .motor_driver_STBY(motor_driver_STBY), 
                                        .pwm_out_rightmotor(pwm_out_rightmotor), .pwm_out_leftmotor(pwm_out_leftmotor), 
                                        .direction_rightmotor_1(direction_rightmotor_1), .direction_rightmotor_2(direction_rightmotor_2), 
                                        .direction_leftmotor_1(direction_leftmotor_1), .direction_leftmotor_2(direction_leftmotor_2), .target_reached(target_reached));


    assign reset = ~rst_n;
    
    assign robot_enable = ui_in[0];
    assign A1           = ui_in[1];
    assign B1           = ui_in[2];
    assign echo_rx      = ui_in[3];
    assign UART_RX      = ui_in[7];
    
    assign uo_out[0]    = UART_TX;
    assign uo_out[1]    = SCL_master;
    assign uo_out[2]    = trig_tx;
    assign uo_out[3]    = mux_sensor_select[0];
    assign uo_out[4]    = mux_sensor_select[1];
    assign uo_out[5]    = mux_sensor_select[2];
    assign uo_out[6]    = mux_sensor_select[3];
    assign uo_out[7]    = target_reached;
    
    
    assign uio_out[1]   = motor_driver_STBY;
    assign uio_out[2]   = pwm_out_rightmotor;
    assign uio_out[3]   = pwm_out_leftmotor;
    assign uio_out[4]   = direction_rightmotor_1;
    assign uio_out[5]   = direction_rightmotor_2;
    assign uio_out[6]   = direction_leftmotor_1;
    assign uio_out[7]   = direction_leftmotor_2;
    
    assign uio_oe[1]    = 1'b1;
    assign uio_oe[2]    = 1'b1;
    assign uio_oe[3]    = 1'b1;
    assign uio_oe[4]    = 1'b1;
    assign uio_oe[5]    = 1'b1;
    assign uio_oe[6]    = 1'b1;
    assign uio_oe[7]    = 1'b1;
    
    assign uio_out[0]   = SDA_reg_top;
    assign SDA_master   = uio_in[0];
    assign uio_oe[0]    = SDA_oe_top;
    
endmodule