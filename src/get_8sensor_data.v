//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: get_8sensor_data
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////
module get_8sensor_data(
    input wire clk,
    input wire reset,
    
    input wire clock_1MHz,
    input wire clock_1MHz_prev, 
    
    input wire [7:0] critical_distance,
    input wire get_data_En,

    output wire trig_tx,
    input wire echo_rx, 
    
    output reg [3:0] mux_sensor_select,
    output reg [7:0]input_sensors_in,
    
    input wire [31:0] ms_counter
    );
        
    parameter module_en_posedge = 0;
    parameter module_en_negedge = 1;
    parameter wait_distance = 2;
    parameter sensor_select = 3;
    
    reg [1:0]state;
    reg [7:0] input_sensors_buffer;
    reg [31:0] wait_counter;
    
    //sensor_driver variables
    reg sensor_module_en;
    wire [8:0] distance;
    wire busy;
    
    sr04_sensor_driver sensor_driver (clk, reset, clock_1MHz, clock_1MHz_prev, sensor_module_en, trig_tx, echo_rx, busy, distance);
    
    reg [8:0] distance_ff1, distance_ff2;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            distance_ff1 <= 1'b0;
            distance_ff2<= 1'b0;
        end else begin
            distance_ff1 <= distance; // asenkron giriş
            distance_ff2 <= distance_ff1;     // senkron kopya
        end
    end
    
    always@(posedge clk or posedge reset) begin
        if(reset) begin
            wait_counter = 0;
            mux_sensor_select = 0;
            sensor_module_en = 1;
            input_sensors_buffer = 0;
            state = module_en_posedge;
        end 
        else begin
            if(clock_1MHz && !clock_1MHz_prev) begin
                if (get_data_En) begin
                    case (state)
                        module_en_posedge : begin
                            sensor_module_en = 1;
                            state = module_en_negedge; 
                        end
                        
                        module_en_negedge : begin
                            sensor_module_en = 0;
                            state = wait_distance;
                        end
                        
                        wait_distance : begin
                            wait_counter = wait_counter + 1;
                            if((wait_counter > ms_counter) && !busy) begin //60 ms wait
                                state = sensor_select;
                                wait_counter = 0;
                            end
                        end
                        
                        sensor_select : begin
                            if (distance_ff2 < critical_distance) begin
                                input_sensors_buffer[mux_sensor_select] = 0;
                            end else input_sensors_buffer[mux_sensor_select] = 1;
                            mux_sensor_select = mux_sensor_select + 1;
                            if (mux_sensor_select == 8) begin
                                input_sensors_in = input_sensors_buffer;
                                mux_sensor_select = 0;
                            end
                            state = module_en_posedge;
                        end
                        
                    endcase
                end
            end
        end
    end
endmodule
