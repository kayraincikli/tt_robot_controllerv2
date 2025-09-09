//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: data_tx
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////
module data_tx (
    input wire clk,
    input wire reset,
    input wire tx_en,
    
    input wire signed [31:0] current_x,
    input wire signed [31:0] current_y,
    input wire signed [31:0] target_position_x,
    input wire signed [31:0] target_position_y,
    input wire [15:0] yaw,
    input wire target_reached,
    input wire [7:0] input_sensors_in,
    input wire [11:0] total_angle,
    input wire [1:0] mode,
    input wire [2:0] motor_out,
    input wire [13:0] i_CLKS_PER_BIT,
    
    
    output wire TX_UART
);
    
    reg [5:0] state;
    reg [1:0] delay_counter;
    
    reg uart_Tx_DV;
    reg [7:0] uart_Tx_Byte;
    wire uart_Tx_Active;
    wire uart_Tx_Done;
    
    uart_tx uart_tx (clk, reset, uart_Tx_DV, uart_Tx_Byte, i_CLKS_PER_BIT, 
                    uart_Tx_Active, TX_UART, uart_Tx_Done);


    always@(posedge clk or posedge reset) begin
        if (reset) begin
            uart_Tx_DV <= 0;
            uart_Tx_Byte <= 0;
            state <= 31;
            delay_counter <=0;
        end 
        else begin
            if (tx_en) begin
                case (state)
                    31: begin
                        uart_Tx_Byte <= 8'h99;
                        uart_Tx_DV <= 1;
                        state <= 0;
                    end
                    0: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0; 
                                uart_Tx_Byte <= current_x [7:0];
                                uart_Tx_DV <= 1;
                                state <= 1;
                            end
                        end
                    end  
                    
                    1: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;   
                                uart_Tx_Byte <= current_x [15:8];
                                uart_Tx_DV <= 1;
                                state <= 2;
                            end 
                        end
                    end
                    
                    2: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= current_x [23:16];
                                uart_Tx_DV <= 1;
                                state <= 3;
                            end
                        end
                    end
                    
                    3: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= current_x [31:24];
                                uart_Tx_DV <= 1;
                                state <= 4;
                            end 
                        end
                    end
                    
                    4: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= current_y [7:0];
                                uart_Tx_DV <= 1;
                                state <= 5;
                            end 
                        end
                    end
                    
                    5: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= current_y [15:8];
                                uart_Tx_DV <= 1;
                                state <= 6;
                            end 
                        end
                    end
                    
                    6: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= current_y [23:16];
                                uart_Tx_DV <= 1;
                                state <= 7;
                            end 
                        end
                    end
                    
                    7: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= current_y [31:24];
                                uart_Tx_DV <= 1;
                                state <= 8;
                            end 
                        end
                    end
                    
                    8: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= yaw [7:0];
                                uart_Tx_DV <= 1;
                                state <= 9;
                            end 
                        end
                    end
                    
                    9: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= yaw [15:8];
                                uart_Tx_DV <= 1;
                                state <= 10;
                            end 
                        end
                    end
                    
                    10: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= input_sensors_in [7:0];
                                uart_Tx_DV <= 1;
                                state <= 11;
                            end 
                        end
                    end
                    
                    11: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= total_angle [7:0];
                                uart_Tx_DV <= 1;
                                state <= 12;
                            end 
                        end
                    end
                    
                    12: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= {target_reached, motor_out, total_angle [11:8]};
                                uart_Tx_DV <= 1;
                                state <= 13;
                            end 
                        end
                    end
                    
                    13: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= {mode, i_CLKS_PER_BIT[13:8]};
                                uart_Tx_DV <= 1;
                                state <= 14;
                            end 
                        end
                    end
                    
                    14: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= target_position_x[7:0];
                                uart_Tx_DV <= 1;
                                state <= 15;
                            end 
                        end
                    end
                    
                    15: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= target_position_x[15:8];
                                uart_Tx_DV <= 1;
                                state <= 16;
                            end 
                        end
                    end
                    
                    16: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= target_position_x[23:16];
                                uart_Tx_DV <= 1;
                                state <= 17;
                            end 
                        end
                    end
                    
                    17: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= target_position_x[31:24];
                                uart_Tx_DV <= 1;
                                state <= 18;
                            end 
                        end
                    end
                    
                    18: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= target_position_y[7:0];
                                uart_Tx_DV <= 1;
                                state <= 19;
                            end 
                        end
                    end
                    
                    19: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= target_position_y[15:8];
                                uart_Tx_DV <= 1;
                                state <= 20;
                            end 
                        end
                    end
                    
                    20: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= target_position_y[23:16];
                                uart_Tx_DV <= 1;
                                state <= 21;
                            end 
                        end
                    end
                    
                    21: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                uart_Tx_Byte <= target_position_y[31:24];
                                uart_Tx_DV <= 1;
                                state <= 30;
                            end 
                        end
                    end
                    
                    30: begin
                        if (uart_Tx_Active) begin
                            delay_counter <= 0;
                            uart_Tx_DV <= 0;
                        end
                        if (uart_Tx_Done) begin
                            delay_counter <= delay_counter + 1;
                            if (delay_counter == 1) begin
                                delay_counter <= 0;
                                state <= 31;
                            end
                        end
                    end
                    
                endcase
            end        
        end
    end
    
    
endmodule