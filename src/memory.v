//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: memory
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////
module memory(
        input wire clk,                  // Saat sinyali
        input wire reset,
        input wire en,
        input wire we,                   // Yazma etkin sinyali (1: yaz, 0: oku)
        input wire [7:0] addr,           // Adres girişi
        input wire [7:0] data_in,        // Yazılacak veri
        
        output wire [7:0] data_out,       // Okunan veri
        
        output wire [15:0] ext_yaw_out,
        output wire [7:0] ext_sensor_data_out,
        output wire [7:0] ext_mode_select
    );
    
    // Veri genişliği 8 bit
    // Adres genişliği 8 bit
    reg [7:0] memory [85:0];

    assign ext_yaw_out = { memory[80], memory[79] }; 
    assign ext_sensor_data_out = memory[81];
    assign ext_mode_select =  memory[82];

    always @(*) begin //posedge clk or posedge reset
        if (reset) begin
            memory[0] = 8'b1;
            memory[1] = 8'b0;
            memory[2] = 8'b0;
            memory[3] = 8'b0;
            memory[4] = 8'b1;
            memory[5] = 8'b0;
            memory[6] = 8'b0;
            memory[7] = 8'b0;
            memory[8] = 8'b0010_0000;
            memory[9] = 8'b0100_1110;
            memory[10] = 8'b0;
            memory[11] = 8'b0;
            memory[12] = 8'b0010_0000;
            memory[13] = 8'b0100_1110;
            memory[14] = 8'b00000000;
            memory[15] = 8'b0;
            memory[16] = 8'b0;
            memory[17] = 8'b1010_0011;
            memory[18] = 8'b0;
            memory[19] = 8'b0001_0001;
            memory[20] = 8'b1110_1000;
            memory[21] = 8'b0000_0011;
            memory[22] = 8'b0;
            memory[23] = 8'b0;
            memory[24] = 8'b0001_1000;
            memory[25] = 8'b0001_1001;
            memory[26] = 8'b1111_0100;
            memory[27] = 8'b0000_0001;
            memory[28] = 8'b1010_1000;
            memory[29] = 8'b0110_0001;
            memory[30] = 8'b1110_1000;
            memory[31] = 8'b0000_0011;
            memory[32] = 8'b1100_1000;
            memory[33] = 8'b0000_0000; 
            memory[34] = 8'b0;
            memory[35] = 8'b0;
            memory[36] = 8'b1001_0000;
            memory[37] = 8'b0000_0001;
            memory[38] = 8'b0;
            memory[39] = 8'b0000_0100;
            memory[40] = 8'b1111_0100;
            memory[41] = 8'b0000_0001;
            memory[42] = 8'b0;
            memory[43] = 8'b0;
            memory[44] = 8'b0110_0000;
            memory[45] = 8'b1110_1010;
            memory[46] = 8'b0;
            memory[47] = 8'b0;
            memory[48] = 8'h68;
            memory[49] = 8'h6B;
            memory[50] = 8'h19;
            memory[51] = 8'h38;
            memory[52] = 8'h47;
            memory[53] = 8'b0;
            memory[54] = 8'b0000_0111;
            memory[55] = 8'b0000_0110;
            memory[56] = 8'b0001_1000;
            memory[57] = 8'b0001_1000;
            memory[58] = 8'h01;
            memory[59] = 8'b1000_0000;
            memory[60] = 8'b1111_0000;
            memory[61] = 8'b1111_1010;
            memory[62] = 8'b0000_0010;
            memory[63] = 8'b0;
            memory[64] = 8'b1100_0010;
            memory[65] = 8'b1110_1011;
            memory[66] = 8'b0000_1011;
            memory[67] = 8'b1110_0000;
            memory[68] = 8'b0010_1110;
            memory[69] = 8'b1110_0000;
            memory[70] = 8'b0010_1110;
            memory[71] = 8'b1100_0000;
            memory[72] = 8'b1100_0110;
            memory[73] = 8'b0010_1101;
            memory[74] = 8'b0;
            memory[75] = 8'b0;
            memory[76] = 8'h01;
            memory[77] = 8'b1011_0010;
            memory[78] = 8'b0000_0001;
            memory[79] = 8'b0110_1000;
            memory[80] = 8'b0000_0001;
            memory[81] = 8'b0;
            memory[82] = 8'b0000_0001;
            memory[83] = 8'b0010_0000;
            memory[84] = 8'b0000_0011;
            
        end
        else begin
            if (en && we) begin
                    memory[addr] = data_in;
            end  else memory[85] = 0;
        end
    end

    assign data_out = memory[addr];
    
endmodule