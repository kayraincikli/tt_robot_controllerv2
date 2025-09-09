
//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: sr04_sensor_driver
// Project Name: Gelisim Robot
// Description: 
//  10 us trig süresi, 150us-25ms echo back süresi, engel yoksa max 38ms exho back süresi
//  60ms'de bir çalışacak şekilde ayarlanmalıdır, 8 sensör 1 saniyede 2 kere çalışmış olur

//////////////////////////////////////////////////////////////////////////////////
module sr04_sensor_driver( 
    input wire clk,
    input wire reset,
    input wire clock_1MHz,
    input wire clock_1MHz_prev,
    
    input wire module_en,
    
    output reg trig_tx,
    input wire echo_rx, 
    
    output reg busy,
    output reg [8:0] distance
    );
    parameter trig_constant = 10;
    parameter min_echo_constant = 100000;

    reg echo_wait = 0;
    reg trig_start = 0;
    reg [15:0] trig_counter = 0;
    reg [15:0] echo_counter = 0;
    reg [31:0] negedge_debounce_counter = 0;
    reg negedge_debounce_en;
    reg [1:0] echo_sample;

    
    always@(posedge clk or posedge module_en) begin
        if (module_en) begin
            trig_counter = 0; 
            trig_start = 1;
            echo_wait = 0;
            negedge_debounce_counter = 0;
            negedge_debounce_en = 0;
        end else begin
            if(!clock_1MHz_prev && clock_1MHz) begin
                //send trigger signal for 10 us
                if (trig_start) begin
                    trig_tx = 1;
                    trig_counter = trig_counter + 1;
                end 
                if(trig_counter == trig_constant) begin
                    trig_start = 0;
                    trig_counter = 0;
                    trig_tx = 0;
                    echo_wait = 1;
                end
                
                 if(echo_wait && echo_rx) begin
                    echo_counter = echo_counter + 1; //us cinsinden echo süresi
                end
            
            end
            
            echo_sample = {echo_sample[0], echo_rx};
           
            if (echo_sample == 3'b01) begin //posedge
                negedge_debounce_counter = 0;
                negedge_debounce_en = 0;
            end
            
            if (echo_sample == 3'b10) begin // negedge
                negedge_debounce_en = 1;
                negedge_debounce_counter = 0;
            end
            
            if (negedge_debounce_en) begin
                negedge_debounce_counter = negedge_debounce_counter + 1;
            end
            
            if (echo_wait && (negedge_debounce_counter > min_echo_constant)) begin
                echo_wait = 0; 
                distance = echo_counter[14:6]; //pulse width(us) / 58 = distance (cm),  20000 / 343 = 58
                //distance = echo_counter / 116; //pulse width(us) / 58 = distance (cm),  20000 / 343 = 58
                echo_counter = 0;
                negedge_debounce_counter = 0;
                negedge_debounce_en = 0;
            end
            
            if (echo_rx) busy = 1;
            else busy = 0;
        end
    end
endmodule
