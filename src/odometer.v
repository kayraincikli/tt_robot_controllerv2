//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: odometer 
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////
module odometer ( 
    input  wire clk,               
    input  wire reset,
    input  wire clock_1MHz,
    input  wire clock_1MHz_prev,               
    input  wire [2:0]  motor_out,
    input  wire [11:0] total_angle,
    input  wire A1, B1,              // Motor 1 için enkoder sinyalleri
    output wire [1:0] direction1,    // encoder_debounce modülünde gelen ilgili yön sinyalleri
    output reg  signed [31:0] forward_distance_x,
    output reg  signed [31:0] forward_distance_y,
    input  wire [31:0] PULSE_PER_CM,
    input  wire [7:0]  DEBOUNCE_TIME
);

    // motor_out constant
    parameter motor_forward = 0;

    // Mesafe hesaplaması için değerler
    reg  signed [7:0]  cos_angle;
    reg  signed [7:0]  sin_angle;    // sin,cos değerleri
    reg  [15:0]        pulse_counter;
    wire [15:0] pulse_per_cm_in;
    wire [15:0] unused;
    reg  [1:0]         direction_past;
    reg                pulse_counter_flag;

    assign pulse_per_cm_in = PULSE_PER_CM[15:0];
    assign unused = PULSE_PER_CM[31:16];

    wire _unused_clock_1MHz_prev_odometer = clock_1MHz_prev;

    reg c1m_d1, c1m_d2;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            c1m_d1 <= 1'b0;
            c1m_d2 <= 1'b0;
        end else begin
            c1m_d1 <= clock_1MHz; // asenkron giriş
            c1m_d2 <= c1m_d1;     // senkron kopya
        end
    end
    // Rising edge: d1 1'e geçerken d2 hala 0 -> 1 clk süreli strobe
    wire tick_1MHz = c1m_d1 & ~c1m_d2;

    // Enkoder debounce modülü (A/B içeride senkronize edilir)
    encoder_debounce encoder1 (
        .clk(clk),
        .reset(reset),
        .clock_1MHz_prev(clock_1MHz_prev),
        .clock_1MHz(clock_1MHz),
        .A(A1),
        .B(B1),
        .direction(direction1),
        .DEBOUNCE_TIME(DEBOUNCE_TIME)
    );

    // Ana yapı
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // forward_distance_x <= 0;
            forward_distance_x <= 32'b0;
            forward_distance_y <= 0;
            pulse_counter      <= 0;
            direction_past     <= 0;
            pulse_counter_flag <= 0;
        end else begin
            if (tick_1MHz) begin
                if ((motor_out == motor_forward) && (direction1 != 2'b00)) begin
                    if (pulse_counter == (PULSE_PER_CM - 1)) begin
                        pulse_counter_flag <= 1'b1;
                        pulse_counter      <= 0;
                    end else begin
                        pulse_counter      <= pulse_counter + 1;
                        pulse_counter_flag <= 1'b0;
                    end
                    direction_past <= direction1;
                end else begin
                    pulse_counter_flag <= 1'b0;
                end

                if (pulse_counter_flag) begin
                    pulse_counter_flag <= 1'b0;
                    case (direction_past)
                        2'b10: begin
                            forward_distance_x <= forward_distance_x + sin_angle; // İleri x
                            forward_distance_y <= forward_distance_y + cos_angle; // İleri y
                        end
                        2'b01: begin
                            forward_distance_x <= forward_distance_x - sin_angle; // Geri x
                            forward_distance_y <= forward_distance_y - cos_angle; // Geri y
                        end
                        default: begin
                            forward_distance_x <= forward_distance_x; // Değişiklik yok
                            forward_distance_y <= forward_distance_y;
                        end
                    endcase
                end
            end

            if (forward_distance_x >= 32'b0111_1111_1111_1111_11111_1111_1111_1111)
                forward_distance_x <= 0;
            if (forward_distance_y >= 32'b0111_1111_1111_1111_11111_1111_1111_1111)
                forward_distance_y <= 0;
        end
    end

    // Açı -> yaklaşık sin/cos (±100 ölçeği)
    always @(posedge clk) begin
        case (total_angle)
            360, 720, 0: begin
                cos_angle <= 100;  sin_angle <=   0;
            end
            370,  10: begin
                cos_angle <=  98;  sin_angle <=  17;
            end
            380,  20: begin
                cos_angle <=  93;  sin_angle <=  34;
            end
            390,  30: begin
                cos_angle <=  86;  sin_angle <=  50;
            end
            400,  40: begin
                cos_angle <=  76;  sin_angle <=  64;
            end
            410,  50: begin
                cos_angle <=  64;  sin_angle <=  76;
            end
            420,  60: begin
                cos_angle <=  50;  sin_angle <=  86;
            end
            430,  70: begin
                cos_angle <=  34;  sin_angle <=  94;
            end
            440,  80: begin
                cos_angle <=  17;  sin_angle <=  98;
            end
            450,  90: begin
                cos_angle <=   0;  sin_angle <= 100;
            end
            460, 100: begin
                cos_angle <= -17;  sin_angle <=  98;
            end
            470, 110: begin
                cos_angle <= -34;  sin_angle <=  93;
            end
            480, 120: begin
                cos_angle <= -50;  sin_angle <=  86;
            end
            490, 130: begin
                cos_angle <= -64;  sin_angle <=  76;
            end
            500, 140: begin
                cos_angle <= -76;  sin_angle <=  64;
            end
            510, 150: begin
                cos_angle <= -86;  sin_angle <=  50;
            end
            520, 160: begin
                cos_angle <= -93;  sin_angle <=  34;
            end
            530, 170: begin
                cos_angle <= -98;  sin_angle <=  17;
            end
            540, 180: begin
                cos_angle <= -100; sin_angle <=   0;
            end
            550, 190: begin
                cos_angle <= -98;  sin_angle <= -17;
            end
            560, 200: begin
                cos_angle <= -93;  sin_angle <= -34;
            end
            570, 210: begin
                cos_angle <= -86;  sin_angle <= -50;
            end
            580, 220: begin
                cos_angle <= -76;  sin_angle <= -64;
            end
            590, 230: begin
                cos_angle <= -64;  sin_angle <= -76;
            end
            600, 240: begin
                cos_angle <= -50;  sin_angle <= -86;
            end
            610, 250: begin
                cos_angle <= -34;  sin_angle <= -94;
            end
            620, 260: begin
                cos_angle <= -17;  sin_angle <= -98;
            end
            630, 270: begin
                cos_angle <=  -0;  sin_angle <= -100;
            end
            640, 280: begin
                cos_angle <=  17;  sin_angle <= -98;
            end
            650, 290: begin
                cos_angle <=  34;  sin_angle <= -93;
            end
            660, 300: begin
                cos_angle <=  50;  sin_angle <= -86;
            end
            670, 310: begin
                cos_angle <=  64;  sin_angle <= -76;
            end
            680, 320: begin
                cos_angle <=  76;  sin_angle <= -64;
            end
            690, 330: begin
                cos_angle <=  86;  sin_angle <= -50;
            end
            700, 340: begin
                cos_angle <=  93;  sin_angle <= -34;
            end
            710, 350: begin
                cos_angle <=  98;  sin_angle <= -17;
            end
        endcase
    end

endmodule

//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: encoder_debounce 
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////
module encoder_debounce (
    input  wire clk,
    input  wire reset,
    input  wire clock_1MHz,
    input  wire clock_1MHz_prev,
    input  wire A,            // Enkoder A fazı
    input  wire B,            // Enkoder B fazı
    output reg  [1:0] direction, // Yön çıkışı (2'b10: İleri, 2'b01: Geri, 2'b00: Değişiklik yok)
    input  wire [7:0] DEBOUNCE_TIME
);

    // Debouncing parametreleri
    parameter IDLE    = 2'b00; // state machine
    parameter CHECK_A = 2'b01;
    parameter CHECK_B = 2'b10;

    wire _unused_clock_1MHz_prev_enc = clock_1MHz_prev;

    reg c1m_d1, c1m_d2;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            c1m_d1 <= 1'b0;
            c1m_d2 <= 1'b0;
        end else begin
            c1m_d1 <= clock_1MHz; // asenkron
            c1m_d2 <= c1m_d1;     // senkron kopya
        end
    end
    wire tick_1MHz = c1m_d1 & ~c1m_d2;

    reg A_d1, A_d2, B_d1, B_d2;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            A_d1 <= 1'b0; A_d2 <= 1'b0;
            B_d1 <= 1'b0; B_d2 <= 1'b0;
        end else begin
            A_d1 <= A;    A_d2 <= A_d1;
            B_d1 <= B;    B_d2 <= B_d1;
        end
    end
    wire A_sync = A_d2;
    wire B_sync = B_d2;

    // Internal registerlar
    reg [1:0] state;               // FSM durumu
    reg [15:0] debounce_counter;   // Debounce sayacı
    reg A_prev, B_prev;            // Önceki A ve B (senkron kopyalardan)
    reg A_stable, B_stable;        // Kararlı A ve B
    reg [1:0] encoder_state;       // Mevcut enkoder durumu (A ve B birleşik)
    reg [1:0] prev_encoder_state;  // Önceki enkoder durumu

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state               <= IDLE;
            debounce_counter    <= 0;
            A_prev              <= 0;
            B_prev              <= 0;
            A_stable            <= 0;
            B_stable            <= 0;
            direction           <= 2'b00; // Varsayılan: değişiklik yok
            encoder_state       <= 2'b00;
            prev_encoder_state  <= 2'b00;
        end else begin
            if (tick_1MHz) begin
                case (state)
                    IDLE: begin
                        if ((A_sync != A_prev) || (B_sync != B_prev)) begin
                            state            <= CHECK_A;
                            debounce_counter <= 0;
                        end else begin
                            direction <= 2'b00;
                        end
                    end

                    CHECK_A: begin
                        if (debounce_counter < DEBOUNCE_TIME) begin
                            debounce_counter <= debounce_counter + 1;
                        end else begin
                            A_stable         <= A_prev;
                            B_stable         <= B_prev;
                            state            <= CHECK_B;
                            debounce_counter <= 0;
                        end
                    end

                    CHECK_B: begin
                        if (debounce_counter < DEBOUNCE_TIME) begin
                            debounce_counter <= debounce_counter + 1;
                        end else begin
                            prev_encoder_state <= encoder_state;
                            encoder_state      <= {A_stable, B_stable};

                            // Enkoder geçişine göre yön belirleme
                            case ({prev_encoder_state, encoder_state})
                                4'b0001, 4'b0111, 4'b1110, 4'b1000: direction <= 2'b10; // İleri
                                4'b0010, 4'b1011, 4'b1101, 4'b0100: direction <= 2'b01; // Geri
                                default:                             direction <= 2'b00; // Yok
                            endcase

                            state <= IDLE;
                        end
                    end

                    default: state <= IDLE;
                endcase

                A_prev <= A_sync;
                B_prev <= B_sync;
            end
        end
    end

    // // Hata ayıklama için gerektiğinde izleme yapılabilir
    // always @(posedge clk) begin
    //     if (tick_1MHz) begin
    //         $display("t=%0t A=%b B=%b | Ap=%b Bp=%b | prev=%b curr=%b | dir=%b",
    //                  $time, A_sync, B_sync, A_prev, B_prev, prev_encoder_state, encoder_state, direction);
    //     end
    // end

endmodule