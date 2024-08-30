`timescale 1ns / 1ps

module d_flip_flop (
    input wire clk,    // 클럭 입력
    input wire reset,  // 리셋 입력
    input wire d,      // 데이터 입력
    output reg q       // 출력
);

// 항상 블록: 클럭의 상승 모서리 또는 리셋 신호의 변화에 반응
always @(posedge clk or posedge reset) begin
    if (reset) begin
        q <= 1'b0;    // 리셋이 활성화되면 출력 Q를 0으로 설정
    end else begin
        q <= d;       // 클럭 상승 모서리에서 입력 D를 출력 Q에 저장
    end
end

endmodule


module decoder_2X4_b(
    input [1:0] code,
    output reg[3:0] signal);
    
    always @(code)begin
        if(code == 2'b00) signal = 4'b0001;
    else if(code == 2'b01) signal = 4'b0010;
    else if(code == 2'b10) signal = 4'b0100;
    else if(code == 2'b11) signal = 4'b1000;
    
   end

endmodule

module sonic_cntr(
     input clk, reset_p,
     input echo,
     output reg trig,
     output reg [12:0] distance,
     output [15:0] led_debug
);
     
     parameter S_IDLE = 4'b0001;   
     parameter S_TRIG_PULSE = 4'b0010; 
     parameter S_WAIT_ECHO = 4'b0100; 
     parameter S_MEASURE_ECHO  = 4'b1000;
    
     parameter S_WAIT_PEDGE = 2'b01;
     parameter S_WAIT_NEDGE = 2'b10;
     
     reg [3:0] state, next_state;
     
     assign led_debug[3:0] = state;
     
     wire clk_usec;
     clock_div_100 usec_clk (.clk(clk), .reset_p(reset_p), .clk_div_100_nedge(clk_usec));
     
     reg [21:0] count_usec;
     reg count_usec_e;
     reg [7:0] data_count;
     reg [31:0] echo_start_time, echo_end_time;
     
     wire echo_pedge, echo_nedge;
     edge_detector_p ed(.clk(clk), .reset_p(reset_p), .cp(echo), .p_edge(echo_pedge), .n_edge(echo_nedge));
     
     always @(negedge clk or posedge reset_p) begin
        if (reset_p) 
            count_usec <= 0;
        else if (clk_usec && count_usec_e)
            count_usec <= count_usec + 1;
        else if (!count_usec_e)
            count_usec <= 0;
     end
       
     always @(negedge clk or posedge reset_p) begin 
        if (reset_p)
            state <= S_IDLE;
        else
            state <= next_state;
     end
     
     always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin 
            trig = 0;
            count_usec_e = 0;
            distance = 0;
            echo_start_time = 0;
            echo_end_time = 0;
            next_state = S_IDLE;
        end else begin
            case (state)
                S_IDLE: begin
                    trig <= 0;
                    count_usec_e = 1;
                    if (count_usec >= 22'd10_000) begin // 10ms 대기
                        count_usec_e = 0;
                        next_state = S_TRIG_PULSE;
                    end
                end    

                S_TRIG_PULSE: begin
                    if (count_usec > 22'd10) begin // 10us 트리거 펄스
                        count_usec_e = 0;
                        trig = 0;
                        next_state = S_WAIT_ECHO;  
                    end else begin
                        trig = 1;
                        count_usec_e = 1;
                                     
                    end
                end

                S_WAIT_ECHO: begin
                    if (echo_pedge) begin
                        echo_start_time = count_usec;
                        next_state = S_MEASURE_ECHO;
                        count_usec_e <= 1;
                    end
                end

                S_MEASURE_ECHO: begin
                    if (echo_nedge) begin
                        echo_end_time = count_usec;
                        data_count = echo_end_time - echo_start_time;
                        distance = (echo_end_time - echo_start_time) / 58;
                        next_state = S_IDLE;
                        count_usec_e = 0;
                    end else begin
                        count_usec_e = 1;
                    end
                end

                default: begin
                    next_state = S_IDLE;
                end    
            endcase
        end 
    end
endmodule

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module top_module (
    input wire clk,
    input wire reset_p,
    input [2:0] btn,         // 3비트 버튼 입력
    input wire btn_input,         // 버튼 입력 신호
    input wire btn4,     // 알람 끄기 버튼 입력
    output wire [3:0] com,
    output wire [7:0] seg_7,
    output reg led_lap,
    output reg led_alarm,
    output reg led_start,
    output buzz
);

    reg [1:0] mode_sel;           // 모드 선택 신호
    wire [3:0] com_watch, com_stopwatch, com_cooktimer;
    wire [7:0] seg_7_watch, seg_7_stopwatch, seg_7_cooktimer;
    wire led_lap_stopwatch;
    wire led_alarm_cooktimer, led_start_stopwatch, led_start_cooktimer;
    reg [3:0] com_selected;
    reg [7:0] seg_7_selected;

    wire btn_input_debounced;

    // btn_input 신호 디바운싱
    button_cntr btn_input_debouncer(
        .clk(clk), 
        .reset_p(reset_p), 
        .btn(btn_input), 
        .btn_pedge(btn_input_debounced)
    );

    // 모드 선택 신호 mode_sel의 값 설정 로직
    always @(posedge clk or posedge reset_p) begin
        if (reset_p)
            mode_sel <= 2'b00;  // 초기값 설정
        else if (btn_input_debounced) begin
            if (mode_sel == 2'b10)
                mode_sel <= 2'b00; // 2에서 0으로 되돌아갑니다.
            else
                mode_sel <= mode_sel + 1;
        end
    end


    // 각 모드의 enable 신호 생성
    wire enable_watch = (mode_sel == 2'b00);
    wire enable_stopwatch = (mode_sel == 2'b01);
    wire enable_cooktimer = (mode_sel == 2'b10);

    // 모드 선택에 따른 버튼 입력 분리
    wire [2:0] btn_watch = (enable_watch) ? btn : 3'b000;
    wire [2:0] btn_stopwatch = (enable_stopwatch) ? btn : 3'b000;
    wire [3:0] btn_cooktimer = (enable_cooktimer) ? btn : {btn4,3'b000};

    // 각 모드의 인스턴스 생성
    loadable_watch_top u_watch(
        .clk(clk),
        .reset_p(reset_p),
        .btn(btn_watch),      // 3비트 버튼 입력 사용
        .com(com_watch),
        .seg_7(seg_7_watch)
    );
    
    stop_watch_top_csec u_stopwatch(
        .clk(clk),
        .reset_p(reset_p),
        .btn(btn_stopwatch),  // 3비트 버튼 입력 사용
        .com(com_stopwatch),
        .seg_7(seg_7_stopwatch),
        .led_start(led_start_stopwatch),
        .led_lap(led_lap_stopwatch)
    );
    
    cook_timer_top u_cooktimer(
         .clk(clk),
         .reset_p(reset_p),
         .btn(btn_cooktimer), // 알람 끄기 버튼 포함하여 4비트 입력 사용
         .btn4(btn4),
         .com(com_cooktimer),
         .seg_7(seg_7_cooktimer),
         .led_alarm(led_alarm_cooktimer),
         .led_start(led_start_cooktimer),
         .buzz(buzz)
    );

    // 모드 선택에 따른 출력 선택
    always @(*) begin
        
        case (mode_sel)
            2'b00: begin // Watch 모드
                com_selected = com_watch;
                seg_7_selected = seg_7_watch;
                led_lap = 1'b0;
                led_alarm = 1'b0;
                led_start = 1'b0;
              
            end
            2'b01: begin // Stopwatch 모드
                com_selected = com_stopwatch;
                seg_7_selected = seg_7_stopwatch;
                led_lap = led_lap_stopwatch;
                led_alarm = 1'b0;
                led_start = led_start_stopwatch;
            end
            2'b10: begin // Cooktimer 모드
                com_selected = com_cooktimer;
                seg_7_selected = seg_7_cooktimer;
                led_lap = 1'b0;
                led_alarm = led_alarm_cooktimer;
                led_start = led_start_cooktimer;
            end
            default: begin
                com_selected = 4'b0000;
                seg_7_selected = 8'b00000000;
                led_lap = 1'b0;
                led_alarm = 1'b0;
                led_start = 1'b0;
            end
        endcase
    end

    assign com = com_selected;
    assign seg_7 = seg_7_selected;

endmodule

















