`timescale 1ns/1ps
`default_nettype none

// Experimental QSPI=3 stream suspension. No instruction/data cache:
// only command/address and serial-position state survives a handoff.
module tta_cartridge_handoff (
    input wire clk,rst_n,
    input wire cpu_cs_n,cpu_sck,
    input wire [3:0] cpu_out,cpu_oe,
    input wire cpu_request_safe,
    input wire psram_request,psram_finished,psram_initializing,
    input wire psram_cs_n,psram_sck,
    input wire [3:0] psram_out,psram_oe,
    output wire cpu_run,
    output wire psram_grant,
    output wire flash_available,
    output wire flash_cs_n,ram_cs_n,sck,
    output wire [3:0] dq_out,dq_oe,
    output reg [31:0] handoffs,restarts,paused_clocks,recovery_clocks
);
    localparam BOOT=0,FLASH=1,QUIESCE=2,RELEASE=3,RAM=4,
        TURN=5,HEADER=6,PRIME=7,RESUME=8;
    reg [3:0] state;
    reg run_cpu;
    reg previous_cs,previous_sck;
    reg [5:0] logical_edges;
    reg [23:0] base_address;
    reg [23:0] nibble_count;
    reg [23:0] resume_address;
    reg resume_odd,resume_selected;
    reg [5:0] header_edges;
    reg [2:0] gap;
    reg restart_sck,restart_cs;
    reg [3:0] restart_out,restart_oe;
    wire restart_owner=state==HEADER || state==PRIME || state==RESUME;
    wire ram_owner=state==BOOT || state==RAM;
    assign cpu_run=run_cpu && (!cpu_cs_n || flash_high_cycles==4);
    assign psram_grant=state==RAM;
    assign flash_available=state==FLASH;
    assign flash_cs_n=restart_owner ? restart_cs :
        state==FLASH || state==QUIESCE ? cpu_cs_n : 1'b1;
    assign ram_cs_n=ram_owner ? psram_cs_n : 1'b1;
    assign sck=restart_owner ? restart_sck :
        ram_owner ? psram_sck :
        state==FLASH || state==QUIESCE ? cpu_sck : 1'b0;
    wire [3:0] host_out=restart_owner ? restart_out : cpu_out;
    wire [3:0] host_oe=restart_owner ? restart_oe :
        (state==FLASH || state==QUIESCE) ? cpu_oe : 4'b0;
    assign dq_out=ram_owner ? psram_out : host_out;
    assign dq_oe=ram_owner ? psram_oe : host_oe;
    // Logical CPU CS history can predate restoration of the physical pin.
    reg [2:0] flash_high_cycles;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) flash_high_cycles<=0;
        else if(!flash_cs_n) flash_high_cycles<=0;
        else if(flash_high_cycles<4) flash_high_cycles<=flash_high_cycles+1'b1;
    end
    always @(negedge clk or negedge rst_n) begin
        if(!rst_n) begin
            run_cpu<=0;previous_cs<=1;previous_sck<=0;
            logical_edges<=0;base_address<=0;nibble_count<=0;
        end else begin
            previous_cs<=cpu_cs_n;previous_sck<=cpu_sck;
            if(cpu_cs_n) begin logical_edges<=0;nibble_count<=0;end
            else begin
                if(previous_cs) begin logical_edges<=0;base_address<=0;nibble_count<=0;end
                if(cpu_sck && !previous_sck) begin
                    if(logical_edges<20) logical_edges<=logical_edges+1'b1;
                    if(logical_edges>=8 && logical_edges<14)
                        base_address<={base_address[19:0],cpu_out};
                end
                if(!cpu_sck && previous_sck && logical_edges>=20)
                    nibble_count<=nibble_count+1'b1;
            end
            if(state==BOOT) run_cpu<=0;
            else if(state==FLASH) run_cpu<=1;
            else if(state==QUIESCE && !cpu_sck && cpu_request_safe &&
                    (cpu_cs_n || (logical_edges==20 && nibble_count!=0)))
                run_cpu<=0;
            else if(state==RESUME && gap==0) run_cpu<=1;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=BOOT;gap<=0;header_edges<=0;
            restart_sck<=0;restart_cs<=1;restart_out<=0;restart_oe<=0;
            resume_address<=0;resume_odd<=0;resume_selected<=0;
            handoffs<=0;restarts<=0;paused_clocks<=0;recovery_clocks<=0;
        end else begin
            if(!run_cpu) paused_clocks<=paused_clocks+1'b1;
            if(state==TURN || state==HEADER || state==PRIME || state==RESUME)
                recovery_clocks<=recovery_clocks+1'b1;
            case(state)
                BOOT: if(!psram_initializing && psram_cs_n) state<=FLASH;
                FLASH: if(psram_request) state<=QUIESCE;
                QUIESCE: if(!run_cpu) begin
                    resume_selected<=!cpu_cs_n;
                    resume_address<=nibble_count;
                    handoffs<=handoffs+1'b1;gap<=2;state<=RELEASE;
                end
                RELEASE: begin
                    // Reuse the guard clocks for full-cycle resume arithmetic.
                    // A selected stream is quiesced only with a nonzero count.
                    if(gap==2) resume_address<=resume_address-1'b1;
                    if(gap==1) begin
                        resume_odd<=resume_address[0];
                        resume_address<=base_address+{1'b0,resume_address[23:1]};
                    end
                    if(gap==0) state<=RAM;else gap<=gap-1'b1;
                end
                RAM: if(psram_finished && psram_cs_n) begin gap<=2;state<=TURN;end
                TURN: if(gap!=0) gap<=gap-1'b1;
                    else if(resume_selected) begin
                        restart_cs<=0;restart_sck<=0;restart_oe<=1;
                        restart_out<=4'b0001;header_edges<=0;
                        restarts<=restarts+1'b1;state<=HEADER;
                    end else begin gap<=1;state<=RESUME;end
                HEADER: begin
                    restart_sck<=!restart_sck;
                    if(!restart_sck) header_edges<=header_edges+1'b1;
                    else if(header_edges==20) begin
                        // The final header falling edge launches the first data nibble.
                        if(resume_odd) state<=PRIME;
                        else begin gap<=2;state<=RESUME;end
                    end else if(header_edges<8) begin
                        restart_out<=(8'heb >> (7-header_edges)) & 1'b1;
                        restart_oe<=1;
                    end else if(header_edges<14) begin
                        restart_out<=resume_address >> ((13-header_edges)*4);
                        restart_oe<=15;
                    end else if(header_edges<16) begin restart_out<=0;restart_oe<=15;end
                    else begin restart_out<=0;restart_oe<=0;end
                end
                PRIME: begin
                    restart_sck<=!restart_sck;
                    if(restart_sck) begin gap<=2;state<=RESUME;end
                end
                RESUME: if(gap!=0) gap<=gap-1'b1;
                    else if(run_cpu) state<=FLASH;
                default: state<=BOOT;
            endcase
        end
    end
endmodule
`default_nettype wire
