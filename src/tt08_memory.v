`default_nettype none

module tta_cartridge_memory (
    input wire clk, rst_n,
    input wire cpu_cs_n, cpu_sck, cpu_request_safe,
    input wire [3:0] cpu_out, cpu_oe,
    output wire cpu_run,
    input wire host_valid, host_write,
    input wire [19:0] host_addr,
    input wire [31:0] host_wdata,
    output wire host_ready,
    output wire host_result_ready,
    output reg host_error,
    output reg [31:0] host_rdata,
    input wire scan_valid, scan_fetching, scan_accept,
    input wire [19:0] scan_addr,
    output wire scan_ready, scan_error,
    output wire [31:0] scan_rdata,
    input wire [3:0] dq_in,
    output wire [3:0] dq_out, dq_oe,
    output wire flash_cs_n, ram_cs_n, sck
);
    reg active, scan_owner, cooldown, host_done;
    wire grant, flash_available;
    wire p_ready, p_error, p_initializing, p_cs, p_sck;
    wire [31:0] p_rdata;
    wire [3:0] p_out, p_oe;
    // Prepare the response before the late run fence; consume only after it.
    assign host_result_ready = rst_n && host_valid && host_done && flash_available;
    assign host_ready = host_result_ready && cpu_run;
    assign scan_ready = active && scan_owner && p_ready && scan_accept;
    assign scan_error = p_error;
    assign scan_rdata = p_rdata;
    wire session_done = grant && !active && !cooldown &&
        !scan_fetching && (host_done || !host_valid) && p_cs;
    tta_psram_quad psram (
        .clk(clk), .rst_n(rst_n), .valid(active),
        .write(!scan_owner && host_write),
        .addr({4'b0, scan_owner ? scan_addr : host_addr}),
        .wdata(host_wdata), .ready(p_ready), .error(p_error), .rdata(p_rdata),
        .cs_n(p_cs), .sck(p_sck), .dq_in(dq_in), .dq_out(p_out), .dq_oe(p_oe),
        .initializing(p_initializing)
    );
    tta_cartridge_handoff handoff (
        .clk(clk), .rst_n(rst_n), .cpu_cs_n(cpu_cs_n), .cpu_sck(cpu_sck),
        .cpu_out(cpu_out), .cpu_oe(cpu_oe), .cpu_request_safe(cpu_request_safe),
        .psram_request(scan_fetching || (host_valid && !host_done)),
        .psram_finished(session_done), .psram_initializing(p_initializing),
        .psram_cs_n(p_cs), .psram_sck(p_sck), .psram_out(p_out), .psram_oe(p_oe),
        .cpu_run(cpu_run), .psram_grant(grant), .flash_available(flash_available),
        .flash_cs_n(flash_cs_n), .ram_cs_n(ram_cs_n), .sck(sck),
        .dq_out(dq_out), .dq_oe(dq_oe),
        .handoffs(), .restarts(), .paused_clocks(), .recovery_clocks()
    );
    always @(posedge clk) begin
        if (!rst_n) begin
            active <= 0; scan_owner <= 0; cooldown <= 0; host_done <= 0;
            host_error <= 0; host_rdata <= 0;
        end else begin
            if (host_ready) host_done <= 0;
            if (cooldown) cooldown <= 0;
            if (active && p_ready && (!scan_owner || scan_accept)) begin
                active <= 0; cooldown <= 1;
                if (!scan_owner) begin
                    host_done <= 1; host_error <= p_error; host_rdata <= p_rdata;
                end
            end else if (!active && !cooldown && grant) begin
                if (scan_valid || (host_valid && !host_done && !scan_fetching)) begin
                    active <= 1; scan_owner <= scan_valid;
                end
            end
        end
    end
endmodule
`default_nettype wire
